#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import heapq
import time

class Planner3D(Node):
    def __init__(self):
        super().__init__('planner_3d')
        self.res = 0.2
        self.declare_parameter('inflation_radius', 2)  # cells (1 = 0.2m, 2 = 0.4m) - increased for maze wall clearance
        self.declare_parameter('max_abs_coordinate_m', 30.0)
        self.declare_parameter('max_obstacle_z_m', 8.0)
        self.declare_parameter('max_obstacle_points', 200000)
        self.declare_parameter('max_search_nodes', 400000)
        self.declare_parameter('search_margin_m', 6.0)
        self.declare_parameter('min_flight_z_m', 1.5)
        self.declare_parameter('max_flight_z_m', 2.5)
        self.declare_parameter('path_horizon_m', 2.0)
        self.max_abs_coordinate = float(self.get_parameter('max_abs_coordinate_m').value)
        self.inflation_radius = int(self.get_parameter('inflation_radius').value)
        self.max_obstacle_z = float(self.get_parameter('max_obstacle_z_m').value)
        self.max_obstacle_points = int(self.get_parameter('max_obstacle_points').value)
        self.max_search_nodes = int(self.get_parameter('max_search_nodes').value)
        self.search_margin_cells = max(1, int(
            float(self.get_parameter('search_margin_m').value) / self.res
        ))
        self.min_flight_z_cell = round(
            float(self.get_parameter('min_flight_z_m').value) / self.res
        )
        self.max_flight_z_cell = round(
            float(self.get_parameter('max_flight_z_m').value) / self.res
        )
        self.path_horizon_m = float(self.get_parameter('path_horizon_m').value)
        self.obstacles = set()
        self.pose = None
        self._map_received_at = 0.0
        self._last_map_frame = ''
        self._last_goal = None
        self._active_goal_pose = None
        self._last_plan_at = 0.0
        self._planning = False
        self.declare_parameter('replan_period_s', 3.0)
        self.replan_period_s = float(self.get_parameter('replan_period_s').value)

        # FIFO queue of revisit Pose objects (from /planner/revisit_waypoints PoseArrays)
        self._revisit_queue: list = []
        self._revisit_active = False  # True while a revisit leg is being executed

        # QoS for MAVROS topics (Best Effort)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pc_sub = self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.pc_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/uas1/local_position/pose', self.pose_cb, qos_best_effort)
        self.goal_sub = self.create_subscription(PoseStamped, '/navigation/goal', self.goal_cb, 10)
        self.revisit_sub = self.create_subscription(
            PoseArray, '/planner/revisit_waypoints', self.revisit_cb, 10
        )
        # Listen to path_follower status so we can dequeue the next revisit leg
        self.status_sub = self.create_subscription(
            String, '/path_follower/status', self._follower_status_cb, 10
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.get_logger().info("Planner 3D (A* with Inflation, FIFO revisit queue) initialized.")

    def pc_cb(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().warn(
                f"Ignoring obstacle map in frame {msg.header.frame_id!r}; expected 'odom'.",
                throttle_duration_sec=5.0,
            )
            return
        new_obs = set()
        accepted = 0
        for p in pc2.read_points(msg, skip_nans=True):
            x, y, z = (float(p[0]), float(p[1]), float(p[2]))
            if not all(np.isfinite((x, y, z))):
                continue
            if z < 0.15 or z > self.max_obstacle_z:
                continue  # Ignore ground and implausible transformed returns.
            if max(abs(x), abs(y)) > self.max_abs_coordinate:
                continue
            accepted += 1
            if accepted > self.max_obstacle_points:
                break
            ox, oy, oz = round(x/self.res), round(y/self.res), round(z/self.res)
            # Inflate
            for dx in range(-self.inflation_radius, self.inflation_radius+1):
                for dy in range(-self.inflation_radius, self.inflation_radius+1):
                    for dz in range(-self.inflation_radius, self.inflation_radius+1):
                        new_obs.add((ox+dx, oy+dy, oz+dz))
        self.obstacles = new_obs
        self._map_received_at = time.monotonic()
        self._last_map_frame = msg.header.frame_id
        self.get_logger().info(
            f'Obstacle map accepted: {accepted} points, {len(new_obs)} inflated cells.',
            throttle_duration_sec=5.0,
        )
        if (
            self._active_goal_pose is not None
            and self.pose is not None
            and not self._planning
            and time.monotonic() - self._last_plan_at >= self.replan_period_s
        ):
            # A depth camera observes the maze incrementally. Replan from the
            # current pose after new occupied cells arrive instead of trusting
            # a one-shot path through still-unseen obstacles.
            self.plan(self._active_goal_pose)

    def pose_cb(self, msg): self.pose = msg.pose

    def goal_cb(self, msg):
        if msg.header.frame_id not in ('', 'odom', 'map'):
            self.get_logger().error(
                f"Rejecting goal in unsupported frame {msg.header.frame_id!r}; use odom."
            )
            return
        goal_key = (
            round(float(msg.pose.position.x), 3),
            round(float(msg.pose.position.y), 3),
            round(float(msg.pose.position.z), 3),
        )
        if goal_key == self._last_goal:
            self.get_logger().info(
                f'Ignoring duplicate goal {goal_key}; existing plan is authoritative.'
            )
            return
        self._last_goal = goal_key
        self._active_goal_pose = msg.pose
        if not self.pose:
            self.get_logger().warn("Current pose unknown, waiting...")
            return
        self.plan(msg.pose)

    def revisit_cb(self, msg):
        """
        Accumulate revisit poses into a FIFO queue.
        Does NOT plan immediately — prevents the 'last path wins' overwrite bug.
        The next pose is dequeued and planned when path_follower reports
        COVERAGE_DONE or REVISIT_LEG_DONE on /path_follower/status.
        """
        if not self.pose:
            self.get_logger().warn("Current pose unknown; queuing revisit poses anyway.")
        for pose in msg.poses:
            self._revisit_queue.append(pose)
            self.get_logger().info(
                f"Revisit pose queued ({len(self._revisit_queue)} total in queue)."
            )

    def _follower_status_cb(self, msg):
        """
        React to path_follower status signals:
          COVERAGE_DONE  — primary lawnmower pass finished; start first revisit leg (if any).
          REVISIT_LEG_DONE — a revisit leg finished; plan the next one (if any).
        """
        status = msg.data
        if status in ('COVERAGE_DONE', 'REVISIT_LEG_DONE'):
            self._plan_next_revisit()

    def _plan_next_revisit(self):
        """Pop the next revisit pose from the queue and run A* for it."""
        if not self._revisit_queue:
            self.get_logger().info("Revisit queue empty — no more revisit legs.")
            return
        if not self.pose:
            self.get_logger().warn("Pose unavailable; cannot plan revisit leg yet.")
            return
        next_pose = self._revisit_queue.pop(0)
        self.get_logger().info(
            f"Planning revisit leg "
            f"({next_pose.position.x:.2f}, {next_pose.position.y:.2f}, {next_pose.position.z:.2f}), "
            f"{len(self._revisit_queue)} leg(s) remaining."
        )
        self._active_goal_pose = None
        self.plan(next_pose)

    def plan(self, goal_pose):
        if self._planning:
            return
        self._planning = True
        self._last_plan_at = time.monotonic()
        try:
            self._plan_impl(goal_pose)
        finally:
            self._planning = False

    def _plan_impl(self, goal_pose):
        start = (
            round(self.pose.position.x / self.res),
            round(self.pose.position.y / self.res),
            max(self.min_flight_z_cell, round(self.pose.position.z / self.res)),
        )
        goal = (round(goal_pose.position.x/self.res), round(goal_pose.position.y/self.res), round(goal_pose.position.z/self.res))
        
        self.get_logger().info(f"Planning attempt: Start {start} -> Goal {goal} (Obstacles: {len(self.obstacles)})")
        
        start = self._nearest_free(start, 'start')
        goal = self._nearest_free(goal, 'goal')
        if start is None or goal is None:
            self.get_logger().error('No free start/goal cell exists in the validated map.')
            return

        frontier = [(0, start)]
        came_from = {start: None}
        cost = {start: 0}
        min_bound = [
            min(start[i], goal[i]) - self.search_margin_cells for i in range(3)
        ]
        max_bound = [
            max(start[i], goal[i]) + self.search_margin_cells for i in range(3)
        ]
        # Never generate a route through the ground. The previous planner
        # ignored ground returns and consequently selected z < 0 waypoints.
        min_bound[2] = max(min_bound[2], self.min_flight_z_cell)
        max_bound[2] = min(max_bound[2], self.max_flight_z_cell)
        expanded = 0

        # 26-connectivity
        neighbors = []
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                for dz in [-1,0,1]:
                    if dx==0 and dy==0 and dz==0: continue
                    neighbors.append((dx,dy,dz))

        while frontier:
            curr = heapq.heappop(frontier)[1]
            expanded += 1
            if expanded > self.max_search_nodes:
                self.get_logger().error(
                    f'A* aborted at {expanded} nodes; map or frame contract is invalid.'
                )
                break
            if np.linalg.norm(np.array(curr)-np.array(goal)) < 1.0: # Close enough
                goal = curr
                break
            
            for d in neighbors:
                nxt = (curr[0]+d[0], curr[1]+d[1], curr[2]+d[2])
                if any(nxt[i] < min_bound[i] or nxt[i] > max_bound[i] for i in range(3)):
                    continue
                if nxt in self.obstacles: continue
                
                step_cost = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
                new_cost = cost[curr] + step_cost
                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    priority = new_cost + np.linalg.norm(np.array(nxt)-np.array(goal))
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = curr

        if goal in came_from:
            path = Path()
            # 'odom': this stack's world frame (tf_bridge chain); avoids
            # depending on rtabmap's wall-clock-stamped map->odom transform
            path.header.frame_id = "odom"
            path.header.stamp = self.get_clock().now().to_msg()
            curr = goal
            while curr:
                ps = PoseStamped()
                ps.header.frame_id = 'odom'
                ps.header.stamp = path.header.stamp
                ps.pose.position.x = float(curr[0]*self.res)
                ps.pose.position.y = float(curr[1]*self.res)
                ps.pose.position.z = float(curr[2]*self.res)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
                curr = came_from[curr]
            path.poses.reverse()
            if self.path_horizon_m > 0.0 and len(path.poses) > 1:
                horizon = [path.poses[0]]
                distance = 0.0
                for previous, current in zip(path.poses, path.poses[1:]):
                    a = previous.pose.position
                    b = current.pose.position
                    distance += math.sqrt(
                        (b.x - a.x) ** 2 +
                        (b.y - a.y) ** 2 +
                        (b.z - a.z) ** 2
                    )
                    horizon.append(current)
                    if distance >= self.path_horizon_m:
                        break
                path.poses = horizon
            self.path_pub.publish(path)
            self.get_logger().info(
                f"Path published: {len(path.poses)} points "
                f"(receding horizon={self.path_horizon_m:.1f}m)."
            )
        else:
            self.get_logger().error(f"No path found after checking {len(came_from)} nodes!")

    def _nearest_free(self, cell, label):
        if cell not in self.obstacles:
            return cell
        self.get_logger().warn(
            f'{label.upper()} POSITION {cell} IS IN OBSTACLE; searching nearby free cells.'
        )
        for radius in range(1, 11):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        candidate = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                        if candidate not in self.obstacles:
                            candidates.append(candidate)
            if candidates:
                selected = min(candidates, key=lambda c: np.linalg.norm(np.array(c) - np.array(cell)))
                self.get_logger().info(f'{label.capitalize()} snapped to free cell {selected}.')
                return selected
        return None

def main():
    rclpy.init(); rclpy.spin(Planner3D()); rclpy.shutdown()

if __name__ == '__main__': main()
