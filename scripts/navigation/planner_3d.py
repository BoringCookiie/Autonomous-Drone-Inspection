#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import heapq

class Planner3D(Node):
    def __init__(self):
        super().__init__('planner_3d')
        self.res = 0.2
        self.inflation_radius = 2  # nodes (0.4m)
        self.obstacles = set()
        self.pose = None
        
        # QoS for MAVROS topics (Best Effort)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.pc_sub = self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.pc_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/uas1/local_position/pose', self.pose_cb, qos_best_effort)
        self.goal_sub = self.create_subscription(PoseStamped, '/navigation/goal', self.goal_cb, 10)
        self.revisit_sub = self.create_subscription(
            PoseArray, '/planner/revisit_waypoints', self.revisit_cb, 10
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.get_logger().info("Planner 3D (A* with Inflation) initialized.")

    def pc_cb(self, msg):
        new_obs = set()
        for p in pc2.read_points(msg, skip_nans=True):
            if p[2] < 0.15: continue # Ignore ground
            ox, oy, oz = round(p[0]/self.res), round(p[1]/self.res), round(p[2]/self.res)
            # Inflate
            for dx in range(-self.inflation_radius, self.inflation_radius+1):
                for dy in range(-self.inflation_radius, self.inflation_radius+1):
                    for dz in range(-self.inflation_radius, self.inflation_radius+1):
                        new_obs.add((ox+dx, oy+dy, oz+dz))
        self.obstacles = new_obs

    def pose_cb(self, msg): self.pose = msg.pose

    def goal_cb(self, msg):
        if not self.pose:
            self.get_logger().warn("Current pose unknown, waiting...")
            return
        self.plan(msg.pose)

    def revisit_cb(self, msg):
        """Route inspection revisit poses through the same A* implementation."""
        if not self.pose:
            self.get_logger().warn("Current pose unknown; delaying revisit route")
            return
        for pose in msg.poses:
            self.get_logger().info("Planning an inspection revisit waypoint")
            self.plan(pose)

    def plan(self, goal_pose):
        start = (round(self.pose.position.x/self.res), round(self.pose.position.y/self.res), round(self.pose.position.z/self.res))
        goal = (round(goal_pose.position.x/self.res), round(goal_pose.position.y/self.res), round(goal_pose.position.z/self.res))
        
        self.get_logger().info(f"Planning attempt: Start {start} -> Goal {goal} (Obstacles: {len(self.obstacles)})")
        
        if start in self.obstacles:
            self.get_logger().warn(f"START POSITION {start} IS IN OBSTACLE! Path might be blocked.")
        
        if goal in self.obstacles:
            self.get_logger().error(f"GOAL POSITION {goal} IS IN OBSTACLE! Searching for nearest free node...")
            # Simple search for nearest free node
            found_free = False
            for r in range(1, 5):
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        for dz in range(-r, r+1):
                            candidate = (goal[0]+dx, goal[1]+dy, goal[2]+dz)
                            if candidate not in self.obstacles:
                                goal = candidate
                                found_free = True
                                break
                        if found_free: break
                    if found_free: break
                if found_free: break
            
            if found_free:
                self.get_logger().info(f"New reachable goal selected: {goal}")
            else:
                self.get_logger().error("Could not find free node near goal. Planning will likely fail.")

        frontier = [(0, start)]
        came_from = {start: None}
        cost = {start: 0}

        # 26-connectivity
        neighbors = []
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                for dz in [-1,0,1]:
                    if dx==0 and dy==0 and dz==0: continue
                    neighbors.append((dx,dy,dz))

        while frontier:
            curr = heapq.heappop(frontier)[1]
            if np.linalg.norm(np.array(curr)-np.array(goal)) < 1.0: # Close enough
                goal = curr
                break
            
            for d in neighbors:
                nxt = (curr[0]+d[0], curr[1]+d[1], curr[2]+d[2])
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
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            curr = goal
            while curr:
                ps = PoseStamped()
                ps.pose.position.x = float(curr[0]*self.res)
                ps.pose.position.y = float(curr[1]*self.res)
                ps.pose.position.z = float(curr[2]*self.res)
                path.poses.append(ps)
                curr = came_from[curr]
            path.poses.reverse()
            self.path_pub.publish(path)
            self.get_logger().info(f"Path published: {len(path.poses)} points.")
        else:
            self.get_logger().error(f"No path found after checking {len(came_from)} nodes!")

def main():
    rclpy.init(); rclpy.spin(Planner3D()); rclpy.shutdown()

if __name__ == '__main__': main()
