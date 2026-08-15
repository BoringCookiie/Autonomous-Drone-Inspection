#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path = []
        self.state = State()
        self.pose = None
        self.battery = None
        self.low_battery = False
        self.taking_off = True
        
        # QoS for MAVROS topics (Best Effort)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Use /uas1 namespace
        self.sp_pub = self.create_publisher(PoseStamped, '/uas1/setpoint_position/local', 10)
        self.create_subscription(State, '/uas1/state', self.state_cb, 10)
        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.create_subscription(PoseStamped, '/uas1/local_position/pose', self.pose_cb, qos_best_effort)
        self.create_subscription(BatteryState, '/uas1/battery', self.battery_cb, qos_best_effort)
        
        self.mode_cli = self.create_client(SetMode, '/uas1/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/uas1/cmd/arming')
        
        self.create_timer(0.05, self.timer_cb)
        self.get_logger().info("Path Follower with Takeoff logic ready.")

    def state_cb(self, msg):
        if msg.mode != self.state.mode:
            self.get_logger().info(f"Mode changed to: {msg.mode}")
        if msg.armed != self.state.armed:
            self.get_logger().info(f"Arming state changed to: {msg.armed}")
        self.state = msg

    def pose_cb(self, msg): self.pose = msg
    def battery_cb(self, msg):
        self.battery = msg
        if msg.percentage < 0.2 and not self.low_battery:
            self.get_logger().error(f"LOW BATTERY: {msg.percentage*100}%! Returning to home...")
            self.low_battery = True
            self.path = [] 

    def path_cb(self, msg): 
        if self.low_battery:
            return
        self.path = [p.pose.position for p in msg.poses]
        self.get_logger().info(f"Received path with {len(self.path)} points.")

    def timer_cb(self):
        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.header.stamp = self.get_clock().now().to_msg()
        
        if self.low_battery:
            # Emergency RTH to (0,0,1.5)
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = 0.0, 0.0, 1.5
            if self.pose:
                dist = np.linalg.norm([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z - 1.5])
                if dist < 0.5:
                    self.get_logger().info("Home reached. Landing...")
                    req = SetMode.Request(custom_mode="AUTO.LAND")
                    self.mode_cli.call_async(req)
        elif self.taking_off:
            # Takeoff to 1.5m
            sp.pose.position.x = 0.0
            sp.pose.position.y = 0.0
            sp.pose.position.z = 1.5
            if self.pose:
                self.get_logger().info(f"Taking off... Current Z: {self.pose.pose.position.z:.2f}", throttle_duration_sec=2.0)
                if self.pose.pose.position.z > 1.2:
                    self.get_logger().info("Takeoff complete. Ready for path.")
                    self.taking_off = False
        elif self.path:
            target = self.path[0]
            sp.pose.position = target
            if self.pose:
                dist = np.linalg.norm([target.x-self.pose.pose.position.x, target.y-self.pose.pose.position.y, target.z-self.pose.pose.position.z])
                if dist < 0.5:
                    if len(self.path) > 1:
                        self.path.pop(0)
                        self.get_logger().info(f"Reached point. {len(self.path)} points remaining.")
                    else:
                        self.path = []
                        self.get_logger().info("Goal reached! Hovering.")
        elif self.pose:
            # Hover at current X, Y but fixed altitude (1.5m)
            sp.pose.position.x = self.pose.pose.position.x
            sp.pose.position.y = self.pose.pose.position.y
            sp.pose.position.z = 1.5
            self.get_logger().info("No path, hovering at (X, Y, 1.5m)...", throttle_duration_sec=5.0)
        else:
            return
        
        self.sp_pub.publish(sp)
        
        # Auto OFFBOARD and Arming (only if not landing)
        # We maintain OFFBOARD while armed and not in LAND mode
        if self.state.mode != "AUTO.LAND" and self.state.armed:
            if self.state.mode != "OFFBOARD":
                self.get_logger().info(f"Requesting OFFBOARD mode (current: {self.state.mode})...", throttle_duration_sec=2.0)
                req = SetMode.Request(custom_mode="OFFBOARD")
                self.mode_cli.call_async(req)
        elif not self.state.armed and (self.path or self.taking_off):
            self.get_logger().info("Requesting Arming...", throttle_duration_sec=2.0)
            req = CommandBool.Request(value=True)
            self.arm_cli.call_async(req)

def main():
    rclpy.init(); rclpy.spin(PathFollower()); rclpy.shutdown()

if __name__ == '__main__': main()
