#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 send_goal.py x y z")
        return

    rclpy.init()
    node = rclpy.create_node('goal_sender')
    goal_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(PoseStamped, '/navigation/goal', goal_qos)
    
    msg = PoseStamped()
    msg.header.frame_id = "odom"
    msg.pose.position.x = float(sys.argv[1])
    msg.pose.position.y = float(sys.argv[2])
    msg.pose.position.z = float(sys.argv[3])
    
    deadline = node.get_clock().now().nanoseconds + 10_000_000_000
    while pub.get_subscription_count() < 1 and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() < 1:
        print('ERROR: no planner subscribed to /navigation/goal')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Keep the goal alive long enough for DDS delivery, while planner_3d
    # suppresses duplicate coordinates and performs one authoritative plan.
    for _ in range(20):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        print(f"Goal sent: {sys.argv[1]}, {sys.argv[2]}, {sys.argv[3]}")
        rclpy.spin_once(node, timeout_sec=0.1)
        
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
