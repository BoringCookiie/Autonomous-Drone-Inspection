#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 send_goal.py x y z")
        return

    rclpy.init()
    node = rclpy.create_node('goal_sender')
    pub = node.create_publisher(PoseStamped, '/navigation/goal', 10)
    
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = float(sys.argv[1])
    msg.pose.position.y = float(sys.argv[2])
    msg.pose.position.z = float(sys.argv[3])
    
    for _ in range(5):
        pub.publish(msg)
        print(f"Goal sent: {sys.argv[1]}, {sys.argv[2]}, {sys.argv[3]}")
        rclpy.spin_once(node, timeout_sec=0.2)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
