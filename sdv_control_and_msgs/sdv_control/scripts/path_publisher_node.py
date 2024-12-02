#!/usr/bin/env python3

import os
import csv
import math 
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class PathPublisherNode(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        parent_frame = 'odom'

        self.path_pub_ = self.create_publisher(Path, "/sdv/guidance/key_waypoints", 10)

        self.timer = self.create_timer(1, self.timer_callback)

        self.waypoints_file_ = os.path.join(
            get_package_share_directory('sdv_control'),
            'config',
            'sim3.csv'
        )

        self.path_ = Path()
        self.path_.header.frame_id = parent_frame
        self.path_.header.stamp = self.get_clock().now().to_msg()

        # # Static Path
        # for i in range(5):
        #     pose_stmpd = PoseStamped()
        #     pose_stmpd.header.frame_id = parent_frame
        #     pose_stmpd.pose.position.x = i * 1.
        #     pose_stmpd.pose.position.y = math.sin(i)
        #     self.path_.poses.append(pose_stmpd)

        # CSV Path
        with open(self.waypoints_file_, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                pose_stmpd = PoseStamped()
                pose_stmpd.header.frame_id = parent_frame
                pose_stmpd.pose.position.x = float(row[0]) + 80.67
                pose_stmpd.pose.position.y = float(row[1]) + 80.97
                self.path_.poses.append(pose_stmpd)
                print(row[0], row[1])

    def timer_callback(self):
        self.path_pub_.publish(self.path_)

def main(args=None):
    rclpy.init(args=args)

    path_publisher_node = PathPublisherNode()
    rclpy.spin(path_publisher_node)
    path_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()