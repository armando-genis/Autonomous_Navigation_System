#!/usr/bin/env python3

import os
import csv
import math 
import numpy as np
from scipy.interpolate import CubicSpline

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Vector3, Pose
from std_msgs.msg import Float32, Float64MultiArray, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

def get_curv_list(cs1, cs2, t):
    x_, y_ = cs1(t, nu=1), cs2(t, nu=1)
    x__, y__ = cs1(t, nu=2), cs2(t, nu=2)
    return (x_ * y__ - y_ * x__) / (x_**2 + y_**2)**1.5

def get_color(k):
    if abs(k) < 0.05:
        return ColorRGBA(r = 0.0, g = 1.0, b = 0.0, a = 1.0)
    elif abs(k) > 0.2:
        return ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0)
    else:
        return ColorRGBA(r = 1.0, g = 1.0, b = 0.0, a = 1.0)

def get_vel_setpoint(k):
    if abs(k) < 0.05: # almost straight
        return 1.0
    elif abs(k) > 0.2: # most curved
        return 0.2
    else: # in the middle
        return 0.6

class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self.parent_frame = 'odom'

        self.wp_sub_ = self.create_subscription(
            Path, '/sdv/guidance/key_waypoints', self.wp_callback, 10
        )
        self.path_pub_ = self.create_publisher(Path, "/sdv/guidance/reference_path", 10)
        # self.curvature_arr_pub_ = self.create_publisher(Float64MultiArray, "/sdv/guidance/path_curvature", 10)
        self.curvature_markers_pub_ = self.create_publisher(MarkerArray, "/sdv/guidance/curvature_markers", 10)

        self.timer = self.create_timer(1, self.timer_callback)

        self.path_ = Path()
        self.path_.header.frame_id = self.parent_frame
        self.path_.header.stamp = self.get_clock().now().to_msg()

        # self.curvature_arr = Float64MultiArray()
        self.path_ = Path()
        self.path_.header.frame_id = self.parent_frame
        self.path_.header.stamp = self.get_clock().now().to_msg()
        self.curvature_markers = MarkerArray()

    def timer_callback(self):
        self.path_pub_.publish(self.path_)
        # self.curvature_arr_pub_.publish(self.curvature_arr)
        self.curvature_markers_pub_.publish(self.curvature_markers)

    def wp_callback(self, msg):
        self.interpol(msg)

    def interpol(self, raw_msg):
        self.path_.poses.clear()
        self.curvature_markers.markers.clear()
        # self.curvature_arr.data = []

        x = []
        y = []
        for i in raw_msg.poses:
            x.append(i.pose.position.x)
            y.append(i.pose.position.y)

        t = np.linspace(0, 1, len(x))
        t2 = np.linspace(0, 1, len(x) * 100)
        cs1 = CubicSpline(t, np.array(x))
        cs2 = CubicSpline(t, np.array(y))
        k = get_curv_list(cs1, cs2, t2)

        for i in range(len(t2)):
            pose_stmpd = PoseStamped()
            pose_stmpd.pose.position.x = cs1(t2[i]) * 1.
            pose_stmpd.pose.position.y = cs2(t2[i]) * 1.
            pose_stmpd.pose.orientation.w = get_vel_setpoint(k[i])
            self.path_.poses.append(pose_stmpd)

            temp_marker = Marker(header = Header(
                frame_id = self.parent_frame), id = i, type = 2, action = 0,
                pose = pose_stmpd.pose,
                scale = Vector3(x = .1, y = .1, z = .1),
                color = get_color(k[i]),
                text = str(k[i]))
            self.curvature_markers.markers.append(temp_marker)
        
        # self.curvature_arr.data = list(k)
    
def main(args=None):
    rclpy.init(args=args)

    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()