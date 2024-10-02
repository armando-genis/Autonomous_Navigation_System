#ifndef POINTCLOUD_ROTATION_NODE_H
#define POINTCLOUD_ROTATION_NODE_H

#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

class pointcloud_rotation_node : public rclcpp::Node
{
private:
    // variables
    float sensor_rotation_y_;

    // functions
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // subscriber & publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

public:
    pointcloud_rotation_node(/* args */);
    ~pointcloud_rotation_node();
};

#endif // POINTCLOUD_ROTATION_NODE_H