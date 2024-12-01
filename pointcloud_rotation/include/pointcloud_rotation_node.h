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
#include <pcl/filters/crop_box.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

class pointcloud_rotation_node : public rclcpp::Node
{
private:
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    // ==============  variables for ground remove  ==============
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    float th_seeds_;
    float th_dist_;
    float sensor_height_;

    struct Model
    {
        Eigen::MatrixXf normal;
        double d = 0.;
    };

    Model estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &seed_points);

    // ==============  variables for voxel grid  ==============
    // voxelgrid resolution
    float voxel_leaf_size_x_ = 0.0;
    float voxel_leaf_size_y_ = 0.0;
    float voxel_leaf_size_z_ = 0.0;

    bool voxel_condition = false;

    //  ==================  variables for ROI boundaries  ==================
    double roi_max_x_ = 0.0; // FRONT THE CAR
    double roi_max_y_ = 0.0; // LEFT THE CAR
    double roi_max_z_ = 0.0; // UP THE VELODYNE

    double roi_min_x_ = 0.0; // RIGHT THE CAR
    double roi_min_y_ = 0.0; // BACK THE CAR
    double roi_min_z_ = 0.0; // DOWN THE VELODYNEs

    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

    // ==============  variables for pointcloud rotation  ==============

    float sensor_rotation_y_;
    Eigen::Matrix4f rotation_matrix_;

    // functions
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // subscriber & publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;

public:
    pointcloud_rotation_node(/* args */);
    ~pointcloud_rotation_node();
};

#endif // POINTCLOUD_ROTATION_NODE_H