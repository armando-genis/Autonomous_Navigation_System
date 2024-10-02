#include "pointcloud_rotation_node.h"

pointcloud_rotation_node::pointcloud_rotation_node(/* args */) : Node("pointcloud_rotation_node")
{
    this->declare_parameter("sensor_rotation_y_", 0.0);
    this->get_parameter("sensor_rotation_y_", sensor_rotation_y_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&pointcloud_rotation_node::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_rotated", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> sensor_rotation_y: %f \033[0m", sensor_rotation_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> pointcloud_rotation_node initialized.\033[0m");
}

pointcloud_rotation_node::~pointcloud_rotation_node()
{
}

void pointcloud_rotation_node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(0, 0) = cos(sensor_rotation_y_);
    transform(0, 2) = sin(sensor_rotation_y_);
    transform(2, 0) = -sin(sensor_rotation_y_);
    transform(2, 2) = cos(sensor_rotation_y_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform);

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*transformed_cloud, ground_msg);
    ground_msg.header.frame_id = msg->header.frame_id;
    ground_msg.header.stamp = msg->header.stamp;
    pub_->publish(ground_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_rotation_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}