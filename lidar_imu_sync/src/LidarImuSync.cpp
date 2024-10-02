
#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

using namespace std;

class LidarImuSync : public rclcpp::Node
{
private:
    /* data */
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using ImuMsg = sensor_msgs::msg::Imu;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, PointCloudMsg>;

    // main callback function
    void points_imuCallback(const ImuMsg::ConstSharedPtr &msg1, const PointCloudMsg::ConstSharedPtr &msg2);

    // message filter synchronizer object
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>>
        point_cloud_imu_synchronizer_;

    // final fused point cloud publisher object
    rclcpp::Publisher<PointCloudMsg>::SharedPtr fused_point_cloud_imu_publisher_;
    // final imu publisher
    rclcpp::Publisher<ImuMsg>::SharedPtr fused_imu_publisher_;

    // need two subscribers for lidar and imu
    std::unique_ptr<message_filters::Subscriber<PointCloudMsg>>
        lidar_subscriber_;
    std::unique_ptr<message_filters::Subscriber<ImuMsg>>
        imu_subscriber_;

public:
    LidarImuSync(/* args */);
    ~LidarImuSync();
};

LidarImuSync::LidarImuSync(/* args */) : Node("lidar_imu_sync")
{

    // tf2 buffer
    tf2_ros::Buffer tf2_buffer(this->get_clock());
    // tf2 listener
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    fused_point_cloud_imu_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_fusion", 10);
    fused_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_fusion", 10);

    // initialize front and rear lidar subscriber objects
    lidar_subscriber_ =
        std::make_unique<message_filters::Subscriber<PointCloudMsg>>(this,
                                                                     "/points_rotated");
    //  "/velodyne_points");

    imu_subscriber_ =
        std::make_unique<message_filters::Subscriber<ImuMsg>>(this,
                                                              "/vectornav/imu");

    // initialize message filter stuffs
    point_cloud_imu_synchronizer_ =
        std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *imu_subscriber_, *lidar_subscriber_);

    point_cloud_imu_synchronizer_->registerCallback(
        std::bind(&LidarImuSync::points_imuCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_imu_sync_node initialized.\033[0m");
}

LidarImuSync::~LidarImuSync()
{
}

void LidarImuSync::points_imuCallback(const ImuMsg::ConstSharedPtr &msg1, const PointCloudMsg::ConstSharedPtr &msg2)
{
    RCLCPP_INFO(this->get_logger(), "\033[1;34m -> lidar & imu data received <- \033[0m");

    // get the imu data
    sensor_msgs::msg::Imu imu_data = *msg1;
    // get the point cloud data
    sensor_msgs::msg::PointCloud2 point_cloud_data = *msg2;

    // publish the imu data
    fused_imu_publisher_->publish(imu_data);
    // publish the point cloud data
    fused_point_cloud_imu_publisher_->publish(point_cloud_data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarImuSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}