
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

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>


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


class LidarPathToOdom : public rclcpp::Node
{
private:
    /* data */

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    nav_msgs::msg::Odometry convertPathToOdometry(const nav_msgs::msg::Path::SharedPtr path);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
public:
    LidarPathToOdom(/* args */);
    ~LidarPathToOdom();
};

LidarPathToOdom::LidarPathToOdom(/* args */): Node("lidar_path_to_odom")
{
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&LidarPathToOdom::pathCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_lidar", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_path_to_odom initialized.\033[0m");
}

LidarPathToOdom::~LidarPathToOdom()
{
}

void LidarPathToOdom::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom = convertPathToOdometry(msg);
    odom_pub_->publish(odom);
}

nav_msgs::msg::Odometry LidarPathToOdom::convertPathToOdometry(const nav_msgs::msg::Path::SharedPtr path)
{
    nav_msgs::msg::Odometry odom;
    odom.header = path->header;
    odom.child_frame_id = "map";
    odom.header.frame_id = "map";
    // cout << "x: " << path->poses.back().pose.position.x << " y: " << path->poses.back().pose.position.y << endl;
    
    odom.pose.pose.position = path->poses.back().pose.position;
    odom.pose.pose.orientation = path->poses.back().pose.orientation;
    return odom;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPathToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}