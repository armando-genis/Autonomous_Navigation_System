#ifndef TARGET_WAYPOINT_INDEX_NODE_H
#define TARGET_WAYPOINT_INDEX_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/int32.hpp>
#include <tf2/time.h>
#include <Eigen/Dense>

using namespace std;

class target_waypoint_index_node : public rclcpp::Node
{
private:
    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string yellow = "\033[1;33m";
    string purple = "\033[1;35m";
    string reset = "\033[0m";

    // Variables from parameters
    double lookahead_min = 0.0; // [m]
    double lookahead_max = 0.0; // [m]
    double mps_alpha = 0.0;     // [m/s]
    double mps_beta = 0.0;      // [m/s]

    // Variables of the car position
    double current_yaw_ = 0.0; // [rad]
    double current_x_ = 0.0;   // [m]
    double current_y_ = 0.0;   // [m]
    double current_z_ = 0.0;
    double current_velocity_ = 0; // [m/s]

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // Waypoints
    vector<Eigen::VectorXd> waypoints; // [x, y, z, yaw]

    // waypoints variable for the computation
    bool waypoints_in_loop = false;
    int closest_waypoint = 0;
    int average_distance_count = 0;
    float average_distance = 0.0;
    float maximum_distance = 0.0;
    size_t target_waypoint = 0;

    rclcpp::Time last_time_;

    // Subscriptions & Publishers
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_subscription_;

    // for debugging
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_pub_;
    void marketTargetWaypoint();

    // functions
    void pub_callback();
    void getCurrentRobotState();
    void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg);
    double getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2);
    double LookAheadDistance();
    double getDistanceFromOdom(Eigen::VectorXd wapointPoint);
    void waypointsComputation();

public:
    target_waypoint_index_node(/* args */);
    ~target_waypoint_index_node();
};

#endif // TARGET_WAYPOINT_INDEX_NODE_H