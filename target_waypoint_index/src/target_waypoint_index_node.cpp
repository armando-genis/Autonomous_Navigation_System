#include "target_waypoint_index_node.h"

target_waypoint_index_node::target_waypoint_index_node(/* args */) : Node("target_waypoint_index_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    this->declare_parameter<double>("lookahead_min", 0.0);
    this->declare_parameter<double>("lookahead_max", 0.0);
    this->declare_parameter<double>("mps_alpha", 0.0);
    this->declare_parameter<double>("mps_beta", 0.0);

    this->get_parameter("lookahead_min", lookahead_min);
    this->get_parameter("lookahead_max", lookahead_max);
    this->get_parameter("mps_alpha", mps_alpha);
    this->get_parameter("mps_beta", mps_beta);

    // waypoints_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //     "/waypoints_routing", 10, std::bind(&target_waypoint_index_node::waypoints_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&target_waypoint_index_node::pub_callback, this));

    waypoints_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "path_waypoints", 10, std::bind(&target_waypoint_index_node::waypoints_callback, this, std::placeholders::_1));

    // for debugging
    target_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_waypoint_marker", 10);

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> target_waypoint_index_node initialized.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->lookahead_min: %f \033[0m", lookahead_min);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->lookahead_max: %f \033[0m", lookahead_max);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->mps_alpha: %f \033[0m", mps_alpha);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->mps_beta: %f \033[0m", mps_beta);
}

void target_waypoint_index_node::pub_callback()
{

    // Check if waypoints have been received
    if (waypoints.empty())
    {
        RCLCPP_WARN(this->get_logger(), "\033[1;33m----> No waypoints received yet.\033[0m");
        return;
    }

    if (this->get_clock()->now() - last_time_ > rclcpp::Duration(0, 100 * 1e6))
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;31m----> No waypoints received.\033[0m");
        return;
    }
    getCurrentRobotState();
    waypointsComputation();
    marketTargetWaypoint();
}

void target_waypoint_index_node::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        current_x_ = pose_tf.translation.x;
        current_y_ = pose_tf.translation.y;
        current_z_ = pose_tf.translation.z - 1.55;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

void target_waypoint_index_node::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    last_time_ = this->get_clock()->now();
    waypoints.clear();

    for (const auto &pose : msg->poses)
    {
        Eigen::VectorXd waypoint(4);
        waypoint(0) = pose.pose.position.x;
        waypoint(1) = pose.pose.position.y;
        waypoint(2) = pose.pose.position.z;

        tf2::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);

        // Extract yaw from the quaternion
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        waypoint(3) = yaw;

        waypoints.push_back(waypoint);
    }
}

double target_waypoint_index_node::getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2)
{
    double distance = sqrt(pow(point1(0) - point2(0), 2) + pow(point1(1) - point2(1), 2));
    return distance;
}

double target_waypoint_index_node::LookAheadDistance()
{
    double clamped_speed = std::clamp(current_velocity_, mps_alpha, mps_beta);
    return (lookahead_max - lookahead_min) / (mps_beta - mps_alpha) * (clamped_speed - mps_alpha) + lookahead_min;
}

double target_waypoint_index_node::getDistanceFromOdom(Eigen::VectorXd wapointPoint)
{
    double x1 = wapointPoint(0);
    double y1 = wapointPoint(1);
    double x2 = current_x_;
    double y2 = current_y_;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void target_waypoint_index_node::waypointsComputation()
{
    if (waypoints.empty())
    {
        std::cout << red << "No waypoints received" << reset << std::endl;
        return;
    }

    constexpr size_t first_wp = 0;
    const size_t last_wp = waypoints.size() - 1;
    constexpr double max_distance_threshold = 10.0;
    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;

    // loop closure is true, when the first and last waypoint are closer than 4.0 meters
    double distance = getDistance(waypoints[first_wp], waypoints[last_wp]);
    waypoints_in_loop = distance < 4.0;

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double smallest_curr_distance = std::numeric_limits<double>::max();

    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (smallest_curr_distance > curr_distance)
        {
            closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }

    if (average_distance_count == 0)
    {
        average_distance = smallest_curr_distance;
    }
    else
    {
        average_distance = (average_distance * average_distance_count + smallest_curr_distance) / (average_distance_count + 1);
    }
    average_distance_count++;

    // Updating maximum_distance
    if (maximum_distance < smallest_curr_distance && smallest_curr_distance < max_distance_threshold)
    {
        maximum_distance = smallest_curr_distance;
    }

    double lookahead_actual = LookAheadDistance();

    for (int i = closest_waypoint; i <= static_cast<int>(last_wp); i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (curr_distance > lookahead_actual && curr_distance < lookahead_actual + 8.0)
        {
            target_waypoint = i;
            break;
        }
        // set if is loop true and the current waypoint is the last waypoint set the target waypoint to the first waypoint
        if (waypoints_in_loop && i == static_cast<int>(last_wp))
        {
            target_waypoint = first_wp;
        }
        else
        {
            target_waypoint = last_wp;
            // finishedpath = true;
        }
        // target_waypoint = last_wp;
    }
}

void target_waypoint_index_node::marketTargetWaypoint()
{
    // Publish target waypoint marker

    if (waypoints.empty())
        return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "target_waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = waypoints[target_waypoint](0);
    marker.pose.position.y = waypoints[target_waypoint](1);
    marker.pose.position.z = current_z_;

    tf2::Quaternion q;
    q.setRPY(0, 0, waypoints[target_waypoint](3));
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = 1.1;
    marker.scale.y = 1.1;
    marker.scale.z = 1.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    target_waypoint_pub_->publish(marker);
}

target_waypoint_index_node::~target_waypoint_index_node()
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<target_waypoint_index_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}