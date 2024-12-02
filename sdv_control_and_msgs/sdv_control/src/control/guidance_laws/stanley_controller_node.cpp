/** ----------------------------------------------------------------------------
 * @file: stanley_controller_node.cpp
 * @date: August 17, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Stanley Controller node.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "controllers/guidance_laws/stanley_controller.cpp"
#include "controllers/guidance_laws/stanley_controller.cpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class StanleyControllerNode : public rclcpp::Node
using namespace std::chrono_literals;

class StanleyControllerNode : public rclcpp::Node
{
    private:
        bool vel_msgs_received_{false};
        bool vehicle_pos_msgs_received_{false};
        bool vehicle_yaw_msgs_received_{false};
        bool new_path_arrived_{false};
        bool path_arrived_{false};
        bool nearest_waypoint_found_{false};

        std::unique_ptr<StanleyController> stanley_;

        /* Stanley Params */
        double k_{2};
        double k_soft_{1.1};
        double k_{2};
        double k_soft_{1.1};
        std::vector<double> DELTA_SAT_; // {max, min} steering in rads
        uint8_t precision_{10};

        /* Control signals */
        double vel_;
        std_msgs::msg::Float64 delta_;
        std_msgs::msg::Float64 delta_;
        std_msgs::msg::Float64 steering_setpoint_;
        std_msgs::msg::Float64 velocity_setpoint_;
        std_msgs::msg::Float64 velocity_setpoint_;
        const double delta_to_steer = 1 / 0.0658; // relation from delta to steering wheel angle

        /* Vehicle pose */
        std::vector<double> init_pose_ = {0,0,0};
        Point vehicle_pos_ = {0, 0};
        double psi_{0};

        /* Path */
        geometry_msgs::msg::Pose p1_, p2_;
        geometry_msgs::msg::Pose p1_, p2_;
        nav_msgs::msg::Path reference_path_;


        size_t waypoint_ = 0;
        size_t waypoint_base_ = 0;
        size_t path_length_;
        // double DISTANCE_VAL_ = 0.5;                // Meters
        double DISTANCE_VAL_ = 3;                // Meters
        // double DISTANCE_VAL_ = 0.5;                // Meters
        double DISTANCE_VAL_ = 3;                // Meters
        std::string parent_frame_;
        nav_msgs::msg::Path smooth_path_;

        const double kLookaheadDistance = 4.0;
        const double kBehindDistance = 1.0;
        const double kLookaheadDistance = 4.0;
        const double kBehindDistance = 1.0;

        nav_msgs::msg::Path current_ref_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_path_message;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;




        /* Publishers */
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr car_steering_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr car_steering_setpoint_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_setpoint_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr car_steering_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr car_steering_setpoint_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_setpoint_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_ref_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;


        /* Subscribers */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_velocity_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_velocity_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_to_follow_;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr lookahead_wp_sub_;
        

        void timer_callback(){
            velocity_setpoint_.data = 0.;
            current_ref_.poses.clear();
            if(path_arrived_ && this->get_clock()->now() - last_path_message < rclcpp::Duration(0, 100 * 1e6) ) {
                    geometry_msgs::msg::TransformStamped transform;
                    try {
                        transform = tf_buffer_->lookupTransform(
                            "map", "velodyne",
                            tf2::TimePointZero);
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Could not transform %s to %s: %s",
                            "map", "velodyne", ex.what());
                        return;
                    }
                    }

                    vehicle_pos_.x = transform.transform.translation.x;
                    vehicle_pos_.y = transform.transform.translation.y;

                    tf2::Quaternion quat;
                    tf2::fromMsg(transform.transform.rotation, quat);
                    double roll, pitch;
                    tf2::Matrix3x3(quat).getRPY(roll, pitch, psi_);
                    // psi_ = std::fmod((psi_+M_PI/2.0) + M_PI, 2*M_PI); // [-pi, pi]
                    // if(psi_ < 0.0){
                    //     psi_ += 2*M_PI;
                    // }
                    // psi_ -= M_PI;

                    // if(waypoint_base_ == -1){
                        p1_.position.x = vehicle_pos_.x;
                        p1_.position.y = vehicle_pos_.y;
                        p1_.position.z = transform.transform.translation.z - 1.45;
                    // } else {
                    //     p1_ = smooth_path_.poses[waypoint_base_].pose;
                    // }

                    // p2_ = smooth_path_.poses[waypoint_].pose;
                    // p2_.position.z+=0.0;
std::cout << "huh" << std::endl;
            geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
            pose_stamped_tmp_.header.frame_id = "map";
            pose_stamped_tmp_.pose = p1_;
                    current_ref_.poses.push_back(pose_stamped_tmp_);
            pose_stamped_tmp_.pose = p2_;
                    current_ref_.poses.push_back(pose_stamped_tmp_);


                    stanley_->calculateCrosstrackError(vehicle_pos_, 
                        Point{p1_.position.x, p1_.position.y}, 
                        Point{p2_.position.x, p2_.position.y}
                        );
                        std::cout << "huh" << std::endl;

                    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, psi: %f", 
                    vehicle_pos_.x, vehicle_pos_.y, psi_
                    );
                    stanley_->setYawAngle(psi_);
                    stanley_->calculateSteering(vel_, precision_);


                    delta_.data = stanley_->delta_;
                    steering_setpoint_.data = std::round(stanley_->delta_ * delta_to_steer * 0.8 * 100.0) / 100.0;
                    

                    car_steering_pub_->publish(steering_setpoint_);
                    car_steering_setpoint_pub_->publish(steering_setpoint_);
                    current_ref_pub_->publish(current_ref_);

                    velocity_setpoint_.data = 0.5;
                    // velocity_setpoint_.data = smooth_path_.poses[waypoint_].pose.orientation.w;

                    // // Find waypoint indexes
                    // double angle_diff{0};
                    // double dist{0};

                    // // Only check for next waypoint if we have more waypoints.
                    // if(waypoint_ < smooth_path_.poses.size()){
                    //     // Find farthest point from current posision, limited by lookahead.
                    //     std::size_t max_idx_ = waypoint_;
                    //     std::size_t min_idx_ = -1;
                    //     double max_distance_found_ = 0;
                    //     for(int i = waypoint_; i < smooth_path_.poses.size(); i++){
                    //         dist = distance(transform.transform.translation, smooth_path_.poses[i].pose.position);
                    //         angle_diff = get_angle_diff(transform.transform.translation, smooth_path_.poses[i].pose.position);

                    //         if(dist < kLookaheadDistance && dist > max_distance_found_ && ((i - max_idx_) < 10) && std::fabs(angle_diff) < 1.3){
                    //             max_idx_ = i;
                    //             max_distance_found_ = dist;
                    //         }
                    //     }
                    //     waypoint_ = max_idx_;
                    //     int i = waypoint_ - 1;
                    //     while(i > 0 && min_idx_ == -1){
                    //         dist = distance(transform.transform.translation, 
                    //             smooth_path_.poses[i].pose.position);

                    //         angle_diff = get_angle_diff(transform.transform.translation, 
                    //             smooth_path_.poses[i].pose.position);
                    //         if(std::fabs(angle_diff) > M_PI_2 && dist > kBehindDistance)
                    //             min_idx_ = i;
                    //         i--;
                    //     }
                    //     if(i == 0 && min_idx_ == -1)
                    //         waypoint_base_ = -1;
                    //     else
                    //         waypoint_base_ = min_idx_;
                        
                    //     if(waypoint_ == smooth_path_.poses.size() - 1 && std::fabs(distance(
                    //         transform.transform.translation, smooth_path_.poses[waypoint_].pose.position)) < 1)
                    //         waypoint_++;
                    // }
                    // RCLCPP_INFO(this->get_logger(), "wp: %d", 
                    // waypoint_
                    // );

            } else {
                    velocity_setpoint_.data = 0.;
                RCLCPP_INFO(this->get_logger(), "Waiting for reference path");
            }
            velocity_setpoint_pub_->publish(velocity_setpoint_);
        }

        double distance(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            velocity_setpoint_pub_->publish(velocity_setpoint_);
        }

        double distance(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            return sqrt(pow(v.x - p.x, 2) + pow(v.y - p.y, 2));
        }

        // needed direction for the transform vector to point towards p
        double needed_angle(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
        double needed_angle(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            return std::atan2(p.y - v.y, p.x - v.x);
        }

        double get_angle_diff(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
        double get_angle_diff(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            double angle_diff = std::fmod((psi_ - needed_angle(v, p) + M_PI), 2*M_PI) - M_PI;
            return angle_diff < -M_PI ? angle_diff + 2*M_PI : angle_diff;        
        }

    public:
        StanleyControllerNode() : Node("stanley_controller_node")
        StanleyControllerNode() : Node("stanley_controller_node")
        {
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /* Params */
            //https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
            this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("K_soft", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("DELTA_SAT", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("parent_frame", rclcpp::PARAMETER_STRING);    // Super important to get parameters from launch files!!

            k_ = this->get_parameter("K").as_double();
            k_soft_ = this->get_parameter("K_soft").as_double();
            DELTA_SAT_ = this->get_parameter("DELTA_SAT").as_double_array();
            init_pose_ = this->get_parameter("init_pose").as_double_array();
            parent_frame_ = this->get_parameter("parent_frame").as_string();
            
            
            /* Publishers */
            car_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/steering/setpoint", 1);
            car_steering_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/steering/can_setpoint", 1);
            current_ref_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sdv/steering/current_reference", 1);
            velocity_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/velocity/desired_setpoint", 1);
            velocity_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/velocity/desired_setpoint", 1);
            smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sdv/steering/smooth_path", 1);

            /* Subscribers */
            imu_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/vectornav/velocity_body",
                1, [this](const nav_msgs::msg::Odometry &msg) { 
                    vel_ = msg.twist.twist.linear.x;
                    vel_msgs_received_ = true;
                });
            imu_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/vectornav/velocity_body",
                1, [this](const nav_msgs::msg::Odometry &msg) { 
                    vel_ = msg.twist.twist.linear.x;
                    vel_msgs_received_ = true;
                });

            // path_to_follow_ = this->create_subscription<nav_msgs::msg::Path>("/path_waypoints",
            //     1, [this](const nav_msgs::msg::Path &msg) { 
            //         last_path_message = this->get_clock()->now();

            //         smooth_path_.poses.clear();
            //         smooth_path_ = msg;
            //         path_arrived_ = true;
            //         if(msg.poses.size() == 0){
            //         waypoint_ = 0;
            //         }
            //         waypoint_base_ = 0;
            //         // waypoint_base_ = 0;
            //     });

            lookahead_wp_sub_ =  this->create_subscription<visualization_msgs::msg::Marker>("/target_waypoint_marker",
                1, [this](const visualization_msgs::msg::Marker &msg) { 
                    last_path_message = this->get_clock()->now();
                    // p2_.position.x = msg.pose.position.x;
                    p2_ = msg.pose;
                    p2_.position.z = msg.pose.position.z+0.1;
                    path_arrived_ = true;

                });


            geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
            pose_stamped_tmp_.header.frame_id = "map";
            current_ref_.header.frame_id = "map";
            current_ref_.header.stamp = StanleyControllerNode::now();
            current_ref_.poses.push_back(pose_stamped_tmp_);
            current_ref_.poses.push_back(pose_stamped_tmp_);

            p2_.position.x = 0.;
            p2_.position.y = 0.;
            p2_.position.z = 0.;

            stanley_ = std::make_unique<StanleyController>(DELTA_SAT_[0], DELTA_SAT_[1], k_, k_soft_);

            timer_ = this->create_wall_timer(100ms, std::bind(&StanleyControllerNode::timer_callback, this));
        }

        ~StanleyControllerNode(){}
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyControllerNode>());
    rclcpp::spin(std::make_shared<StanleyControllerNode>());
    rclcpp::shutdown();
    return 0;
}