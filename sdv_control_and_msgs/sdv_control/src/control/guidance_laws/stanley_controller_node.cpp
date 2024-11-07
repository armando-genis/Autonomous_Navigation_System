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

#include "controllers/guidance_laws/stanley_controller.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

class CarGuidanceNode : public rclcpp::Node
{
    private:
        float sample_time_;

        bool is_simulation_;
        bool vel_msgs_received_{false};
        bool vehicle_pos_msgs_received_{false};
        bool vehicle_yaw_msgs_received_{false};
        bool new_path_arrived_{false};
        bool path_arrived_{false};
        bool nearest_waypoint_found_{false};

        std::unique_ptr<StanleyController> stanley_;

        /* Stanley Params */
        float k_{2};
        float k_soft_{1.1};
        std::vector<double> DELTA_SAT_; // {max, min} steering in rads
        uint8_t precision_{10};

        /* Control signals */
        double vel_;
        std_msgs::msg::Float32 delta_;
        std_msgs::msg::Float64 steering_setpoint_;
        std_msgs::msg::Float32 slope_;
        const double delta_to_steer = 1 / 0.0658; // relation from delta to steering wheel angle

        /* Vehicle pose */
        std::vector<double> init_pose_ = {0,0,0};
        Point vehicle_pos_ = {0, 0};
        Point pos_z_ = {0,0};
        double psi_{0};

        /* Path */
        Point p1_;
        Point p2_;
        nav_msgs::msg::Path reference_path_;
        size_t waypoint_ = 0;
        size_t waypoint_base_ = 0;
        size_t path_length_;
        // float DISTANCE_VAL_ = 0.5;                // Meters
        float DISTANCE_VAL_ = 3;                // Meters
        std::string parent_frame_;
        nav_msgs::msg::Path smooth_path_;

        const float kLookaheadDistance = 5.0;
        const float kBehindDistance = 5.0;

        nav_msgs::msg::Path current_ref_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


        /* Publishers */
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr car_steering_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr car_steering_setpoint_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr path_slope_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_ref_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;


        /* Subscribers */
        rclcpp::Subscription<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr car_ned_pos_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr car_velocity_;
        // rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr car_velocity_imu_;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_velocity_sub_;
        rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr current_yaw_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_to_follow_;

        void timer_callback()
        {
            
            if(is_simulation_) {

                traverse_path();

            } else {
                if(vel_msgs_received_){
                    // RCLCPP_INFO(this->get_logger(), "Vectornav msgs received");

                    traverse_path();

                } else {
                    RCLCPP_INFO(this->get_logger(), "Waiting for vectornav");
                }
            }
        }

        void traverse_path(){
            if(new_path_arrived_) {
                if(!nearest_waypoint_found_) {
                    //waypoint_ = check_nearest_waypoint();
                }
            }

            if(path_arrived_) {

                if(waypoint_ < path_length_-1){
                    // RCLCPP_INFO(this->get_logger(), "In travel");

                    geometry_msgs::msg::TransformStamped transform;
                    try {
                        transform = tf_buffer_->lookupTransform(
                            "map", "base_link",
                            tf2::TimePointZero);
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Could not transform %s to %s: %s",
                            "base_link", "map", ex.what());
                        return;
                    }                    

                    vehicle_pos_.x = transform.transform.translation.x;
                    vehicle_pos_.y = transform.transform.translation.y;

                    tf2::Quaternion quat;
                    tf2::fromMsg(transform.transform.rotation, quat);
                    double roll, pitch;
                    tf2::Matrix3x3(quat).getRPY(roll, pitch, psi_);
                    psi_ = std::fmod((psi_ + M_PI_2) + M_PI, 2*M_PI) - M_PI;


                    // RCLCPP_ERROR(this->get_logger(), "Pose-> x : %f, y: %f, yaw: %f", vehicle_pos_.x, vehicle_pos_.y, psi_);
                    
                    // p1_.x = reference_path_.poses[waypoint_].pose.position.x;
                    // p1_.y = reference_path_.poses[waypoint_].pose.position.y;
                    if(waypoint_base_ == -1){
                        p1_.x = vehicle_pos_.x;
                        p1_.y = vehicle_pos_.y;
                        pos_z_.x = transform.transform.translation.z - 1.8; // cambiar esto huh
                    } else {
                        p1_.x = smooth_path_.poses[waypoint_base_].pose.position.x;
                        p1_.y = smooth_path_.poses[waypoint_base_].pose.position.y;
                        pos_z_.x = smooth_path_.poses[waypoint_base_].pose.position.z; // cambiar esto huh
                    }

                    // p1_.x = 0.0;
                    // p1_.y = 0.0;

                    p2_.x = smooth_path_.poses[waypoint_].pose.position.x;
                    p2_.y = smooth_path_.poses[waypoint_].pose.position.y;
                    pos_z_.y = smooth_path_.poses[waypoint_].pose.position.z;

                    current_ref_.poses[0].pose.position.x = p1_.x;
                    current_ref_.poses[0].pose.position.y = p1_.y;
                    current_ref_.poses[0].pose.position.z = pos_z_.x;
                    current_ref_.poses[1].pose.position.x = p2_.x;
                    current_ref_.poses[1].pose.position.y = p2_.y;
                    current_ref_.poses[1].pose.position.z = pos_z_.y;

                    slope_.data = (p2_.y - p1_.y)/ (p2_.x - p1_.x);

                    // RCLCPP_INFO(this->get_logger(), "Traversing path segment : (%f, %f) to (%f, %f)",
                    //             p1_.x, p1_.y, p2_.x, p2_.y);

                    stanley_->calculateCrosstrackError(vehicle_pos_, p1_, p2_);
                    stanley_->setYawAngle(psi_);
                    stanley_->calculateSteering(vel_, precision_);
                    delta_.data = stanley_->delta_;
                    steering_setpoint_.data = stanley_->delta_ * delta_to_steer;
                    car_steering_->publish(delta_);
                    path_slope_->publish(slope_);
                    car_steering_setpoint_->publish(steering_setpoint_);
                    current_ref_pub_->publish(current_ref_);
                    smooth_path_pub_->publish(smooth_path_);

                    double angle_diff{0};
                    double dist{0};
                    // Only check for next waypoint if we have more waypoints.

                    RCLCPP_INFO(this->get_logger(), "angle diff: %f", get_angle_diff(transform.transform.translation, 
                        smooth_path_.poses[waypoint_].pose.position));

                    if(waypoint_ < smooth_path_.poses.size() - 1){
                        // Find farthest point from current posision, limited by lookahead.
                        std::size_t max_idx_ = waypoint_;
                        std::size_t min_idx_ = -1;
                        double max_distance_found_ = 0;
                        for(int i = waypoint_; i < smooth_path_.poses.size(); i++){
                            dist = distance(transform.transform.translation, smooth_path_.poses[i].pose.position);
                            
                            // RCLCPP_INFO(this->get_logger(), "dist: %f, max_distance_found: %f", dist, max_distance_found_);
                            angle_diff = get_angle_diff(transform.transform.translation, 
                                smooth_path_.poses[i].pose.position);

                            if(dist < kLookaheadDistance && dist > max_distance_found_ && ((i - max_idx_) < 10)){
                            // if(dist < kLookaheadDistance && dist > max_distance_found_ && std::fabs(angle_diff) < 1){
                                max_idx_ = i;
                                max_distance_found_ = dist;
                            }
                        }
                        waypoint_ = max_idx_;
                        int i = waypoint_ - 1;
                        if(i < 0)
                            i++;
                        while(i > 0 && min_idx_ == -1){
                            dist = distance(transform.transform.translation, 
                                smooth_path_.poses[i].pose.position);

                            angle_diff = get_angle_diff(transform.transform.translation, 
                                smooth_path_.poses[i].pose.position);
                            if(std::fabs(angle_diff) > M_PI_2 && dist > kBehindDistance){
                            // if(std::fabs(angle_diff) > M_PI_2)
                                min_idx_ = i;
                            }
                            i--;
                        }
                        if(i == 0 && min_idx_ == -1)
                            waypoint_base_ = -1;
                        else
                            waypoint_base_ = min_idx_;

                        // if(waypoint_base_ == waypoint_){
                        //     if(waypoint_base_ > 0)
                        //         waypoint_base_--;
                        //     else
                        //         waypoint_++;
                        // }
                            
                        RCLCPP_INFO(this->get_logger(), "waypoint base %d, waypoint end %d", waypoint_, waypoint_base_);
                    }

                } else {
                    RCLCPP_INFO(this->get_logger(), "Reached the end of the path");
                }


            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for reference path");
            }
        }

        size_t check_nearest_waypoint()
        {
            size_t waypoint;
            float shortest_distance = __FLT_MAX__;
            float distance;
            float ex;
            float ey;

            for(size_t i=0; i<path_length_; ++i){
                ex = smooth_path_.poses[i].pose.position.x - vehicle_pos_.x;
                ey = smooth_path_.poses[i].pose.position.y - vehicle_pos_.y;
                distance = std::sqrt(ex*ex + ey*ey);

                if (distance < shortest_distance){
                    shortest_distance = distance;
                    waypoint = i;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Nearest waypoint found : (%f, %f)",
                        smooth_path_.poses[waypoint].pose.position.x,
                        smooth_path_.poses[waypoint].pose.position.y );

            nearest_waypoint_found_ = true;

            return waypoint;
        }

        // void set_velocity_imu(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
        // {
        //     // v_norm = std::sqrt(msg_in->velbody.x * msg_in->velbody.x + msg_in->velbody.y * msg_in->velbody.y);
        //     vel_ = msg_in->velbody.x;
        //     vel_msgs_received_ = true;
        // }

        void set_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            vel_ = msg->linear.x;
        }

        void set_sim_pose(const sdv_msgs::msg::EtaPose& msg)
        {
            // // In NED
            // vehicle_pos_.x = msg.x;
            // vehicle_pos_.y = msg.y;
            // psi_ = msg.psi;
        }

        // void set_real_pos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        // {
        //     // In NED
        //     vehicle_pos_.x = msg->pose.pose.position.x;
        //     vehicle_pos_.y = msg->pose.pose.position.y;
        //     // psi_ = msg_in->yawpitchroll.x;
        //     vehicle_pos_msgs_received_ = true;
        // }

        // void set_yaw(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
        // {
        //     psi_ = msg_in->yawpitchroll.x * M_PI / 180;
        //     vehicle_yaw_msgs_received_ = true;
        // }

        void set_path(const nav_msgs::msg::Path::SharedPtr msg)
        {
            rclcpp::Time path_msg_time(msg->header.stamp);
            rclcpp::Time current_path_time(reference_path_.header.stamp);

            if (std::abs((current_path_time - path_msg_time).nanoseconds()) > 0){
                RCLCPP_ERROR(this->get_logger(), "New path received");
                reference_path_ = *msg;
                new_path_arrived_ = true;
                nearest_waypoint_found_ = false;
                // smooth_path_ = catmull_rom_ify(msg);
                smooth_path_ = *msg;
                path_length_ = smooth_path_.poses.size();
                
            } else
                new_path_arrived_ = false;

            path_arrived_ = true;
        }

        nav_msgs::msg::Path catmull_rom_ify(const nav_msgs::msg::Path::SharedPtr msg){
            nav_msgs::msg::Path out;
            out.header = msg->header;

            geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
            pose_stamped_tmp_.header.frame_id = 'map';

            Eigen::Vector2f p0, p1, p2, p3, a, b, c, d, m1, m2, p;
            float t, t01, t12, t23, alpha{0.5}, tension{0.0};

            for(int i = 1 ; i < msg->poses.size() - 3 ; i++){
                p1 << msg->poses[i].pose.position.x, msg->poses[i].pose.position.y;
                p2 << msg->poses[i+1].pose.position.x, msg->poses[i+1].pose.position.y;
                t12 = pow(distance(p1, p2), alpha);

                if(i == 0) {
                    p3 << msg->poses[i+2].pose.position.x, msg->poses[i+2].pose.position.y;

                    t23 = pow(distance(p2, p3), alpha);

                    m1 = 2 * (1.0f - tension) * (p2 - p1);
                    m2 = (1.0f - tension) * 
                        (p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)));

                } else if(i == msg->poses.size() - 2) {
                    p0 << msg->poses[i-1].pose.position.x, msg->poses[i-1].pose.position.y;

                    t01 = pow(distance(p0, p1), alpha);

                    m1 = (1.0f - tension) *
                        (p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)));
                    m2 << 0, 0;

                } else {
                    p0 << msg->poses[i-1].pose.position.x, msg->poses[i-1].pose.position.y;
                    p3 << msg->poses[i+2].pose.position.x, msg->poses[i+2].pose.position.y;

                    t01 = pow(distance(p0, p1), alpha);
                    t23 = pow(distance(p2, p3), alpha);

                    m1 = (1.0f - tension) *
                        (p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)));
                    m2 = (1.0f - tension) *
                        (p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)));
                }

                a = 2.0f * (p1 - p2) + m1 + m2;
                b = -3.0f * (p1 - p2) - 2 * m1 - m2;
                c = m1;
                d = p1;

                float n = 10.0;
                for(int j = 0 ; j < n ; j++){
                    t = j/n;
                    p = a*pow(t,3) + b*pow(t,2) + c*t + d;
                    pose_stamped_tmp_.pose.position.x = p(0);
                    pose_stamped_tmp_.pose.position.y = p(1);
                    out.poses.push_back(pose_stamped_tmp_);
                    // RCLCPP_ERROR(this->get_logger(), "%d) x : %f, y: %f", i, p(0), p(1));
                }                
            }
            return out;
        }

        float distance(Eigen::Vector2f v1, Eigen::Vector2f v2){
            return (v2 - v1).norm();
        }

        float distance(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            // RCLCPP_ERROR(this->get_logger(), "Distance between x : %f, y: %f and x : %f, y: %f ", v.x, v.y, p.x, p.y);
            return sqrt(pow(v.x - p.x, 2) + pow(v.y - p.y, 2));
        }

        // needed direction for the transform vector to point towards p
        float needed_angle(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            return std::atan2(p.y - v.y, p.x - v.x);
        }

        float get_angle_diff(geometry_msgs::msg::Vector3 v, geometry_msgs::msg::Point p){
            double angle_diff = std::fmod((psi_ - needed_angle(v, p) + M_PI), 2*M_PI) - M_PI;
            return angle_diff < -M_PI ? angle_diff + 2*M_PI : angle_diff;        
        }

    public:
        CarGuidanceNode() : Node("car_guidance_node")
        {
            int frequency;

            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


            /* Params */
            //https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
            this->declare_parameter("is_simulation", rclcpp::PARAMETER_BOOL);
            this->declare_parameter("frequency", rclcpp::PARAMETER_INTEGER);    // Super important to get parameters from launch files!!
            this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("K_soft", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("DELTA_SAT", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("parent_frame", rclcpp::PARAMETER_STRING);    // Super important to get parameters from launch files!!

            frequency = this->get_parameter("frequency").as_int();
            is_simulation_ = this->get_parameter("is_simulation").as_bool();
            k_ = this->get_parameter("K").as_double();
            k_soft_ = this->get_parameter("K_soft").as_double();
            DELTA_SAT_ = this->get_parameter("DELTA_SAT").as_double_array();
            init_pose_ = this->get_parameter("init_pose").as_double_array();
            parent_frame_ = this->get_parameter("parent_frame").as_string();
            
            sample_time_ = 1.0 / static_cast<float>(frequency);
            
            /* Publishers */
            car_steering_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/delta", 1);
            car_steering_setpoint_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/steering/setpoint", 1);
            path_slope_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/path_slope", 1);
            current_ref_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sdv/steering/current_reference", 1);
            smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sdv/steering/smooth_path", 1);

            /* Subscribers */
            
            if(is_simulation_){

                vehicle_pos_.x = init_pose_[0];
                vehicle_pos_.y = init_pose_[1];
                psi_ = init_pose_[2];

                car_eta_pose_ = this->create_subscription<sdv_msgs::msg::EtaPose>("/sdc_simulation/dynamic_model/eta_pose",
                                    1, std::bind(&CarGuidanceNode::set_sim_pose, this, std::placeholders::_1));
                car_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/sdc_simulation/dynamic_model/vel",
                                    1, std::bind(&CarGuidanceNode::set_velocity, this, std::placeholders::_1));
            } else {
                // car_ned_pos_  = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/vectornav/pose",
                //                 1, std::bind(&CarGuidanceNode::set_real_pos, this, std::placeholders::_1));
                // car_velocity_imu_ = this->create_subscription<vectornav_msgs::msg::InsGroup>("/vectornav/raw/ins",
                //                     1, std::bind(&CarGuidanceNode::set_velocity_imu, this, std::placeholders::_1));
                imu_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vectornav/velocity_body",
                    1, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &msg) { 
                        vel_ = msg.twist.twist.linear.x;
                        vel_msgs_received_ = true;
                    });

                // current_yaw_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>("/vectornav/raw/common",
                //                     1, std::bind(&CarGuidanceNode::set_yaw, this, std::placeholders::_1));
            }
            path_to_follow_ = this->create_subscription<nav_msgs::msg::Path>("/sdc_control/reference_path", ///sdc_control/reference_path
                                    1, std::bind(&CarGuidanceNode::set_path, this, std::placeholders::_1));

            geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
            pose_stamped_tmp_.header.frame_id = "map";
            current_ref_.header.frame_id = "map";
            current_ref_.header.stamp = CarGuidanceNode::now();
            current_ref_.poses.push_back(pose_stamped_tmp_);
            current_ref_.poses.push_back(pose_stamped_tmp_);

            timer_ = this->create_wall_timer( std::chrono::milliseconds(1000 / frequency),
                                                std::bind(&CarGuidanceNode::timer_callback, this));
        }

        ~CarGuidanceNode(){}

        void configure(){
            std::vector<float> deltas = {static_cast<float>(DELTA_SAT_[0]),static_cast<float>(DELTA_SAT_[1])};
            stanley_ = std::make_unique<StanleyController>(deltas, k_, k_soft_);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto guidance_node = std::make_shared<CarGuidanceNode>();
    guidance_node->configure();
    rclcpp::spin(guidance_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}