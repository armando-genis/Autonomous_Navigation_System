/** ----------------------------------------------------------------------------
 * @file: los_controller_node.cpp
 * @date: August 17, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Line-Of-Sight Controller node.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "controllers/guidance_laws/LOS.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

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

        std::unique_ptr<LOS> los_;

        /* LOS Params */
        float kappa_{3};
        float KAPPA_MAX_;
        std::vector<double> DELTA_SAT_; // {max, min} steering in rads
        uint8_t precision_{10};

        /* Control signals */
        float vel_;
        std_msgs::msg::Float32 delta_;

        /* Vehicle pose */
        std::vector<double> init_pose_ = {0,0,0};
        Point vehicle_pos_ = {0, 0};
        float psi_{0};

        /* Path */
        Point p1_;
        Point p2_;
        nav_msgs::msg::Path reference_path_;
        size_t waypoint_ = 0;
        size_t path_length_;
        float DISTANCE_VAL_ = 0.5;                // Meters
        std::string parent_frame_;

        rclcpp::TimerBase::SharedPtr timer_;

        /* Publishers */
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr car_steering_;

        /* Subscribers */
        rclcpp::Subscription<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr car_ned_pos_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr car_velocity_;
        rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr car_velocity_imu_;
        rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr current_yaw_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_to_follow_;

        void timer_callback()
        {
            
            if(is_simulation_) {

                traverse_path();

            } else {
                if(vel_msgs_received_ && vehicle_pos_msgs_received_ && vehicle_yaw_msgs_received_){
                    RCLCPP_INFO(this->get_logger(), "Vectornav msgs received");

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
                    p1_.x = reference_path_.poses[waypoint_].pose.position.x;
                    p1_.y = reference_path_.poses[waypoint_].pose.position.y;

                    p2_.x = reference_path_.poses[waypoint_+1].pose.position.x;
                    p2_.y = reference_path_.poses[waypoint_+1].pose.position.y;

                    RCLCPP_INFO(this->get_logger(), "Traversing path segment : (%f, %f) to (%f, %f)",
                                p1_.x, p1_.y, p2_.x, p2_.y);

                    los_->calculateCrosstrackError(vehicle_pos_, p1_, p2_);
                    los_->setYawAngle(psi_);
                    los_->calculateSteering(vel_, 0, precision_);
                    delta_.data = los_->delta_;
                    car_steering_->publish(delta_);

                    if(los_->ex_ < DISTANCE_VAL_){
                        waypoint_++;
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
                ex = reference_path_.poses[i].pose.position.x - vehicle_pos_.x;
                ey = reference_path_.poses[i].pose.position.y - vehicle_pos_.y;
                distance = std::sqrt(ex*ex + ey*ey);

                if (distance < shortest_distance){
                    shortest_distance = distance;
                    waypoint = i;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Nearest waypoint found : (%f, %f)",
                        reference_path_.poses[waypoint].pose.position.x,
                        reference_path_.poses[waypoint].pose.position.y );

            nearest_waypoint_found_ = true;

            return waypoint;
        }

        void set_velocity_imu(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
        {
            // v_norm = std::sqrt(msg_in->velbody.x * msg_in->velbody.x + msg_in->velbody.y * msg_in->velbody.y);
            vel_ = msg_in->velbody.x;
            vel_msgs_received_ = true;
        }

        void set_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            vel_ = msg->linear.x;
        }

        void set_sim_pose(const sdv_msgs::msg::EtaPose& msg)
        {
            // In NED
            vehicle_pos_.x = msg.x;
            vehicle_pos_.y = msg.y;
            psi_ = msg.psi;
        }

        void set_real_pos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            // In NED
            vehicle_pos_.x = msg->pose.pose.position.x;
            vehicle_pos_.y = msg->pose.pose.position.y;
            // psi_ = msg_in->yawpitchroll.x;
            vehicle_pos_msgs_received_ = true;
        }

        void set_yaw(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
        {
            psi_ = msg_in->yawpitchroll.x * M_PI / 180;
            vehicle_yaw_msgs_received_ = true;
        }

        void set_path(const nav_msgs::msg::Path::SharedPtr msg)
        {
            rclcpp::Time path_msg_time(msg->header.stamp);
            rclcpp::Time current_path_time(reference_path_.header.stamp);

            if (std::abs((current_path_time - path_msg_time).nanoseconds()) > 0){
                RCLCPP_INFO(this->get_logger(), "New path received");
                reference_path_ = *msg;
                path_length_ = reference_path_.poses.size();
                new_path_arrived_ = true;
                nearest_waypoint_found_ = false;
            } else {
                new_path_arrived_ = false;
                // RCLCPP_INFO(this->get_logger(), "Same path received");
            }

            path_arrived_ = true;
        }

    public:
        CarGuidanceNode() : Node("car_guidance_node")
        {
            int frequency;

            /* Params */
            //https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
            this->declare_parameter("is_simulation", rclcpp::PARAMETER_BOOL);
            this->declare_parameter("frequency", rclcpp::PARAMETER_INTEGER);    // Super important to get parameters from launch files!!

            this->declare_parameter("kappa", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("KAPPA_MAX", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("DELTA_SAT", rclcpp::PARAMETER_DOUBLE_ARRAY);

            this->declare_parameter("init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("parent_frame", rclcpp::PARAMETER_STRING);

            frequency = this->get_parameter("frequency").as_int();
            is_simulation_ = this->get_parameter("is_simulation").as_bool();

            kappa_ = this->get_parameter("kappa").as_double();
            KAPPA_MAX_ = this->get_parameter("KAPPA_MAX").as_double();
            DELTA_SAT_ = this->get_parameter("DELTA_SAT").as_double_array();

            init_pose_ = this->get_parameter("init_pose").as_double_array();
            parent_frame_ = this->get_parameter("parent_frame").as_string();
            
            sample_time_ = 1.0 / static_cast<float>(frequency);
            
            /* Publishers */
            car_steering_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/delta", 1);

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
                car_ned_pos_  = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/vectornav/pose",
                                1, std::bind(&CarGuidanceNode::set_real_pos, this, std::placeholders::_1));
                car_velocity_imu_ = this->create_subscription<vectornav_msgs::msg::InsGroup>("/vectornav/raw/ins",
                                    1, std::bind(&CarGuidanceNode::set_velocity_imu, this, std::placeholders::_1));
                current_yaw_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>("/vectornav/raw/common",
                                    1, std::bind(&CarGuidanceNode::set_yaw, this, std::placeholders::_1));
            }
            path_to_follow_ = this->create_subscription<nav_msgs::msg::Path>("/sdc_control/reference_path", ///sdc_control/reference_path
                                    1, std::bind(&CarGuidanceNode::set_path, this, std::placeholders::_1));

            timer_ = this->create_wall_timer( std::chrono::milliseconds(1000 / frequency),
                                                std::bind(&CarGuidanceNode::timer_callback, this));
        }

        ~CarGuidanceNode(){}

        void configure(){
            std::vector<float> deltas = {static_cast<float>(DELTA_SAT_[0]),static_cast<float>(DELTA_SAT_[1])};
            los_ = std::make_unique<LOS>(deltas, kappa_, static_cast<float>(KAPPA_MAX_));
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
