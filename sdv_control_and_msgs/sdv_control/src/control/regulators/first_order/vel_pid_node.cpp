#include <algorithm>
#include <cmath>
#include <cstdio>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "controllers/control_laws/PID/first_order/pid.hpp"

#include "utils/utils.hpp"

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"

using namespace std::chrono_literals;

class VelPidNode : public rclcpp::Node {
    public:
        VelPidNode() : Node("vel_pid_node") {
            using namespace std::placeholders;
            params = initialize_params();
            controller = PID(params);

            velocity_setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                "/sdv/velocity/setpoint", 10,
                [this](const std_msgs::msg::Float64 &msg) { vel_d = msg.data; });

            velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "/vectornav/velocity_body", 10,
                [this](const geometry_msgs::msg::TwistWithCovarianceStamped &msg) { 
                    vel = msg.twist.twist.linear.x;
                });

            throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                "/sdv/velocity/throttle", 10);

            updateTimer =
                this->create_wall_timer(100ms, std::bind(&VelPidNode::update, this));
        }

    private:
        PIDParameters params;
        PID controller{PID::defaultParams()};

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_setpoint_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;

        rclcpp::TimerBase::SharedPtr updateTimer;

        double vel{0.0}, vel_d{0.0}, u_{0.0};

        std_msgs::msg::Float64 throttle_msg;

        PIDParameters initialize_params() {
            this->declare_parameter("Kp", 1.0);
            this->declare_parameter("Ki", 0.01);
            this->declare_parameter("Kd", 0.2);
            this->declare_parameter("U_MAX", 180.0);
            this->declare_parameter("U_MIN", -180.0);
            this->declare_parameter("enable_ramp", true);
            this->declare_parameter("ramp_rate", 10.0);

            PIDParameters p;
            p.kP = this->get_parameter("Kp").as_double();
            p.kI = this->get_parameter("Ki").as_double();
            p.kD = this->get_parameter("Kd").as_double();
            p.kDt = 0.01;
            p.kUMax = this->get_parameter("U_MAX").as_double();
            p.kUMin = this->get_parameter("U_MIN").as_double();
            p.enable_ramp_rate_limit = this->get_parameter("enable_ramp").as_bool();
            p.ramp_rate = this->get_parameter("ramp_rate").as_double();
            return p;
        }

        void update() {
            u_ = controller.update(vel, vel_d);
            // if(u_ < -0.3 || u_ > 0.2)
            if(u_ < -0. || u_ > 0.)
                throttle_msg.data = u_;
            else
                throttle_msg.data = 0.0;
            throttle_pub_->publish(throttle_msg);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelPidNode>());
    rclcpp::shutdown();
    return 0;
}