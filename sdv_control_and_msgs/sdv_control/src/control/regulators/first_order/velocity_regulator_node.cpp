#include <cstdio>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sdv_msgs/msg/perception_mode.hpp"

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "utils/utils.hpp"

using namespace std::chrono_literals;

class VelRegulatorNode : public rclcpp::Node {
    public:
        VelRegulatorNode() : Node("velocity_regulator_node") {
            using namespace std::placeholders;

            stanley_setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                "/sdv/velocity/desired_setpoint", 10,
                [this](const std_msgs::msg::Float64 &msg) { vel_d = msg.data; });

            perception_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/sdv/emergency_stop", 10,
                [this](const std_msgs::msg::Bool &msg) { mode_value = (double)(msg.data); });

            velocity_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                "/sdv/velocity/setpoint", 10);

            updateTimer =
                this->create_wall_timer(10ms, std::bind(&VelRegulatorNode::update, this));
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stanley_setpoint_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr perception_mode_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_setpoint_pub_;

        rclcpp::TimerBase::SharedPtr updateTimer;

        double vel_d{0.0};

        double mode_value{1.};

        // double perception_mode_value[3]{0., 0.5, 1.};

        std_msgs::msg::Float64 setpoint_msg;

        void update() {
            setpoint_msg.data = vel_d * mode_value;
            velocity_setpoint_pub_->publish(setpoint_msg);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelRegulatorNode>());
    rclcpp::shutdown();
    return 0;
}