#ifndef _CAR_FUSED_CONTROLLERS_
#define _CAR_FUSED_CONTROLLERS_
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"
#include "sdv_msgs/msg/system_dynamics.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "common.hpp"
#include <sdv_control/stanley_controller.hpp>
#include <sdv_control/pid.hpp>

namespace sdc_controller
{
    class CarController : public stanleycontroller::StanleyController, public pid::PID
    {
        public:
            CarController(rclcpp::Node::SharedPtr node, float delta_max, float k, float k_soft, float sample_time, float k_p, float k_i, float k_d, float u_max, const DOFControllerType_E& type);
            virtual ~CarController();
            void timer_callback();

        private:
            rclcpp::Node::SharedPtr node_;
            float delta_max;
            float k;
            float k_soft;
            float sample_time;
            float k_p;
            float k_i;
            float k_d;
            float u_max;
            DOFControllerType_E type;

            rclcpp::Subscription<sdv_msgs::msg::EtaPose>::SharedPtr stanley_heading;
            rclcpp::Subscription<sdv_msgs::msg::SystemDynamics>::SharedPtr car_dynamics;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr car_vel1;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr car_vel2;
    };
}
#endif
