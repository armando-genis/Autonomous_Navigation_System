/** ----------------------------------------------------------------------------
 * @file: sdc1_simulation_node.cpp
 * @date: August 12, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * @author: Edison Altamirano
 * 
 * @brief: Self-Driving Car 1 simulation node. Based on Sebas' car dyn model.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "vehicles/vtec_sdc1.hpp"

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"

class CarSimulationNode : public rclcpp::Node
{
  private:
    float sample_time_;
    uint8_t D_MAX_{255};

    std::unique_ptr<VTecSDC1DynamicModel> car_model_;
    rclcpp::TimerBase::SharedPtr timer_;
    diagnostic_msgs::msg::DiagnosticStatus throttle_diag_;

    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr car_accel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr car_vel_;
    rclcpp::Publisher<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;
    // rclcpp::Publisher<sdv_msgs::msg::SystemDynamics>::SharedPtr car_dynamics_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_publisher_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr in_subscriber_;

    std::vector<float> init_pose_ = {0,0,0};

    void timer_callback()
    {
      /* calculate Model States */
      car_model_->calculateStates();

      car_model_->calculateModelParams();

      // car_model_->updateDBSignals();

      /* Publish Odometry */
      car_accel_->publish(car_model_->accelerations_);
      car_vel_->publish(car_model_->velocities_);
      car_eta_pose_->publish(car_model_->eta_pose_);

      /* Publish diagnostics */
      diagnostic_msgs::msg::KeyValue value;
      // TODO. Check for errors in throttle computation (maybe there are no real values) to updated diagnostics
      throttle_diag_.level = 0;
      value.key = "throttle_signal";
      // value.value = car_model_->u_(0);
      value.value = car_model_->D_;
      // RCLCPP_INFO(this->get_logger(), "U: %f",  car_model_->u_(0));
      
      throttle_diag_.values.push_back(value);
      diagnostics_publisher_->publish(throttle_diag_);
    }

    void force_callback(const std_msgs::msg::UInt8& msg) const
    {
      car_model_->setThrottle(msg.data);
    }
    
  public:
    CarSimulationNode() : Node("sdc1_simulation_node")
    {
      int frequency;
      this->declare_parameter("frequency", rclcpp::PARAMETER_INTEGER);    // Super important to get parameters from launch files!!
      this->declare_parameter("D_MAX", rclcpp::PARAMETER_INTEGER);
      this->get_parameter_or("frequency", frequency, 100);
      this->get_parameter_or("D_MAX", D_MAX_, static_cast<uint8_t>(255));

      sample_time_ = 1.0 / static_cast<float>(frequency);

      car_accel_ = this->create_publisher<geometry_msgs::msg::Accel>("/sdc_simulation/dynamic_model/accel", 10);
      car_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/sdc_simulation/dynamic_model/vel", 10);
      car_eta_pose_ = this->create_publisher<sdv_msgs::msg::EtaPose>("/sdc_simulation/dynamic_model/eta_pose", 10);
      diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics",10);

      in_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("/sdc_simulation/dynamic_model/set_throttle",
                      1, std::bind(&CarSimulationNode::force_callback, this, std::placeholders::_1));

      throttle_diag_.name = "Throttle command (D)";
      throttle_diag_.message = "Expected value must be integer in the range of [0, 255]";
      throttle_diag_.hardware_id = "Throttle";

      timer_ = this->create_wall_timer( std::chrono::milliseconds(1000 / frequency),
                                        std::bind(&CarSimulationNode::timer_callback, this));
    }

    ~CarSimulationNode(){car_model_.reset();}

    void configure()
    {
      car_model_ = std::make_unique<VTecSDC1DynamicModel>(sample_time_, D_MAX_);
      car_model_->setInitPose(init_pose_);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto model_node = std::make_shared<CarSimulationNode>();
  model_node->configure();
  rclcpp::spin(model_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}