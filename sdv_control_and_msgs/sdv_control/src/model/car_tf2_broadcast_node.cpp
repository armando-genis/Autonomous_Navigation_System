/** ----------------------------------------------------------------------------
 * @file: car_tf2_broadcast_node.cpp
 * @date: August 12, 2023
 * @author: Sebas Mtz
 * @author: Edison Altamirano
 * 
 * @brief: tf2 broadcast node for RViz
 * -----------------------------------------------------------------------------
 **/

#include <chrono>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "simulation/car_tfs/sdc1_broadcaster.hpp"

#include "nav_msgs/msg/path.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"

using namespace std::chrono_literals;

class CarTf2Broadcast : public rclcpp::Node
{
  private:
    int frequency_;
    sdv_msgs::msg::EtaPose car_pose_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<SDC1Broadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr car_path_;

    rclcpp::Subscription<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;

    std::string parent_frame_;
    std::string child_frame_;

    void timer_callback()
    {
        tf_broadcaster_->broadcastTransform(car_pose_);
      car_path_->publish(tf_broadcaster_->path_);
    }

    void set_sim_pose(const sdv_msgs::msg::EtaPose& msg)
    {
        car_pose_.x = msg.x;
        car_pose_.y = msg.y;
        car_pose_.psi = msg.psi;
    }

  public:
    CarTf2Broadcast() : Node("car_t2_broadcast_node")
    {
      this->declare_parameter("frequency", rclcpp::PARAMETER_INTEGER);    // Super important to get parameters from launch files!!

      this->get_parameter_or("frequency", frequency_, 100);

      car_path_ = this->create_publisher<nav_msgs::msg::Path>("/sdc_simulation/car_tf_broadcast/car_path", 10);

      car_eta_pose_ = this->create_subscription<sdv_msgs::msg::EtaPose>("/sdc_simulation/dynamic_model/eta_pose",
                          1, std::bind(&CarTf2Broadcast::set_sim_pose, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency_),
        std::bind(&CarTf2Broadcast::timer_callback, this));   
    }

    ~CarTf2Broadcast(){tf_broadcaster_.reset();}

    void configure()
    {
      tf_broadcaster_ = std::make_unique<SDC1Broadcaster>(shared_from_this());
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto tf2_node = std::make_shared<CarTf2Broadcast>();
  tf2_node->configure();
  rclcpp::spin(tf2_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}