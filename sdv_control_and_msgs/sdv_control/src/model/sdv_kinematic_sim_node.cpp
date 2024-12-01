#include <deque>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sdv_msgs/msg/state.hpp"

using namespace std::chrono_literals;

struct SdvKinematicSimParams {
  double kDt{0.01};
  double kAccel_delay{0.1};
  double kSteering_delay{0.1};

  double kSteerTimeConstant{0.01};
  double kAccelTimeConstant{0.01};

  double kVxLim{5};
  double kAccXLim{1};
  double kSteerRateLim{1};

  double kSteerMin{-M_PI};
  double kSteerMax{M_PI};

  double kWheelbase{1};

  bool kDelayInput{true};
  bool kConstrainAngle{true};
};

using State = Eigen::Matrix<double, 6, 1>;

enum IdxState { X = 0, Y, Yaw, Vx, Steer, Accx };

using U = Eigen::Matrix<double, 2, 1>;

enum IdxU { UAccx = 0, USteer };

class SdvKinematicSim {
public:
  SdvKinematicSim(const SdvKinematicSimParams &params) {
    params_ = params;

    state_(Yaw) = M_PI; // TODO: Set parameter in config file

    initialize_delay_queue();
  }

  void initialize_delay_queue() {
    auto acc_queue_size = static_cast<size_t>(std::round(params_.kAccel_delay / params_.kDt));
    accel_queue_ = std::deque<double>(acc_queue_size, 0);

    auto steering_queue_size = static_cast<size_t>(std::round(params_.kSteering_delay / params_.kDt));
    steering_queue_ = std::deque<double>(steering_queue_size, 0);
  }

  double constrainAngle(double angle) {
    angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }

  void update(const U &u){
    U input = u;
    if(params_.kDelayInput){
      accel_queue_.push_back(input(UAccx));
      steering_queue_.push_back(input(USteer));

      input(UAccx) = accel_queue_.front();
      accel_queue_.pop_front();

      input(USteer) = steering_queue_.front();
      steering_queue_.pop_front();

      if(accel_queue_.empty() || steering_queue_.empty()){
        throw std::runtime_error("Empty input queue.");
      }
    }

    state_ = updateRungeKutta(input);
    state_(Vx) = std::clamp(state_(Vx), -params_.kVxLim, params_.kVxLim);
    if(params_.kConstrainAngle){
        state_(Yaw) = constrainAngle(state_(Yaw));
    }
  }

  const State& getState() const {
    return state_;
  }

private:
  State updateRungeKutta(const U &u) const {
    // Probs unecessary, we might be able to use euler integration instead :)
    State k1 = calc(state_, u);
    State k2 = calc(state_ + k1 * 0.5 * params_.kDt, u);
    State k3 = calc(state_ + k2 * 0.5 * params_.kDt, u);
    State k4 = calc(state_ + k3 * params_.kDt, u);
    return state_ + (1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * params_.kDt);
  }

  State calc(const State &state, const U &u) const {
    const double vel = std::clamp(state(Vx), -params_.kVxLim, params_.kVxLim);
    const double accel = std::clamp(state(Accx), -params_.kAccXLim, params_.kAccXLim);
    const double yaw = state(Yaw);
    const double acc_desired = std::clamp(u(UAccx), -params_.kAccXLim, params_.kAccXLim);
    const double steer_desired = std::clamp(u(USteer), params_.kSteerMin, params_.kSteerMax);
    const double steer = state(Steer);

    // TODO: Add steering deadband.
    const double steer_delta = steer - steer_desired;
    const double steer_rate = std::clamp(-steer_delta / params_.kSteerTimeConstant, -params_.kSteerRateLim, params_.kSteerRateLim);

    State d_state;
    d_state(X) = vel * std::cos(yaw);
    d_state(Y) = vel * std::sin(yaw);
    d_state(Yaw) = vel * std::tan(steer) / params_.kWheelbase;
    d_state(Vx) = accel;
    d_state(Steer) = steer_rate;
    d_state(Accx) = -(accel - acc_desired) / params_.kAccelTimeConstant;

    return d_state;
  }

private:
  State state_{State::Zero()};
  std::deque<double> accel_queue_, steering_queue_;

  SdvKinematicSimParams params_;
};

class SdvKinematicSimNode : public rclcpp::Node {
public:
  SdvKinematicSimNode() : Node("sdv_kinematic_sim"){
    SdvKinematicSimParams params;
    this->declare_parameters(
      "", std::map<std::string, double>({
        {"dt", params.kDt},
        {"accel_delay", params.kAccel_delay},
        {"steering_delay", params.kSteering_delay},
        {"steer_time_constant", params.kSteerTimeConstant},
        {"accel_time_constant", params.kAccelTimeConstant},
        {"vx_lim", params.kVxLim},
        {"accx_lim", params.kAccXLim},
        {"steer_rate_lim", params.kSteerRateLim},
        {"steer_min", params.kSteerMin},
        {"steer_max", params.kSteerMax},
        {"wheelbase", params.kWheelbase}
      })
    );

    params.kDt = this->get_parameter("dt").as_double();
    params.kAccel_delay = this->get_parameter("accel_delay").as_double();
    params.kSteering_delay = this->get_parameter("steering_delay").as_double();
    params.kSteerTimeConstant = this->get_parameter("steer_time_constant").as_double();
    params.kAccelTimeConstant = this->get_parameter("accel_time_constant").as_double();
    params.kVxLim = this->get_parameter("vx_lim").as_double();
    params.kAccXLim = this->get_parameter("accx_lim").as_double();
    params.kSteerRateLim = this->get_parameter("steer_rate_lim").as_double();
    params.kSteerMin = this->get_parameter("steer_min").as_double();
    params.kSteerMax = this->get_parameter("steer_max").as_double();
    params.kWheelbase = this->get_parameter("wheelbase").as_double();

    this->declare_parameter("delay_input", params.kDelayInput);
    params.kDelayInput = this->get_parameter("delay_input").as_bool();

    this->declare_parameter("constrain_angle", params.kConstrainAngle);
    params.kConstrainAngle = this->get_parameter("constrain_angle").as_bool();

    sim_ = std::make_unique<SdvKinematicSim>(params);

    accel_x_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "input/accel_x", 10, [this](const std_msgs::msg::Float64 &msg){
        accel_x_desired = msg.data;
        last_accel_time_ = this->get_clock()->now();
      }
    );

    steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "input/steering", 10, [this](const std_msgs::msg::Float64 &msg){
        steering_desired = msg.data;
      }
    );

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/output/odom", 10);

    state_pub_ = this->create_publisher<sdv_msgs::msg::State>("debug/state", 10);
    dstate_pub_ = this->create_publisher<sdv_msgs::msg::State>("debug/dstate", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_accel_time_ = this->get_clock()->now();

    update_timer_ = this->create_wall_timer(
      10ms, std::bind(&SdvKinematicSimNode::update, this)
    );
  }
protected:
  sdv_msgs::msg::State to_msg(const State &s){
    sdv_msgs::msg::State msg;
    msg.x = s(X);
    msg.y = s(Y);
    msg.yaw = s(Yaw);
    msg.vx = s(Vx);
    msg.steer = s(Steer);
    msg.accx = s(Accx);
    return msg;
  }

  void update() {
    U input;
    if(this->get_clock()->now() - last_accel_time_ < rclcpp::Duration(0, 1000 * 1e6)){
      input(UAccx) = accel_x_desired;
    } else {
      input(UAccx) = 0;
    }
    
    input(USteer) = steering_desired;
    sim_->update(input);

    const auto state = sim_->getState();
    sdv_msgs::msg::State state_msg = to_msg(state);
    state_pub_->publish(state_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = state(X);
    odom_msg.pose.pose.position.y = state(Y);
    odom_msg.pose.pose.position.z = 0;
    odom_msg.twist.twist.linear.x = state(Vx);

    tf2::Quaternion q;
    q.setRPY(0, 0, state(Yaw));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odometry_pub_->publish(odom_msg);

    // Publish tf broadcast.
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = state(X);
    t.transform.translation.y = state(Y);
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
  }
private:
  std::unique_ptr<SdvKinematicSim> sim_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr accel_x_sub_, steering_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sdv_msgs::msg::State>::SharedPtr state_pub_, dstate_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double accel_x_desired{0}, steering_desired{0};
  rclcpp::Time last_accel_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SdvKinematicSimNode>());
  rclcpp::shutdown();
  return 0;
}