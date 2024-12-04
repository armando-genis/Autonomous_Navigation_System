#include "can_node_base.h"
#include "Vanttec_CANLib/CANMessage.h"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sdv_msgs/srv/uint8.hpp"
#include "std_srvs/srv/empty.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

#define STEER_MOTOR_ID 0x00
#define BRAKE_MOTOR_ID 0x01

#define STEER_MOTOR_ID 0x00
#define BRAKE_MOTOR_ID 0x01

class CanNodeSDV : public CanNodeBase {
public:
    CanNodeSDV() : CanNodeBase("sdv_can_node"){
        using namespace std::placeholders;

        // Initialize parameters
        this->declare_parameter("steer_enable", true);
        this->declare_parameter("throttle_enable", true);
        this->declare_parameter("brake_enable", true);

        steer_enable = this->get_parameter("steer_enable").as_bool();
        throttle_enable = this->get_parameter("throttle_enable").as_bool();
        brake_enable = this->get_parameter("brake_enable").as_bool();

        // [steer_motor_angle_sub] : convert Float64 to CANMessage and send it
        steer_motor_angle_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/sdv/steering/setpoint", 10, [this](const std_msgs::msg::Float64::SharedPtr msg){
                if(steer_enable){
                    uint8_t base_msg_id = (STEER_MOTOR_ID & 0b11) << 6;
                    vanttec::CANMessage can_msg;
                    vanttec::packFloat(can_msg, base_msg_id | 0x01, msg->data);
                    send_frame(0x410, can_msg);
                }
            }
        );

        // [throttle_setpoint_sub] : convert Float64 to CANMessage and send it
        throttle_setpoint_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/sdv/velocity/throttle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg){
                last_throttle_message = this->get_clock()->now();
                vanttec::CANMessage throttle_msg, braking_msg;
                double output = std::clamp(msg->data, -1., 1.);
                double throttle_{0.}, brake_{0.};

                // Throttle
                if(throttle_enable){
                    if(output >= this->max_throttle_threshold){
                        throttle_ = output;
                    } else if(output < this->max_throttle_threshold) {
                        throttle_ = 0.;
                    }
                    RCLCPP_ERROR(this->get_logger(), "%f", throttle_);
                    vanttec::packByte(throttle_msg, 0x05, (uint8_t)(throttle_*180)); // TODO: Change positive throttle canmsg to float too
                    send_frame(0x406, throttle_msg);
                }

                // Brake
                if(brake_enable){
                    if(output <= this->min_throttle_threshold) {
                        brake_ = std::clamp(-output, 0., 1.);
                    } else if(output > this->min_throttle_threshold) {
                        brake_ = 0.;
                    }
                    std::cout << brake_ << std::endl;
                    uint8_t base_msg_id = (BRAKE_MOTOR_ID & 0b11) << 6;
                    vanttec::packFloat(braking_msg, base_msg_id | 0x01, brake_);
                    send_frame(0x410, braking_msg);
                }
            }
        );

        // [steering_angle_pub] : convert encoder's angle to Float64 and publish it
        steering_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/sdv/steering/position", 10
        );

        is_auto_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/sdv/is_auto", 10
        );

        // [zero_steering_service] : if called, send a CAN message to zero the encoder on current position
        zero_steering_service = this->create_service<std_srvs::srv::Empty>(
            "/sdv/steering/reset_encoder",
            std::bind(
                &CanNodeSDV::zero_steering, this, _1, _2
            )
        );

        // [zero_braking_service] : if called, send a CAN message to zero the encoder on current position
        zero_braking_service = this->create_service<std_srvs::srv::Empty>(
            "/sdv/braking/reset_encoder",
            std::bind(
                &CanNodeSDV::zero_braking, this, _1, _2
            )
        );

        // [mode_service] : will send the value of the service as the current mode to CAN 
        mode_service = this->create_service<sdv_msgs::srv::Uint8>(
            "/sdv/steering/set_mode",
            std::bind(
                &CanNodeSDV::set_mode, this, _1, _2
            )
        );

        throttle_watchdog_timer_ = this->create_wall_timer(200ms, std::bind(&CanNodeSDV::throttle_watchdog, this));
    }
protected:
    void parse_frame(const struct can_frame &frame) override {
        vanttec::CANMessage msg;
        std::copy(std::begin(frame.data), std::end(frame.data), std::begin(msg.data));
        msg.len = frame.can_dlc;
        uint8_t vttec_msg_id = vanttec::getId(msg);
        uint32_t can_id = frame.can_id;
        
        auto steady_clock = rclcpp::Clock();

    if (frame.can_id == 0x1A0) {
        // 24-bit raw encoder value (3 bytes)
        int32_t raw_encoder_value = (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];

        // Adjust the raw value by subtracting the offset 0x800000
        raw_encoder_value -= 0x800000;

        // Scale it based on 4096 and convert to radians (-π to +π)
        // Using the original scaling factor: ((float)raw_encoder_value / 4096.0f) * 2.0 * M_PI * -1
        float encoder_position = (static_cast<float>(raw_encoder_value) / 4096.0f) * 2.0 * M_PI /16.0;
        // float encoder_position = (static_cast<float>(raw_encoder_value) / 4096.0f) * 2.0 * M_PI;
    
        // Publish the encoder angle in radians
        std_msgs::msg::Float64 encoder_msg;
        encoder_msg.data = encoder_position;
        steering_angle_pub->publish(encoder_msg);

        // RCLCPP_INFO(this->get_logger(), "Steering encoder angle (radians): %f", encoder_position);
        // RCLCPP_INFO(this->get_logger(), "Raw encoder value (hex): %#X", raw_encoder_value);
    }

    }

    void throttle_watchdog(){
        /*
cansend can0 410#0200
cansend can0 406#0701
cansend can0 406#0601
        */
        vanttec::CANMessage pot_enable_msg, steer_enable_msg, enable_motor; 

        // If throttle message has been received within 100ms
        //if(is_auto && this->get_clock()->now() - last_throttle_message < rclcpp::Duration(0, 100 * 1e6)){
        if ( is_auto ) {
            RCLCPP_INFO(this->get_logger(), "auto enabled.");
	   	
            vanttec::packByte(pot_enable_msg, 0x07, 0x01);
            vanttec::packByte(enable_motor, 0x06, 0x01);
            vanttec::packByte(steer_enable_msg, 0x02, 0x00);

        } else {
            // Disable pot control, enable manual control.
            vanttec::packByte(pot_enable_msg, 0x07, 0x00);
            vanttec::packByte(enable_motor, 0x06, 0x00);
            vanttec::packByte(steer_enable_msg, 0x02, 0x01);
        }

        send_frame(0x406, enable_motor);
        send_frame(0x406, pot_enable_msg);
	    send_frame(0x410, steer_enable_msg);

        is_auto_msg.data = bool(is_auto);
        is_auto_pub->publish(is_auto_msg);
    }
 
    void zero_braking(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "setting braking encoder to zero");

        /*Reset braking encoder to the middle
cansend can0 013#04130C01
        */
        
        vanttec::CANMessage msg1{0x04,0x13,0x0C,0x01};
        vanttec::CANMessage msg2{0x04,0x13,0x04,0xAA};
        send_frame(0x13, msg1);
        send_frame(0x13, msg2);
    }        

    void zero_steering(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "setting steering encoder to zero");

        /* Reset steering encoder
        cansend can0 000#8000 
        cansend can0 620#2303600000008000
        cansend can0 620#2310100173617665
        cansend can0 000#0100
        */
        vanttec::CANMessage stop_encoder_msg, reboot_encoder_msg;
        vanttec::packByte(stop_encoder_msg,0x80,0x00);
        vanttec::packByte(reboot_encoder_msg,0x01,0x00);

        vanttec::CANMessage set_zero_msg{0x23,0x03,0x60,0x00,0x00,0x00,0x80,0x00};
        set_zero_msg.len=8;
        vanttec::CANMessage store_params_msg{0x23,0x10,0x10,0x01,0x73,0x61,0x76,0x65};
        store_params_msg.len=8;

        send_frame(0x000, stop_encoder_msg);
        send_frame(0x620, set_zero_msg);
        send_frame(0x620, store_params_msg);
        send_frame(0x000, reboot_encoder_msg);
    }

    void set_mode(const std::shared_ptr<sdv_msgs::srv::Uint8::Request> request,
        std::shared_ptr<sdv_msgs::srv::Uint8::Response> response) {

        is_auto = request.get()->data == 1;
    }

private:
    bool is_auto{false};
    bool steer_enable{true}, throttle_enable{true}, brake_enable{true};
    double min_throttle_threshold{-0.05};
    double max_throttle_threshold{0.05};
    rclcpp::TimerBase::SharedPtr throttle_watchdog_timer_;
    rclcpp::Time last_throttle_message;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_steering_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_braking_service;
    rclcpp::Service<sdv_msgs::srv::Uint8>::SharedPtr mode_service;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_motor_angle_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_setpoint_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub;

    std_msgs::msg::Bool is_auto_msg;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNodeSDV>());
  rclcpp::shutdown();
  return 0;
}
