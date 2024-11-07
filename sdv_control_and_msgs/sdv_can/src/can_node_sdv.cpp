#include "can_node_base.h"
#include "Vanttec_CANLib/CANMessage.h"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "sdv_msgs/srv/uint8.hpp"
#include "std_srvs/srv/empty.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class CanNodeSDV : public CanNodeBase {
public:
    CanNodeSDV() : CanNodeBase("sdv_can_node"){
        using namespace std::placeholders;

        // [joystick] -> [this node] -> [stepper pcb] -> [stepper motor]
        // [this node][motor_angle_sub] : convert FLoat64 to CANMessage and send it
        motor_angle_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/sdv/steering/setpoint", 10, [this](const std_msgs::msg::Float64::SharedPtr msg){
                // RCLCPP_INFO(this->get_logger(), "Setpoint: %f", msg->data);
                vanttec::CANMessage can_msg;
                vanttec::packFloat(can_msg, 0x01, msg->data);
                send_frame(0x410, can_msg);
            }
        );

        throttle_setpoint_sub = this->create_subscription<std_msgs::msg::UInt8>(
            "/sdv/throttle/setpoint", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg){
                last_throttle_message = this->get_clock()->now();
                vanttec::CANMessage throttle_setpoint_msg;
                uint8_t output = msg->data;
                if(output > 180)
                    output = 180;

                vanttec::packByte(throttle_setpoint_msg, 0x05, output);
                send_frame(0x406, throttle_setpoint_msg);
            }
        );

        // [britter encoder] -> [CAN] -> [this node] -> [ros]
        // [this node][steering_angle_pub] : convert encoder's angle to Float64 and publish it
        steering_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/sdv/steering/position", 10
        );

        // [ros] -> [this node] -> [stepper pcb] -> [stepper motor]
        // [this node][zero_service] : if called, send a CAN message to zero the encoder on current position
        zero_service = this->create_service<std_srvs::srv::Empty>(
            "/sdv/steering/reset_encoder",
            std::bind(
                &CanNodeSDV::zero_encoder, this, _1, _2
            )
        );

        // [ros] -> [this node] -> [CAN network]
        // [this node][mode_service] : will send the value of the service as the current mode to CAN 
        mode_service = this->create_service<sdv_msgs::srv::Uint8>(
            "/sdv/steering/set_mode",
            std::bind(
                &CanNodeSDV::set_mode, this, _1, _2
            )
        );

        throttle_watchdog_timer_ = this->create_wall_timer(100ms, std::bind(&CanNodeSDV::throttle_watchdog, this));
    }
protected:
    void parse_frame(const struct can_frame &frame) override {
        vanttec::CANMessage msg;
        std::copy(std::begin(frame.data), std::end(frame.data), std::begin(msg.data));
        msg.len = frame.can_dlc;
        uint8_t vttec_msg_id = vanttec::getId(msg);
        uint32_t can_id = frame.can_id;
        
        auto steady_clock = rclcpp::Clock();

        // RCLCPP_INFO(this->get_logger(), "Got message from: %#X  with vttec id: %#X", can_id, vttec_msg_id);

        if(can_id == 0x407){
            if(vttec_msg_id == 0x03){
                std_msgs::msg::Float64 encoder_msg;
                encoder_msg.data  = vanttec::getFloat(msg);
                auto steady_clock = rclcpp::Clock();
                // RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock, 1000, "Got encoder message: %f", encoder_msg.data);
                steering_angle_pub->publish(encoder_msg);
            }
        }
    }

    void throttle_watchdog(){
        vanttec::CANMessage pot_enable_msg, stepper_enable_msg; 

    //     // If throttle message has been received within 100ms
    //     //if(is_auto && this->get_clock()->now() - last_throttle_message < rclcpp::Duration(0, 100 * 1e6)){
    //     if ( is_auto ) {
    //         RCLCPP_INFO(this->get_logger(), "auto enabled.");
    //         vanttec::packByte(pot_enable_msg, 0x07, 0x01);

    //         vanttec::CANMessage enable_motor{0x06, 0x01};
    //         send_frame(0x406, enable_motor);
	   	
	//     vanttec::packByte(stepper_enable_msg, 0x02, 0x00);

    //     } else {
    //         // Disable pot control, enable manual control.
    //         vanttec::packByte(pot_enable_msg, 0x07, 0x00);

	//     vanttec::packByte(stepper_enable_msg, 0x02, 0x01);
    //     }

    //     send_frame(0x406, pot_enable_msg);
	// send_frame(0x410, stepper_enable_msg);
    }
 
    void zero_encoder(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "setting encoder to zero");

        // For debugging
        // cansend can0 620#2303600000000080
        // cansend can0 620#2310100173617665
        
        vanttec::CANMessage set_zero_msg{0x23,0x03,0x60,0x00,0x00,0x00,0x00,0x80};
        vanttec::CANMessage store_params_msg{0x23,0x10,0x10,0x01,0x73,0x61,0x76,0x65};

        send_frame(0x620, set_zero_msg);
        send_frame(0x620, store_params_msg);
    }

    void set_mode(const std::shared_ptr<sdv_msgs::srv::Uint8::Request> request,
        std::shared_ptr<sdv_msgs::srv::Uint8::Response> response) {

        is_auto = request.get()->data == 1;
        
        // Send auto mode to steering stepper board.
        uint8_t data = request.get()->data;
        vanttec::CANMessage set_mode_msg{0x2, is_auto};
        send_frame(0x410, set_mode_msg);
    }

private:
    bool is_auto{false};
    rclcpp::TimerBase::SharedPtr throttle_watchdog_timer_;
    rclcpp::Time last_throttle_message;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_service;
    rclcpp::Service<sdv_msgs::srv::Uint8>::SharedPtr mode_service;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_angle_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr throttle_setpoint_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_angle_pub;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNodeSDV>());
  rclcpp::shutdown();
  return 0;
}
