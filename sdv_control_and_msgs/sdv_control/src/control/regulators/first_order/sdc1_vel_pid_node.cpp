/** ----------------------------------------------------------------------------
 * @file: sdc1_vel_pid_node.cpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Self-Driving Car 1 velocity pid node. Based on SDC1 car dyn model.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "controllers/feedback_linearization/model_based_controllers/SDCs/regulators/vtec_sdc1_pid.hpp"
#include "utils/utils.hpp"

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "sdv_msgs/msg/eta_pose.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

// #include "std_msgs/msg/float32.hpp"

class CarControlNode : public rclcpp::Node
{
    private:
        float sample_time_;
        bool is_simulation_;
        bool vel_msgs_received_{false};
        std::string drive_mode_;
        std::string auto_mode_;
        
        /* PID Params */
        float kp_;
        float ki_;
        float kd_;
        uint8_t D_MAX_;
        float U_MAX_{12800};     // MAX THROTTLE (pasarnos de esto no es bueno
                                  // de acuerdo a sims con modelo parametrizado hasta step 95)
        float vel_d_{0.0};
        float vel_body_x_{0.0};
        DOFControllerType_E controller_type_{LINEAR_DOF};

        /* Model Params */
        std::unique_ptr<VTEC_SDC1_1DOF_PID> model_;
        std::vector<double> init_pose_ = {0,0,0};

        rclcpp::TimerBase::SharedPtr timer_;
        diagnostic_msgs::msg::DiagnosticStatus throttle_diag_;

        /* Publishers */
        rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr car_accel_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr car_vel_;
        rclcpp::Publisher<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr throttle_diag_pub;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr calc_throttle_;

        /* Subscribers */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr car_steering_sim_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr car_steering_;
        rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr current_velocity_;
        rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr current_attitude_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_velocity_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_velocity_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr auto_mode_sub_;

        // rclcpp::Publisher<sdv_msgs::msg::ThrustControl>::SharedPtr car_force_;

        void timer_callback()
        {

                if(is_simulation_){

                    /* calculate Model States */
                    model_->calculateModelParams();

                    model_->calculateStates();

                    model_->updateNonLinearFunctions();

                    model_->calculateControlSignals();
                    model_->updateControlSignals();
                    model_->updateDBSignals(vel_d_);

                    /* Publish Odometry */
                    car_accel_->publish(model_->accelerations_);
                    car_vel_->publish(model_->velocities_);
                    car_eta_pose_->publish(model_->eta_pose_);

                } else {

                    if(drive_mode_ == "Automatic"){
                        RCLCPP_INFO(this->get_logger(), "Autonomous mode enabled");

                        if(vel_msgs_received_){

                            /* calculate Model States */
                            model_->calculateModelParams();

                            model_->calculateStates();

                            model_->updateNonLinearFunctions();

                            RCLCPP_INFO(this->get_logger(), "Vectornav vel received");
                            model_->calculateControlSignals(vel_body_x_);
                            model_->updateControlSignals();
                            model_->updateDBSignals(vel_d_);
                        } else
                            RCLCPP_INFO(this->get_logger(), "Waiting for vectornav");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Warning: Manual mode enabled");
                    }
                }
            

            /* Publish diagnostics */
            // TODO. Check for errors in throttle computation (maybe there are no real values) to updated diagnostics
            // value.value = model_->u_(0);
            // RCLCPP_INFO(this->get_logger(), "U: %f",  model_->u_(0));
            diagnostic_msgs::msg::KeyValue value;
            throttle_diag_.level = 0;
            value.key = "throttle_signal";
            value.value = std::to_string(model_->D_);
            throttle_diag_.values.push_back(value);
            throttle_diag_pub->publish(throttle_diag_);

            std_msgs::msg::UInt8 D;
            D.data = model_->D_;
            calc_throttle_->publish(D);

            // model_->calculateCrosstrackError(x0,y0,x1,y1);
            // std_msgs::msg::Float32 deltainfo;
            // deltainfo.data = model_->delta_;
            // // car_steering_->publish(deltainfo);

            // model_->updateSetpoint(4,0);
            // sdv_msgs::msg::ThrustControl u;
            // u.tau_x = model_->u_;
            // car_force_->publish(u);

        }
        
        void set_reference(const std_msgs::msg::Float32& msg) //const
        {
            vel_d_ = msg.data;
            model_->updateCurrentReference(vel_d_, 0);
        }

        void save_velocity(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in) //const
        {
            vel_body_x_ = msg_in->velbody.x;
            // model_->updateCurrentReference(vel_body_x_, 0);
            vel_msgs_received_ = true;
        }

        void set_pitch(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in) //const
        {
            if(this->is_simulation_)
                model_->setPitch(msg_in->yawpitchroll.y * M_PI / 180.0);
        }

        void set_steering(const std_msgs::msg::Float32& msg) //const
        {
            if(this->is_simulation_){
                model_->setSteering(msg.data);
            } else {
                // std::cout << std::to_string(msg.data * M_PI / 180.0) <<std::endl;
                model_->setSteering(/*msg.data * M_PI / 180.0*/0.0);
            }
        }

        void set_drive_mode(const std_msgs::msg::String& msg)
        {
            drive_mode_ = msg.data;
        }

        void set_auto_mode(const std_msgs::msg::String& msg)
        {
            auto_mode_ = msg.data;
        }
    public:
        CarControlNode() : Node("sdc_control_node")
        {
            int frequency;

            /* Params */
            //https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
            this->declare_parameter("is_simulation", rclcpp::PARAMETER_BOOL);
            this->declare_parameter("frequency", rclcpp::PARAMETER_INTEGER);    // Super important to get parameters from launch files!!
            this->declare_parameter("Kp", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("Ki", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("Kd", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("D_MAX", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);

            frequency = this->get_parameter("frequency").as_int();
            is_simulation_ = this->get_parameter("is_simulation").as_bool();
            kp_ = this->get_parameter("Kp").as_double();
            ki_ = this->get_parameter("Ki").as_double();
            kd_ = this->get_parameter("Kd").as_double();
            this->get_parameter_or("D_MAX", D_MAX_, static_cast<uint8_t>(180));
            init_pose_ = this->get_parameter("init_pose").as_double_array();

            // std::cout << "Freq = " << frequency << std::endl;
            // std::cout << "kp = " << kp_ << std::endl;
            // std::cout << "ki = " << ki_ << std::endl;
            // std::cout << "kd = " << kd_ << std::endl;
            // std::cout << "D_MAX = " << static_cast<int>(D_MAX_) << std::endl;

            sample_time_ = 1.0 / static_cast<float>(frequency);
            
            /* Publishers */
            if(is_simulation_){
                car_accel_ = this->create_publisher<geometry_msgs::msg::Accel>("/sdc_simulation/dynamic_model/accel", 10);
                car_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/sdc_simulation/dynamic_model/vel", 10);
                car_eta_pose_ = this->create_publisher<sdv_msgs::msg::EtaPose>("/sdc_simulation/dynamic_model/eta_pose", 10);
            }
            calc_throttle_ = this->create_publisher<std_msgs::msg::UInt8>("/sdc_control/control_signal/D",10);
            throttle_diag_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics",10);
            // car_force_ = this->create_publisher<sdv_msgs::msg::ThrustControl>("/sdc_control/sdc_control_node/force",1);

            /* Subscribers */
            if(is_simulation_){
                car_steering_sim_     = this->create_subscription<std_msgs::msg::Float32>("/sdc_control/control_signal/delta",
                                    1, std::bind(&CarControlNode::set_steering, this, std::placeholders::_1));
            } else {
                car_steering_     = this->create_subscription<std_msgs::msg::Float32>("/sdc_state/steering",
                                    1, std::bind(&CarControlNode::set_steering, this, std::placeholders::_1));
                current_attitude_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>("/vectornav/raw/common",
                                    1, std::bind(&CarControlNode::set_pitch, this, std::placeholders::_1));
                // current_velocity_ = this->create_subscription<vectornav_msgs::msg::InsGroup>("/vectornav/raw/ins",
                //                     1, std::bind(&CarControlNode::save_velocity, this, std::placeholders::_1));

                imu_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vectornav/velocity_body",
                    1, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &msg) { 
                        this->vel_body_x_ = msg.twist.twist.linear.x;
                        this->vel_msgs_received_ = true;
                    });
            }

            desired_velocity_ = this->create_subscription<std_msgs::msg::Float32>("/sdc_control/setpoint/velocity",
                                1, std::bind(&CarControlNode::set_reference, this, std::placeholders::_1));
            drive_mode_sub_   = this->create_subscription<std_msgs::msg::String>("/sdv/drive_mode",
                                1, std::bind(&CarControlNode::set_drive_mode, this, std::placeholders::_1));
            auto_mode_sub_   = this->create_subscription<std_msgs::msg::String>("/sdv/xbox_controller/auto_mode",
                    1, std::bind(&CarControlNode::set_auto_mode, this, std::placeholders::_1));

            throttle_diag_.name = "Throttle command (D)";
            throttle_diag_.message = "Integer in the range of [0, 255] for motor controller";
            throttle_diag_.hardware_id = "Throttle";

            timer_ = this->create_wall_timer( std::chrono::milliseconds(1000 / frequency),
                                                std::bind(&CarControlNode::timer_callback, this));
        }

        ~CarControlNode(){model_.reset();}

        void configure(){
            model_ = std::make_unique<VTEC_SDC1_1DOF_PID>(sample_time_, kp_, ki_, kd_,
                                                            U_MAX_, controller_type_, D_MAX_);
            
            std::vector<float> init_pose = {static_cast<float>(init_pose_[0]),
                                            static_cast<float>(init_pose_[1]),
                                            static_cast<float>(init_pose_[2])};
            model_->setInitPose(init_pose);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<CarControlNode>();
    control_node->configure();
    rclcpp::spin(control_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
