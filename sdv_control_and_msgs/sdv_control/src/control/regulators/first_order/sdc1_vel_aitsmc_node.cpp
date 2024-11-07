/** ----------------------------------------------------------------------------
 * @file: sdc1_vel_asmc_node.cpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 *
 * @brief: Self-Driving Car 1 velocity aitsmc node. Based on SDC1 car dyn model.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"

#include "controllers/feedback_linearization/model_based_controllers/SDCs/regulators/vtec_sdc1_aitsmc.hpp"
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
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"

class CarControlNode : public rclcpp::Node
{
    private:
        float sample_time_;
        bool is_simulation_;
        bool vel_msgs_received_{false};
        std::string drive_mode_;
        std::string auto_mode_;

        /* AITSMC Params */
        AITSMC_Params params;
        float lambda2_;

        uint8_t D_MAX_;
        float U_MAX_{12800};    // MAX THROTTLE (pasarnos de esto no es bueno
                                // de acuerdo a sims con modelo parametrizado hasta step 95)
        float vel_d_{0.0};
        float vel_body_x_{0.0};

        /* Model Params */
        std::unique_ptr<VTEC_SDC1_1DOF_AITSMC> model_;
        std::vector<double> init_pose_ = {0,0,0};

        rclcpp::TimerBase::SharedPtr timer_;
        // diagnostic_msgs::msg::DiagnosticStatus throttle_diag_;

        /* Publishers */
        rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr car_accel_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr car_vel_;
        rclcpp::Publisher<sdv_msgs::msg::EtaPose>::SharedPtr car_eta_pose_;
        // rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr throttle_diag_pub;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr calc_throttle_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_signal_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr K1_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surface_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_;


        /* Subscribers */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr car_steering_sim_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr car_steering_;
        rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr current_velocity_;
        rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr current_attitude_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_velocity_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
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

                if(drive_mode_ == "Automatic" && auto_mode_ == "Setpoint_Controller"){
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

            // TODO: Check for errors in throttle computation (maybe there are no real values) to update diagnostics
            // throttle_diag_.level = 0;

            std_msgs::msg::UInt8 D;
            std_msgs::msg::Float32 msg;

            D.data = model_->D_;
            calc_throttle_->publish(D);

            msg.data = model_->AITSMCLin::u_;
            control_signal_->publish(msg);

            msg.data = model_->AITSMCLin::control_law_.K1_;
            K1_->publish(msg);

            msg.data = model_->AITSMCLin::control_law_.s_;
            surface_->publish(msg);

            msg.data = model_->AITSMCLin::control_law_.error_;
            error_->publish(msg);

        }

        void set_reference(const std_msgs::msg::Float32& msg) //const
        {
            vel_d_ = msg.data;
            model_->updateCurrentReference(vel_d_, 0);
        }

        void save_velocity(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in) //const
        {
            vel_body_x_ = msg_in->velbody.x;
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

            this->declare_parameter("lambda", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("beta", rclcpp::PARAMETER_DOUBLE);

            this->declare_parameter("K1_init", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("K2", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("K_alpha", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("K_min", rclcpp::PARAMETER_DOUBLE);

            this->declare_parameter("mu", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("D_MAX", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("init_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);

            /* Fill controller configuration */
            frequency = this->get_parameter("frequency").as_int();
            is_simulation_ = this->get_parameter("is_simulation").as_bool();

            params.lambda = this->get_parameter("lambda").as_double();
            params.beta = this->get_parameter("beta").as_double();

            params.K_min = this->get_parameter("K_min").as_double();
            params.K_alpha = this->get_parameter("K_alpha").as_double();
            params.K2 = this->get_parameter("K2").as_double();
            params.K1_init = this->get_parameter("K1_init").as_double();

            params.mu = this->get_parameter("mu").as_double();
            this->get_parameter_or("D_MAX", D_MAX_, static_cast<uint8_t>(180));
            init_pose_ = this->get_parameter("init_pose").as_double_array();

            params.U_MAX = __FLT_MAX__;
            params.controller_type = LINEAR_DOF;

            sample_time_ = 1.0 / static_cast<float>(frequency);

            /* Publishers */
            if(is_simulation_){
                car_accel_ = this->create_publisher<geometry_msgs::msg::Accel>("/sdc_simulation/dynamic_model/accel", 10);
                car_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/sdc_simulation/dynamic_model/vel", 10);
                car_eta_pose_ = this->create_publisher<sdv_msgs::msg::EtaPose>("/sdc_simulation/dynamic_model/eta_pose", 10);
            }
            calc_throttle_ = this->create_publisher<std_msgs::msg::UInt8>("/sdc_control/control_signal/D",10);
            control_signal_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/U",10);
            K1_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/K1",10);
            surface_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/s",10);
            error_ = this->create_publisher<std_msgs::msg::Float32>("/sdc_control/control_signal/e",10);


            // throttle_diag_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics",10);

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
                current_velocity_ = this->create_subscription<vectornav_msgs::msg::InsGroup>("/vectornav/raw/ins",
                                    1, std::bind(&CarControlNode::save_velocity, this, std::placeholders::_1));
            }

            desired_velocity_ = this->create_subscription<std_msgs::msg::Float32>("/sdc_control/setpoint/velocity",
                                1, std::bind(&CarControlNode::set_reference, this, std::placeholders::_1));
            drive_mode_sub_   = this->create_subscription<std_msgs::msg::String>("/sdv/drive_mode",
                                1, std::bind(&CarControlNode::set_drive_mode, this, std::placeholders::_1));
            auto_mode_sub_   = this->create_subscription<std_msgs::msg::String>("/sdv/xbox_controller/auto_mode",
                    1, std::bind(&CarControlNode::set_auto_mode, this, std::placeholders::_1));

            // throttle_diag_.name = "AITSMC Longitudinal Controller for Throttle Commands (D)";
            // throttle_diag_.message = "Integer in the range of [0, 255] for motor controller";
            // throttle_diag_.hardware_id = "Throttle";

            timer_ = this->create_wall_timer( std::chrono::milliseconds(1000 / frequency),
                                                std::bind(&CarControlNode::timer_callback, this));
        }

        ~CarControlNode(){model_.reset();}

        void configure(){
            model_ = std::make_unique<VTEC_SDC1_1DOF_AITSMC>(sample_time_, params, U_MAX_, D_MAX_);
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