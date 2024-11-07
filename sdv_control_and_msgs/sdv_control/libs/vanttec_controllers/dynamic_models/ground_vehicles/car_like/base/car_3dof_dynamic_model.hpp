/** ----------------------------------------------------------------------------
 * @file: car_3dof_dynamic_model.hpp
 * @date: November 29, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief:  Description of generic 3-DOF car model in a non-inertial frame with
            Euler Angles.
            The nonlinear bycicle model was used.
            This model works for low speed movement and considers no
            longitudinal slip.
            Coordinate frame convention:
                DYN_MODEL: (equations are in calculated in this frame)
                - x (front)
                - y (left)
                - z (up)
                BASE_LINK: (in accordance with ned)
                - x (front)
                - y (right)
                - z (down)
            ------- Both frames share the same origin -------

                INERTIAL_FRAME: (pose is in this frame. Is different from NED)
                - x (front)
                - y (left)
                - z (up)

 * @TODO:   Determine correctly:
            - Fthrottle
            - Fbrake
            - Rolling resistance
 * -----------------------------------------------------------------------------
 **/

#ifndef __CAR_DYNAMIC_MODEL__
#define __CAR_DYNAMIC_MODEL__

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"

class CarDynamicModel {
    protected:
        float sample_time_;

        // State vectors
        Eigen::Vector3f eta_;           // x, y, psi
        Eigen::Vector3f eta_dot_;       // x_dot, y_dot, psi_dot
        Eigen::Vector3f eta_dot_prev_;  // x_dot, y_dot, psi_dot
        Eigen::Vector3f nu_;            // u, v, r
        Eigen::Vector3f nu_dot_;        // u_dot, v_dot, r_dot
        Eigen::Vector3f nu_dot_prev_;
        float theta_;

        // Rotation matrix
        Eigen::Matrix3f R_;

        /* Vehicle physical parameters */
        float m_;           // Vehicle mass
        float Iz_;          // Moment of inertia on Z axis
        float A_;           // Vehicle frontal projected area
        float Cm1_{0.0};      // Motor constant 1
        float Cm2_{0.0};      // Motor constant 2
        float Cd_;          // Air drag coefficient
        float len_f_;       // Length from the front of the vehicle to the center of mass
        float len_r_;       // Length from the rear of the vehicle to the center of mass
        float car_len_;     // Vehicle length
        float MAX_R_;       // Maximum turning radius
        float fr_;          // Rolling resistance coefficient
        float C_alpha_;     // Tire cornering stiffness
        float alpha_f_;     // Front tire velocity angle
        float alpha_r_;     // Rear tire velocity angle

        /* Model forces */
        float F_grav_;      // Force due to gravity
        float F_brake_;     // Braking force
        float F_throttle_;  // Throttle force
        float F_drag_;      // Air drag force
        float F_rr_;        // Rolling resistance force
        float F_fy_;        // Frontal lateral force
        float F_ry_;        // Rear lateral force
        float rr_offset_{0.0};
        float t_offset_{0.0};

        /* Constructor and destructor */
        CarDynamicModel(float sample_time);
        ~CarDynamicModel();

        /* Class methods */
        void setOffsets(float rr_offset, float t_offset);
        void setMotorConstants(float Cm1, float Cm2);
        // void manualControl(const sdv_msgs::msg::msg::VehicleControl &manual);

    public:
        sdv_msgs::msg::EtaPose      eta_pose_;
        geometry_msgs::msg::Twist   velocities_;
        geometry_msgs::msg::Accel   accelerations_;
        
        /* Non-linear functions */
        Eigen::Vector3f f_;
        Eigen::Matrix3f g_;
        Eigen::Vector3f u_;

        /* Control inputs */
        float B_;           // Braking command
        uint8_t D_;           // Throttle command
        float delta_;       // Steering angle

        void setInitPose(const std::vector<float>& eta);
        void calculateStates();
        void setThrottle(uint8_t D);
        void setSteering(float delta);
        void setPitch(float pitch);
};

#endif