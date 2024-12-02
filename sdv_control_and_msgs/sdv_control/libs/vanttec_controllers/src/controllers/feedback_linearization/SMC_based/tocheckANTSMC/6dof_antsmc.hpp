/** ----------------------------------------------------------------------------
 * @file: 6dof_antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive non-singular terminal sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __ANTSMC6DOF_H__
#define __ANTSMC6DOF_H__

#include "antsmc.hpp"
#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class ANTSMC6DOF
{
    private:
        ros::NodeHandle handle;
        // bool functs_arrived;

        float MAX_FORCE_X_;
        float MAX_FORCE_Y_;
        float MAX_FORCE_Z_;
        float MAX_TORQUE_K_;
        float MAX_TORQUE_M_;
        float MAX_TORQUE_N_; 
        
    public:
        Eigen::VectorXf u_;              // Control
        Eigen::VectorXf ua_;             // Auxiliary Control
        Eigen::VectorXf delta_;
        Eigen::VectorXf f_;
        Eigen::MatrixXf g_;
        Eigen::VectorXf q_dot_dot_;      //ref_dot_dot
        vanttec_msgs::ThrustControl  thrust_;

        ANTSMC ANTSMC_x_;
        ANTSMC ANTSMC_y_;
        ANTSMC ANTSMC_z_;
        ANTSMC ANTSMC_phi_;
        ANTSMC ANTSMC_theta_;
        ANTSMC ANTSMC_psi_;
        
        ANTSMC6DOF(float sample_time, const float* alpha, const float* beta_, const float* K2, const float* K_alpha, const float* K_min,  const float* K_min_init, const float* mu);
        ~ANTSMC6DOF();

        void setMaxThrust(float* MAX_TAU);
        void updateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions);
        void updatePose(const vanttec_msgs::EtaPose& current);
        void updateReferencess(const vanttec_msgs::EtaPose& set_points);
        void calculateManipulation();

        // void updateTwist(const geometry_msgs::Twist& twist);
        
        // void updateControlLaw();
        // void updateThrustOutput();

        // void publishAccel();
};

#endif