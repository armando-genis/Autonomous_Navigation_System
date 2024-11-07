/** ----------------------------------------------------------------------------
 * @file: 6dof_asmc.hpp
 * @date: March 2, 2022
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC6DOF_H__
#define __ASMC6DOF_H__

#include "asmc.hpp"
#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class ASMC6DOF
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
        Eigen::VectorXf f_;
        Eigen::MatrixXf g_;
        Eigen::VectorXf q_dot_dot_;      //ref_dot_dot
        vanttec_msgs::ThrustControl  thrust_;

        ASMC ASMC_x_;
        ASMC ASMC_y_;
        ASMC ASMC_z_;
        ASMC ASMC_phi_;
        ASMC ASMC_theta_;
        ASMC ASMC_psi_;
        
        ASMC6DOF(float sample_time, const float* lambda, const float* K2, const float* K_alpha, const float* K1_init, const float* K_min,  const float* mu);
        ~ASMC6DOF();

        void setMaxThrust(float* MAX_TAU);
        void updateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions);
        void updatePose(const vanttec_msgs::EtaPose& current);
        void updateReferencess(const vanttec_msgs::EtaPose& set_points);
        void calculateManipulation();

        // void updateTwist(const geometry_msgs::Twist& twist);
        
        // void updateControlLaw();
        // void updateThrustOutput();

        // void PublishAccel();
};

#endif