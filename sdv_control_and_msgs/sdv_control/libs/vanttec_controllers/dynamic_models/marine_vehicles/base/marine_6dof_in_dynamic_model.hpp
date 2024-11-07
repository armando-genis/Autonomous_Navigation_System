/** ----------------------------------------------------------------------------
 * @file: marine_6dof_in_dynamic_model.hpp
 * @date: August 31, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a generic 6dof UUV model in the inertial frame with
           Euler Angles.
            Coordinate frame convention:
            - N (front)
            - E (right)
            - D (down) 
   @TODO: Modify matrices for the true general case of non-diagonal matrices.
          Include ALL terms. Also, include offset vector in the case the origin
          is no the COM_.
 * -----------------------------------------------------------------------------
 **/
 
#ifndef __MARINE_IN_6DOF_DYNAMIC_MODEL__
#define __MARINE_IN_6DOF_DYNAMIC_MODEL__

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "utils/utils.hpp"

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/EtaPose.h"

class Marine6DOFInDynamicModel
{
    private:
        float sample_time_;

        /* Transformation matrix */

        Eigen::MatrixXf J_;
        Eigen::Matrix3f R_;
        Eigen::Matrix3f T_;
        Eigen::MatrixXf J_dot_;
        Eigen::Matrix3f R_dot_;
        Eigen::Matrix3f T_dot_;
        Eigen::MatrixXf J_inv_;

        /* System matrices */
        
        Eigen::MatrixXf M_;
        Eigen::MatrixXf M_rb_;
        Eigen::MatrixXf M_a_;
        Eigen::MatrixXf C_;
        Eigen::MatrixXf C_rb_;
        Eigen::MatrixXf C_a_;
        Eigen::MatrixXf D_;
        Eigen::MatrixXf D_lin_;
        Eigen::MatrixXf D_qua_;
        Eigen::VectorXf g_eta_;

        /* Physical Parameters */

        float m_;
        float W_;
        float volume_;
        float B_;
        float Ixx_;
        float Ixy_;
        float Ixz_;
        float Iyx_;
        float Iyy_;
        float Iyz_;
        float Izx_;
        float Izy_;
        float Izz_;

        /* Added Mass Parameters */

        float X_u_dot_;
        float Y_v_dot_;
        float Z_w_dot_;
        float K_p_dot_;
        float M_q_dot_;
        float N_r_dot_;

        /* Damping Parameters */

        float X_u_;
        float Y_v_;
        float Z_w_;
        float K_p_;
        float M_q_;    
        float N_r_;

        float X_uu_;
        float Y_vv_;
        float Z_ww_;
        float K_pp_;
        float M_qq_;
        float N_rr_;

        /* Distance from origin to center of mass */
        // They are the same

        /* Distance from origin to center of buoyancy  */

        float rb_x_;
        float rb_y_;
        float rb_z_;

        float MAX_FORCE_X_;
        float MAX_FORCE_Y_;
        float MAX_FORCE_Z_;
        float MAX_TORQUE_K_;
        float MAX_TORQUE_M_;
        float MAX_TORQUE_N_;

    public:
        /* Input forces vector */
        Eigen::VectorXf u_;
    
        /* Non-linear functions */
        Eigen::VectorXf f_x_;
        Eigen::MatrixXf g_x_;

        /* System states */

        Eigen::VectorXf eta_;            // x, y, z, phi, theta_, psi
        Eigen::VectorXf eta_dot_;
        Eigen::VectorXf eta_dot_prev_;
        Eigen::VectorXf eta_dot_dot_;
        Eigen::VectorXf eta_dot_dot_prev_;
        Eigen::VectorXf nu_;             // u, v, w, p, q, r
        Eigen::VectorXf nu_dot_;
        Eigen::VectorXf nu_dot_prev_;

        vanttec_msgs::EtaPose   eta_pose_;
        geometry_msgs::Twist    velocities_;
        geometry_msgs::Accel    accelerations_;

        Marine6DOFInDynamicModel(float sample_time);
        ~Marine6DOFInDynamicModel();

        void setInitPose(const std::vector<float>& eta);
        void calculateCoriolis();
        void calculateDamping();
        void thrustCallbacK(const vanttec_msgs::ThrustControl& thrust);
        void calculateStates();

        friend class VTecU4InDynamicModel;
};

#endif