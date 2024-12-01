/** ----------------------------------------------------------------------------
 * @file: marine_6dof_dynamic_model.cpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief:  Description of a generic 6dof UUV model in the non-inertial frame with
            Euler Angles.
            Coordinate frame convention:
            - x (front)
            - y (right)
            - z (down) 
   @TODO: Modify matrices for the true general case of non-diagonal matrices.
          Include ALL terms. Also, include offset vector in the case the origin
          is not the COM_.
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/marine_vehicles/base/marine_6dof_dynamic_model.hpp"

Marine6DOFDynamicModel::Marine6DOFDynamicModel(float sample_time)
{
    sample_time_ = sample_time;

    J_ = Eigen::MatrixXf::Zero(6, 6);
    R_ = Eigen::Matrix3f::Zero(3,3);
    T_ = Eigen::Matrix3f::Zero(3,3);

    f_x_ = Eigen::MatrixXf::Zero(6,1);
    g_x_ = Eigen::MatrixXf::Zero(6,6);

    eta_ = Eigen::MatrixXf::Zero(6,1);            // x, y, z, phi, theta, psi
    eta_dot_ = Eigen::MatrixXf::Zero(6,1);
    eta_dot_prev_ = Eigen::MatrixXf::Zero(6,1);
    nu_ = Eigen::MatrixXf::Zero(6,1);             // u, v, w, p, q, r
    nu_dot_ = Eigen::MatrixXf::Zero(6,1);
    nu_dot_prev_ = Eigen::MatrixXf::Zero(6,1);

    M_ = Eigen::MatrixXf::Zero(6, 6);
    M_rb_ = Eigen::MatrixXf::Zero(6, 6);
    M_a_ = Eigen::MatrixXf::Zero(6, 6);
    C_ = Eigen::MatrixXf::Zero(6, 6);
    C_rb_ = Eigen::MatrixXf::Zero(6, 6);
    C_a_ = Eigen::MatrixXf::Zero(6, 6);
    D_ = Eigen::MatrixXf::Zero(6, 6);
    D_lin_ = Eigen::MatrixXf::Zero(6, 6);
    D_qua_ = Eigen::MatrixXf::Zero(6, 6);
    g_eta_ = Eigen::MatrixXf::Zero(6, 1);
    *u_ = Eigen::MatrixXf::Zero(6, 1);

    MAX_FORCE_X_ = 0.0;
    MAX_FORCE_Y_ = 0.0;
    MAX_FORCE_Z_ = 0.0;
    MAX_TORQUE_K_ = 0.0;
    MAX_TORQUE_M_ = 0.0;
    MAX_TORQUE_N_ = 0.0;
}

Marine6DOFDynamicModel::~Marine6DOFDynamicModel(){}

void Marine6DOFDynamicModel::setInitPose(const std::vector<float>& eta){
    eta_(0) = eta[0];
    eta_(1) = eta[1];
    eta_(2) = eta[2];
    eta_(3) = eta[3];
    eta_(4) = eta[4];
    eta_(5) = eta[5];
}

void Marine6DOFDynamicModel::calculateCoriolis()
{
    /* Rigid Body Coriolis Matrix */

    float m_u = m_ * nu_(0);
    float m_v = m_ * nu_(1);
    float m_w = m_ * nu_(2);
    float ixx_p = Ixx_ * nu_(3);
    float iyy_q = Iyy_ * nu_(4);
    float izz_r = Izz_ * nu_(5);
    
    C_rb_ << 0,   0,   0,   0,    m_w,    -m_v,
            0,   0,   0,   -m_w,  0,     m_u,
            0,   0,   0,   m_v,   -m_u,   0,
            0,   m_w,  -m_v, 0,    -izz_r, iyy_q,
            -m_w, 0 ,  m_u,  izz_r, 0,     -ixx_p,
            m_v,  -m_u, 0,   -iyy_q, ixx_p,  0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a1 = X_u_dot_ * nu_(0);
    float a2 = Y_v_dot_ * nu_(1);
    float a3 = Z_w_dot_ * nu_(2);
    float a4 = K_p_dot_ * nu_(3);
    float a5 = M_q_dot_ * nu_(4);
    float a6 = N_r_dot_ * nu_(5);
    
    C_a_ << 0,     0,    0,    0,  -a3,  a2,
           0,     0,    0,   a3,   0,  -a1,
           0,     0,    0,  -a2,  a1,   0,
           0,   -a3,  a2,   0,  -a6,  a5,
           a3,   0,  -a1,  a6,   0,  -a4,
           -a2,  a1,   0,  -a5,   a4,  0;
    
    C_= C_rb_ + C_a_;
    // std::cout << "C:" << C_ << std::endl;

}

void Marine6DOFDynamicModel::calculateDamping()
{    
    /* Hydrodynamic Damping */

    D_lin_ << -(X_u_), 0,      0,    0,   0,    0,
                0,   -(Y_v_), 0,    0,   0,    0,
                0,   0,    -(Z_w_), 0,   0,    0,
                0,   0,      0, K_p_,   0,    0,
                0,   0,      0,    0, M_q_,   0,
                0,   0,      0,    0,   0, -(N_r_);

    D_qua_ << -(X_uu_ * std::fabs(nu_(0))), 0, 0, 0, 0, 0,
             0, -(Y_vv_ * std::fabs(nu_(1))), 0, 0, 0, 0,
             0, 0, -(Z_ww_ * std::fabs(nu_(2))), 0, 0, 0,
             0, 0, 0, -(K_pp_ * std::fabs(nu_(3))), 0, 0,
             0, 0, 0, 0, -(M_qq_ * std::fabs(nu_(4))), 0,
             0, 0, 0, 0, 0, -(N_rr_ * std::fabs(nu_(5)));
    
    D_= D_lin_ + D_qua_;
    // std::cout << "D:" << D_ << std::endl;
}

void Marine6DOFDynamicModel::thrustCallbacK(const vanttec_msgs::ThrustControl& thrust)
{
    *u_ <<  thrust.tau_x,
            thrust.tau_y,
            thrust.tau_z,
            thrust.tau_phi,
            thrust.tau_theta,
            thrust.tau_psi;
}

void Marine6DOFDynamicModel::calculateStates()
{
    utils::calculate6DOFTransformation(R_, T_, J_, eta_);
    nu_dot_prev_ = nu_dot_;
    eta_dot_prev_ = eta_dot_;

    calculateCoriolis();

    calculateDamping();

    /* Rigid Body Mass Matrix */

    M_rb_ << m_, 0, 0, 0, 0, 0,
            0, m_, 0, 0, 0, 0,
            0, 0, m_, 0, 0, 0,
            0, 0, 0, Ixx_, 0, 0,
            0, 0, 0, 0, Iyy_, 0,
            0, 0, 0, 0, 0, Izz_;

    /* Hydrodynamic Added Mass Matrix */

    M_a_ << X_u_dot_, 0, 0, 0, 0, 0,
           0, Y_v_dot_, 0, 0, 0, 0,
           0, 0, Z_w_dot_, 0, 0, 0,
           0, 0, 0, K_p_dot_, 0, 0,
           0, 0, 0, 0, M_q_dot_, 0,
           0, 0, 0, 0, 0, N_r_dot_;

    M_ = M_rb_ + M_a_;

    /* Restoring Forces */
    
    g_eta_ <<  (W_ - B_)*sin(eta_(4)),
              -(W_ - B_)*cos(eta_(4))*sin(eta_(3)),
              -(W_ - B_)*cos(eta_(4))*cos(eta_(3)),
              rb_y_*B_*cos(eta_(4))*cos(eta_(4)) - rb_z_*B_*cos(eta_(4))*sin(eta_(4)),
              -rb_z_*B_*sin(eta_(4)) - rb_x_*B_*cos(eta_(4))*cos(eta_(4)),
              rb_x_*cos(eta_(4))*sin(eta_(4)) + rb_y_*B_*sin(eta_(4));

    /* 6 DoF State Calculation */

    g_x_ = M_.inverse();
    f_x_ = -M_.inverse() * ((C_ * nu_) + (D_ * nu_) + g_eta_);
    // std::cout << "f:" << f_ << std::endl;
    // std::cout << "g:" << g << std::endl;

    nu_dot_ = f_x_ + g_x_*(*u_);
    // nu_dot_ =  M_.inverse() * (tau - (C_ * nu_) - (D_ * nu_) - g_eta_);

    /* Integrating Acceleration to get velocities_ */

    Eigen::VectorXf nu_dot_sum = nu_dot_ + nu_dot_prev_;
    nu_ = (nu_dot_sum / 2 * sample_time_) + nu_;

    /* Integrating velocities_ to get Position */

    eta_dot_ = J_ * nu_;

    Eigen::VectorXf eta_dot_sum_ = eta_dot_ + eta_dot_prev_;
    eta_ = (eta_dot_sum_ / 2 * sample_time_) + eta_;

    if (std::fabs(eta_(3)) > M_PI)
    {
        eta_(3) = (eta_(3) / std::fabs(eta_(3))) * (std::fabs(eta_(3)) - 2 * M_PI);
    }
    if (std::fabs(eta_(4)) > M_PI)
    {
        eta_(4) = (eta_(4) / std::fabs(eta_(4))) * (std::fabs(eta_(4)) - 2 * M_PI);
    }
    if (std::fabs(eta_(5)) > M_PI)
    {
        eta_(5) = (eta_(5) / std::fabs(eta_(5))) * (std::fabs(eta_(5)) - 2 * M_PI);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = eta_(3);  // yaw
    // double eta_(4) = 0.0;         // pitch
    // double eta_(4) = 0.0;           // roll

    // double C_11 = cos(eta_(4))*cos(Theta_(2));
    // double C_12 = cos(eta_(4))*sin(Theta_(2));
    // double C_13 = -sin(eta_(4));
    // double C_21 = sin(eta_(4))*sin(eta_(4))*cos(Theta_(2))-cos(eta_(4))*sin(Theta_(2));
    // double C_22 = sin(eta_(4))*sin(eta_(4))*sin(Theta_(2))+cos(eta_(4))*cos(Theta_(2));
    // double C_23 = sin(eta_(4))*cos(eta_(4));
    // double C_31 = cos(eta_(4))*sin(eta_(4))*cos(Theta_(2))+sin(eta_(4))*sin(Theta_(2));
    // double C_32 = cos(eta_(4))*sin(eta_(4))*sin(Theta_(2))-sin(eta_(4))*cos(Theta_(2));
    // double C_33 = cos(eta_(4))*cos(eta_(4));
    // double trace = C_11+C_22+C_33;
    
    // C_rot <<  C_11, C_12, C_13,
    //                 C_21, C_22, C_23,
    //                 C_31, C_32, C_33;

    // Elements are squared their value
    // quat << 0.25*(1+trace),
    //               0.25*(1+2*C_11-trace),
    //               0.25*(1+2*C_22-trace),
    //               0.25*(1+2*C_33-trace);
    
    // char i;
    // quat.maxCoeff(&i);

    // switch (i){
    // case 0:
    //     quat(0) = sqrt(quat(0));
    //     quat(1) = (C_23-C_32)/4/quat(0);
    //     quat(2) = (C_31-C_13)/4/quat(0);
    //     quat(3) = (C_12-C_21)/4/quat(0);
    //     break;
    // case 1:
    //     quat(1) = sqrt(quat(1));
    //     quat(0) = (C_23-C_32)/4/quat(1);
    //     quat(2) = (C_12+C_21)/4/quat(1);
    //     quat(3) = (C_31+C_13)/4/quat(1);
    //     break;
    // case 2:
    //     quat(2) = sqrt(quat(2));
    //     quat(0) = (C_31-C_13)/4/quat(2);
    //     quat(1) = (C_12+C_21)/4/quat(2);
    //     quat(3) = (C_23+C_32)/4/quat(2);
    //     break;
    // case 3:
    //     quat(3) = sqrt(quat(3));
    //     quat(0) = (C_12-C_21)/4/quat(3);
    //     quat(1) = (C_31+C_13)/4/quat(3);
    //     quat(2) = (C_23+C_32)/4/quat(3);
    //     break;
    // default:
    //     break;
    // }

    // std::cout << "Cuaterniones:" <<std::endl;
    // std::cout << quat[1] << std::endl;
    // std::cout << quat[2] << std::endl;
    // std::cout << quat[3] << std::endl;
    // std::cout << quat[0] << std::endl;
    // std::cout << "Yaw:" <<std::endl;
    // std::cout << eta_(3) << std::endl;

    /* update ROS Messages */

    accelerations_.linear.x = nu_dot_(0);
    accelerations_.linear.y = nu_dot_(1);
    accelerations_.linear.z = nu_dot_(2);
    accelerations_.angular.x = nu_dot_(0);
    accelerations_.angular.y = nu_dot_(1);
    accelerations_.angular.z = nu_dot_(2);

    velocities_.linear.x = nu_(0);
    velocities_.linear.y = nu_(1);
    velocities_.linear.z = nu_(2);
    velocities_.angular.x = nu_(3);
    velocities_.angular.y = nu_(4);
    velocities_.angular.z = nu_(5);
    
    eta_pose_.x = eta_(0);
    eta_pose_.y = eta_(1);
    eta_pose_.z = eta_(2);
    eta_pose_.phi = eta_(3);
    eta_pose_.theta = eta_(4);
    eta_pose_.psi = eta_(5);
}