/** ----------------------------------------------------------------------------
 * @file: fb_lin_6dof_control.cpp
 * @date: April 25, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */

#include <iostream>
#include <cmath>

#include "controllers/feedback_linearization/base/fb_lin_6dof_control.hpp"

FBLin6DOF::FBLin6DOF(const std::array<float,6>& u_max){
    f_x_ = Eigen::VectorXf::Zero(6, 1);
    g_x_ = Eigen::MatrixXf::Zero(6, 6);

    chiX_dot_d_ = Eigen::VectorXf::Zero(6, 1);
    u_n_ = Eigen::VectorXf::Zero(6, 1);
    u_ = Eigen::VectorXf::Zero(6, 1);
    u_aux_ = Eigen::VectorXf::Zero(6, 1);

    U_MAX_ = u_max;
}

FBLin6DOF::~FBLin6DOF(){}

void FBLin6DOF::updateControlSignal(){

    Eigen::FullPivLU<Eigen::MatrixXf> glu(g_x_);
    if (glu.isInvertible())
    {
        u_ = g_x_.inverse()*(chiX_dot_d_ - f_x_ + u_n_ - u_aux_);
    }

    for(size_t i = 0; i < 6; ++i)
    {
        u_[i] = std::fabs(u_[i]) > U_MAX_[i] ? u_[i] / std::fabs(u_[i]) * U_MAX_[i] : u_[i];
    }
    // std::cout << "u_vec: " << std::endl << u_ << std::endl;
}
