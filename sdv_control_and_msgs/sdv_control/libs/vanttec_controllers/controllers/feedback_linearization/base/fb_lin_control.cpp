/** ----------------------------------------------------------------------------
 * @file: fb_lin_control.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */

#include <iostream>
#include <cmath>

#include "controllers/feedback_linearization/base/fb_lin_control.hpp"

FBLin::FBLin(float u_max) : U_MAX_(u_max) {}

FBLin::~FBLin(){}

void FBLin::updateControlSignal(){

    if(std::isnormal(g_x_))
    {
        u_ = (chiX_dot_d_ - f_x_ + u_n_ - u_aux_)/g_x_;
    }

    u_ = std::fabs(u_) > U_MAX_ ? u_ / std::fabs(u_) * U_MAX_ : u_;

}