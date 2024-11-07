/** ----------------------------------------------------------------------------
 * @file: fblin_pid.cpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/PID_based/first_order/fblin_pid.hpp"
#include <cfloat>

float U_AUX_MAX = FLT_MAX;

PIDLin::PIDLin( float sample_time, float k_p, float k_i, float k_d,
                float u_max, const DOFControllerType_E& type) : 
                FBLin (u_max),
                control_law_ (sample_time, k_p, k_i, k_d, U_AUX_MAX, type)
{}

PIDLin::~PIDLin(){};

void PIDLin::calculateManipulations(float chi1)
{
    control_law_.calculateManipulation(chi1);

    u_aux_ = -control_law_.u_;

    updateControlSignal();
}

void PIDLin::updateReferences(float chi1_d, float chi1_dot_d)
{
    control_law_.updateReferences(chi1_d);

    chiX_dot_d_ = chi1_dot_d;
}