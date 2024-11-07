/** ----------------------------------------------------------------------------
 * @file: fblin_6dof_pid.hpp
 * @date: April 25, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/PID_based/second_order/fblin_6dof_pid.hpp"

std::array<float,6> U_AUX_MAX {FLT_MAX,6};

PID6DOFLin::PID6DOFLin(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                 const std::vector<float>& k_d, const std::array<float,6>& u_max, 
                 const std::array<DOFControllerType_E,6>& type) : 
                 FBLin6DOF (u_max),
                 control_law_ (sample_time, k_p, k_i, k_d, U_AUX_MAX, type)
{}

PID6DOFLin::~PID6DOFLin(){};

void PID6DOFLin::calculateManipulations(const std::array<float,6>& chi1, const std::array<float,6>& chi2)
{
    control_law_.calculateManipulations(chi1,chi2);

    u_aux_ << -control_law_.PID_x_.u_,
              -control_law_.PID_y_.u_,
              -control_law_.PID_z_.u_,
              -control_law_.PID_phi_.u_,
              -control_law_.PID_theta_.u_,
              -control_law_.PID_psi_.u_;

    updateControlSignal();
}

void PID6DOFLin::updateReferences(const std::array<float,6>& chi1_d, const std::array<float,6>& chi2_d, const std::array<float,6>& chi2_dot_d)
{
    control_law_.updateReferences(chi1_d, chi2_d);

    chiX_dot_d_ << chi2_dot_d[0],
                   chi2_dot_d[1],
                   chi2_dot_d[2],
                   chi2_dot_d[3],
                   chi2_dot_d[4],
                   chi2_dot_d[5];
}