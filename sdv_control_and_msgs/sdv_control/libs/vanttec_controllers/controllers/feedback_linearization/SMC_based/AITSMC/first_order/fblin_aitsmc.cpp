/** ----------------------------------------------------------------------------
 * @file: fblin_aitsmc.hpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 * 
 * @brief: 1-DOF feedback linearization ASMC controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/SMC_based/AITSMC/first_order/fblin_aitsmc.hpp"

AITSMCLin::AITSMCLin(float sample_time, const AITSMC_Params& config, float u_max) : 
                FBLin (u_max),
                control_law_ (sample_time, config)
{}

AITSMCLin::~AITSMCLin(){};

void AITSMCLin::calculateManipulations(float chi1)
{
    control_law_.calculateManipulation(chi1);

    u_aux_ = -control_law_.u_;
    u_n_ = control_law_.params_.lambda*control_law_.error_I_dot_;

    updateControlSignal();
}

void AITSMCLin::updateReferences(float chi1_d, float chi1_dot_d)
{
    control_law_.updateReferences(chi1_d);

    chiX_dot_d_ = chi1_dot_d;
}