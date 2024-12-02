/** ----------------------------------------------------------------------------
 * @file: fblin_asmc.hpp
 * @date: August 15, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization ASMC controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/SMC_based/ASMC/first_order/fblin_asmc.hpp"

ASMCLin::ASMCLin(float sample_time, const ASMC_Config& config, float lambda, float u_max) : 
                FBLin (u_max),
                control_law_ (sample_time, config),
                lambda_(lambda)
{}

ASMCLin::~ASMCLin(){};

void ASMCLin::calculateManipulations(float chi1)
{
    control_law_.calculateManipulation(chi1);

    u_aux_ = -control_law_.u_;
    u_n_ = lambda_*control_law_.error_;

    updateControlSignal();
}

void ASMCLin::updateReferences(float chi1_d, float chi1_dot_d)
{
    control_law_.updateReferences(chi1_d);

    chiX_dot_d_ = chi1_dot_d;
}