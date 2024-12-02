/** ----------------------------------------------------------------------------
 * @file: aitsmc.cpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: First Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/SMC/AITSMC/first_order/aitsmc.hpp"

AITSMC::AITSMC(float sample_time, const AITSMC_Params& params )
{
    sample_time_ = sample_time;
    
    error_ = 0.0;

    /* Setpoint */
    chi1_d = 0.0;
    
    params_ = params;

    /* Auxiliar control */
    u_ = 0.0;

    /* Sliding surface */
    s_ = 0.0;

    /* Gains */
    K1_ = params.K1_init;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;
}

AITSMC::~AITSMC(){}

void AITSMC::reset()
{
    error_ = 0.0;
    error_I_ = 0.0;
    error_I_dot_ = 0.0;
    prev_error_I_dot_ = 0.0;

    u_ = 0.0;
    K1_ = 0.0;
}

void AITSMC::updateReferences(float q_d)
{
    chi1_d = q_d;
}

void AITSMC::calculateManipulation(float q)
{
    prev_error_I_dot_ = error_I_dot_;
    prev_dot_K1_ = dot_K1_;

    error_ = chi1_d - q;

    if (params_.controller_type == ANGULAR_DOF)
    {
        if (std::fabs(error_) > M_PI)
        {
            error_ = (error_ / std::fabs(error_)) * (std::fabs(error_) - 2 * M_PI);
        }
    }

    error_I_dot_ = utils::sig(error_, params_.beta);

    if(!init_val_){
        error_I_ = -error_/params_.lambda;;
        init_val_ = true;
    }

    error_I_ += (error_I_dot_ + prev_error_I_dot_) / 2 * sample_time_;

    s_ =  error_ + params_.lambda*error_I_;

    // Adaptive law
    dot_K1_ = K1_ > params_.K_min ?  params_.K_alpha*static_cast<float>(utils::sign(std::fabs(s_) - params_.mu)) : params_.K_min;
    
    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;
    
    u_ = K1_*utils::sig(s_, 0.5) + params_.K2*s_;
}

// Saturate manipulation function is intended to be used in applications where a FBLin AITSMC is not required,
// as FBLin base classes already saturate the control signals
void AITSMC::saturateManipulation(float q)
{
    calculateManipulation(q);
    u_ = std::fabs(u_) > params_.U_MAX ? u_ / std::fabs(u_) * params_.U_MAX : u_;
}