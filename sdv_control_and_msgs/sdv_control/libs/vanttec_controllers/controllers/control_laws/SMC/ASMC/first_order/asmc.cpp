/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: First Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/SMC/ASMC/first_order/asmc.hpp"

ASMC::ASMC(float sample_time, const ASMC_Config& config )
{
    sample_time_ = sample_time;
    
    error_ = 0.0;
    prev_error_ = 0.0;

    /* Setpoint */
    chi1_d = 0.0;
    
    /* Auxiliar control */
    u_ = 0.0;
    U_MAX_ = config.u_max;

    /* Sliding surface */
    lambda_ = config.lambda;
    s_ = 0.0;

    /* Gains */
    K1_ = config.K1_init;
    K2_ = config.K2;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;

    /* Adaptive law */
    K_min_ = config.K_min;
    K_alpha_ = config.K_alpha;
    mu_ = config.mu;

    controller_type_ = config.type;
}

ASMC::~ASMC(){}

void ASMC::reset()
{
    error_ = 0.0;
    prev_error_ = 0.0;
    u_ = 0.0;
    K1_ = 0.0;
}

void ASMC::updateReferences(float q_d)
{
    chi1_d = q_d;
}

void ASMC::calculateManipulation(float q)
{
    float error_dot;
    prev_error_ = error_;
    prev_dot_K1_ = dot_K1_;

    error_ = chi1_d - q;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::fabs(error_) > M_PI)
        {
            error_ = (error_ / std::fabs(error_)) * (std::fabs(error_) - 2 * M_PI);
        }
    }

    error_dot = (error_ - prev_error_) / sample_time_;

    // Checar que calc de sign est[e bien]
    s_ = error_dot + lambda_*error_;

    // Adaptive law
    dot_K1_ = K1_ > K_min_ ?  K_alpha_*static_cast<float>(utils::sign(std::fabs(s_) - mu_)) : K_min_;
    
    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;
    
    u_ = K1_*utils::sig(s_, 0.5) + K2_*s_;
}

// Saturate manipulation function is intended to be used in applications where a FBLin ASMC is not required,
// as FBLin base classes already saturate the control signals
void ASMC::saturateManipulation(float q)
{
    calculateManipulation(q);
    u_ = std::fabs(u_) > U_MAX_ ? u_ / std::fabs(u_) * U_MAX_ : u_;
}