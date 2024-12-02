/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Second Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/SMC/ASMC/second_order/asmc.hpp"

ASMC::ASMC( float sample_time, const ASMC_Config& config)
{
    sample_time_ = sample_time;
    chi1_d = 0.0;
    chi1_dot_d = 0.0;
    error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_1_ = 0.0;
    prev_error_2_ = 0.0;
    
    // Auxiliar control
    u_ = 0.0;
    U_MAX_ = config.u_max;

    // Sliding surface
    lambda_ = config.lambda;
    s_ = 0.0;

    // Gains
    K1_ = config.K1_init;
    K2_ = config.K2;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;

    // Adaptive law
    K_min_ = config.K_min;
    K_alpha_ = config.K_alpha;
    mu_ = config.mu;

    controller_type_ = config.type;
}

ASMC::~ASMC(){}

void ASMC::reset()
{
    error_1_ = 0.0;
    prev_error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_2_ = 0.0;
    u_ = 0.0;
    K1_ = 0.0;
}

void ASMC::updateReferences(float q_d, float q_dot_d)
{
    chi1_d = q_d;
    chi1_dot_d = q_dot_d;
}


void ASMC::calculateAuxControl(float q, float q_dot)
{
    prev_error_1_ = error_1_;
    prev_error_2_ = error_2_;
    prev_dot_K1_ = dot_K1_;

    error_1_ = chi1_d - q;
    error_2_ = chi1_dot_d - q_dot;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::fabs(error_1_) > M_PI)
        {
            error_1_ = (error_1_ / std::fabs(error_1_)) * (std::fabs(error_1_) - 2 * M_PI);
        }
        if (std::fabs(error_2_) > M_PI)
        {
            error_2_ = (error_2_ / std::fabs(error_2_)) * (std::fabs(error_2_) - 2 * M_PI);
        }
    }

    // Checar que calc de sign est[e bien]
    s_ = error_2_ + lambda_*error_1_;

    dot_K1_ = K1_ > K_min_ ?  K_alpha_*static_cast<float>(utils::sign(std::fabs(s_) - mu_)) : K_min_;

    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;

    u_ = K1_*utils::sig(s_, 0.5) + K2_*s_;
}

// Saturate manipulation function is intended to be used in applications where a FBLin ASMC is not required,
// as FBLin base classes already saturate the control signals
void ASMC::saturateManipulation(float q, float q_dot)
{
    calculateManipulation(q, q_dot);
    u_ = std::fabs(u_) > U_MAX_ ? u_ / std::fabs(u_) * U_MAX_ : u_;
}