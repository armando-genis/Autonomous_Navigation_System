/** ----------------------------------------------------------------------------
 * @file: antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/SMC_based/ANTSMC/antsmc.hpp"

ANTSMC::ANTSMC(float sample_time,float alpha,float beta,float K2,float K_alpha,float K_min,float K1_init,float mu, const DOFControllerType_E& type)
{
    sample_time_ = sample_time_;
    chi1_d = 0.0;
    chi1_dot_d = 0.0;
    error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_1_ = 0.0;
    prev_error_2_ = 0.0;
    
    // Auxiliar control
    ua_ = 0.0;

    // Control parameters
    alpha_ = alpha;
    beta_ = beta;
    s_ = 0.0;
    delta_ = 0.0;

    // Gains
    K1_ = K1_init;
    K2_ = K2;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;

    // Adaptive law
    K_min_ = K_min;
    K_alpha_ = K_alpha;
    mu_ = mu;

    controller_type_ = type;
}

ANTSMC::~ANTSMC(){}

void ANTSMC::reset()
{
    error_1_ = 0.0;
    prev_error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_2_ = 0.0;
    ua_ = 0.0;
    K1_ = 0.0;
}

void ANTSMC::updateReferences(float q_d,float q_dot_d)
{
    chi1_d = q_d;
    chi1_dot_d = q_dot_d;
}

void ANTSMC::calculateAuxControl(float q,float q_dot)
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

    delta_ = utils::sig(error_2_, 2-beta_)/(alpha_*beta_);

    s_ = error_1_ + alpha_*utils::sig(error_2_, beta_);
    
    dot_K1_ = K1_ > K_min_ ?  K_alpha_*utils::sign(std::fabs(s_) - mu_) : K_min_;
    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;
    ua_ = -K1_*utils::sig(s_, 0.5) - K2_*s_;

}