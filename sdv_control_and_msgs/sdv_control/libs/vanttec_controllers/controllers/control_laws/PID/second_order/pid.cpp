/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: Single DOF PID Second Order Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/PID/second_order/pid.hpp"

PID::PID(float sample_time,float k_p,float k_i,float k_d, float u_max, const DOFControllerType_E& type)
{
    k_p_            = k_p;
    k_i_            = k_i;
    k_d_            = k_d;

    error_          = 0;
    prev_error_     = 0;
    chi1_d_         = 0;
    chi2_d_         = 0;
    u_              = 0;

    U_MAX_ = u_max;

    controller_type_   = type;
}

PID::~PID(){}

void PID::updateReferences(float chi1_d, float chi2_d)
{
    chi1_d_ = chi1_d;
    chi2_d_ = chi2_d;
}

void PID::calculateManipulation(float chi1, float chi2)
{
    float error_i;
    float u;

    prev_error_    = error_;
    error_         = chi1_d_ - chi1;
    error_d_       = chi2_d_ - chi2;

    if (controller_type_ == ANGULAR_DOF)
        if (std::abs(error_) > M_PI)
            // error_ = error_ - ((error_ / std::abs(error_)) * 2 * M_PI);
            error_ = (error_ / std::abs(error_)) * (std::abs(error_) - 2 * M_PI);

    error_i = ((error_ + prev_error_) / 2 * sample_time_) + error_;

    // std::cout << "e1 = " << error_ << std::endl;
    // std::cout << "e2 = " << error_d_ << std::endl;

    u  = k_p_ * error_ + k_i_ * error_i + k_d_ * error_d_;

    if(std::isnormal(u))
        u_ = u;
}

// Saturate manipulation function is intended to be used in applications where a FBLin PID is not required,
// as FBLin base classes already saturate the control signals
void PID::saturateManipulation(float chi1, float chi2)
{
    calculateManipulation(chi1, chi2);
    u_ = std::fabs(u_) > U_MAX_ ? u_ / std::fabs(u_) * U_MAX_ : u_;
}