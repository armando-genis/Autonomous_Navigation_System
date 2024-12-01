/** ----------------------------------------------------------------------------
 * @file: antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ANTSMC_H__
#define __ANTSMC_H__

#include "utils/utils.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <cmath>

class ANTSMC
{
    private:
        float sample_time_;
        float chi1_d;         // Setpoint
        float chi1_dot_d;
        float error_1_;
        float error_2_;
        float prev_error_1_;
        float prev_error_2_;
        
        // Auxiliar control
        float ua_;

        // Control parameters
        float alpha_;
        float beta_;
        float s_;
        float delta_;

        // Gains
        float K1_;
        float K2_;
        float dot_K1_;
        float prev_dot_K1_;

        // Adaptive law
        float K_min_;
        float K_alpha_;
        float mu_;
        
        DOFControllerType_E controller_type_;
    public:
        // Constructor
        ANTSMC(float sample_time,float alpha,float beta_,float K2,float K_alpha,float K_min,float K1_init,float mu, const DOFControllerType_E& type);

        // Destructor
        ~ANTSMC();

        void reset();
        void updateReferences(float q_d,float q_dot_d);
        void calculateAuxControl(float q,float q_dot);

        friend class ANTSMC6DOF;
};

#endif