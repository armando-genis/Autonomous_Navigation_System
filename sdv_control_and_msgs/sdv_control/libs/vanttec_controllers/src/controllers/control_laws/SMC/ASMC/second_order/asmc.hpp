/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Second Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include "utils/utils.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"
// #include "vanttec_msgs/EtaPose.h"
#include <cmath>

typedef struct 
{
    float lambda;
    float K2;
    float K_alpha;
    float K1_init;
    float K_min;
    float mu;
    float u_max;
    DOFControllerType_E type;
} ASMC_Config;

class ASMC
{
    private:
        float sample_time_;

        float error_1_;
        float error_2_;
        
        float prev_error_1_;
        float prev_error_2_;

        /* Setpoint */
        float chi1_d;
        float chi1_dot_d;
        
        /* Auxiliar control */
        float u_;
        float U_MAX_;

        /* Sliding surface */
        float lambda_;
        float s_;

        /* Gains */
        float K1_;
        float K2_;
        float dot_K1_;
        float prev_dot_K1_;

        /* Adaptive law */
        float K_min_;
        float K_alpha_;
        float mu_;

        DOFControllerType_E controller_type_;

    public:
        // Constructor
        ASMC(float sample_time, const ASMC_Config& config);

        // Destructor
        ~ASMC();

        void reset();
        void updateReferences(float q_d, float q_dot_d);
        void calculateManipulation(float q, float q_dot);
        
        // Saturate manipulation function is intended to be used in applications where a FBLin ASMC is not required,
        // as FBLin base classes already saturate the control signals
        void saturateManipulation(float q, float q_dot);
        // friend class ASMC6DOF;
};

#endif