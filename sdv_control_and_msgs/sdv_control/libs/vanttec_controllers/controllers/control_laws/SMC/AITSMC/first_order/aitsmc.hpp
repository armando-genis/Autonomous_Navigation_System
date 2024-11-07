/** ----------------------------------------------------------------------------
 * @file: aitsmc.hpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 * @author: Sebas Mtz
 * 
 * @brief: First Order Adaptive Integral Terminal Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __AITSMC_H__
#define __AITSMC_H__

#include "utils/utils.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"
#include <cmath>

typedef struct 
{
    /* Sliding surface */
    float lambda;
    float beta;

    /* Adaptive law*/
    float K_min;
    float K_alpha;

    /* Gains */
    float K1_init;
    float K2;

    float mu;
    float U_MAX;
    
    DOFControllerType_E controller_type;
} AITSMC_Params;

class AITSMC
{
    protected:
        float sample_time_;
        
        /* Adaptive Law Gains */
        float dot_K1_;
        float prev_dot_K1_;
        
        bool init_val_{false};

    public:
        /* Auxiliar control */
        float u_;

        /* Sliding surface */
        float s_;
        
        /* Adaptive Law Gains */
        float K1_;

        /* Setpoint */
        float chi1_d;

        /* Errors */
        float error_;
        float error_I_;
        float error_I_dot_;
        float prev_error_I_dot_;
        
        AITSMC_Params params_;

        // Constructor
        AITSMC(float sample_time, const AITSMC_Params& config);

        // Destructor
        ~AITSMC();

        void reset();
        void updateReferences(float q_d);
        void calculateManipulation(float q);

        // Saturate manipulation function is intended to be used in applications where a FBLin AITSMC is not required,
        // as FBLin base classes already saturate the control signals
        void saturateManipulation(float q);
};

#endif