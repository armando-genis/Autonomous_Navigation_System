/** ----------------------------------------------------------------------------
 * @file: fblin_aitsmc.hpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 * 
 * @brief: 1-DOF feedback linearization ASMC controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_ASMC_H__
#define __FBLIN_ASMC_H__

#include "controllers/feedback_linearization/base/fb_lin_control.hpp"
#include "controllers/control_laws/SMC/AITSMC/first_order/aitsmc.hpp"

class AITSMCLin : public FBLin
{
    public:
        AITSMC control_law_;

        AITSMCLin(float sample_time, const AITSMC_Params& config, float u_max);
        ~AITSMCLin();

        void calculateManipulations(float chi1);
        void updateReferences(float chi1_d, float chi1_dot_d);
};

#endif