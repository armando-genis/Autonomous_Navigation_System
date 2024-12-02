/** ----------------------------------------------------------------------------
 * @file: fblin_asmc.hpp
 * @date: August 15, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization ASMC controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_ASMC_H__
#define __FBLIN_ASMC_H__

#include "controllers/feedback_linearization/base/fb_lin_control.hpp"
#include "controllers/control_laws/SMC/ASMC/first_order/asmc.hpp"

class ASMCLin : public FBLin
{
    private:
        ASMC control_law_;
        float lambda_;

    public:
        ASMCLin(float sample_time, const ASMC_Config& config, float lambda, float u_max);
        ~ASMCLin();

        void calculateManipulations(float chi1);
        void updateReferences(float chi1_d, float chi1_dot_d);
};

#endif