/** ----------------------------------------------------------------------------
 * @file: fblin_pid.hpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_PID_H__
#define __FBLIN_PID_H__

#include "controllers/feedback_linearization/base/fb_lin_control.hpp"
#include "controllers/control_laws/PID/first_order/pid.hpp"

class PIDLin : public FBLin
{
    private:
        PID control_law_;

    public:
        PIDLin(float sample_time, float k_p, float k_i, float k_d,
        float u_max, const DOFControllerType_E& type);
        ~PIDLin();

        void calculateManipulations(float chi1);
        void updateReferences(float chi1_d,float chi1_dot_d);
};

#endif