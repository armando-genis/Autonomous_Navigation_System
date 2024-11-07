/** ----------------------------------------------------------------------------
 * @file: fblin_6dof_pid.hpp
 * @date: April 25, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_6DOF_PID_H__
#define __FBLIN_6DOF_PID_H__

#include "controllers/feedback_linearization/base/fb_lin_6dof_control.hpp"
#include "controllers/control_laws/PID/second_order/pid_6dof.hpp"

#include <ros/ros.h>

class PID6DOFLin : public FBLin6DOF
{
    private:
        PID6DOF control_law_;

    public:
        PID6DOFLin(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                const std::vector<float>& k_d, const std::array<float,6>& u_max,
                const std::array<DOFControllerType_E,6>& type);
        ~PID6DOFLin();

        void calculateManipulations(const std::array<float,6>& chi1, const std::array<float,6>& chi2);
        void updateReferences(const std::array<float,6>& chi1_d, const std::array<float,6>& chi2_d, const std::array<float,6>& chi2_dot_d);
};

#endif