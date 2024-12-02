/** ----------------------------------------------------------------------------
 * @file: pid_6dof.hpp
 * @date: May 12, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: First Order 6-DOF PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __PID_6DOF_H__
#define __PID_6DOF_H__

#include "controllers/control_laws/PID/first_order/pid.hpp"

class PID6DOF
{
    private:
        Eigen::VectorXf u_;
        
        PID PID_x_;
        PID PID_y_;
        PID PID_z_;
        PID PID_phi_;
        PID PID_theta_;
        PID PID_psi_;

    public:
        PID6DOF(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                const std::vector<float>& k_d, const std::array<float,6>& u_max,
                const std::array<DOFControllerType_E,6>& type);
        ~PID6DOF();

        void calculateManipulations(const std::array<float,6>& chi1, const std::array<float,6>& chi2);
        void updateReferences(const std::array<float,6>& chi1_d, const std::array<float,6>& chi2_d);
};

#endif