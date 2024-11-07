/** ----------------------------------------------------------------------------
 * @file: pid_6dof.hpp
 * @date: May 12, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Second Order 6-DOF PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/PID/second_order/pid_6dof.hpp"

PID6DOF::PID6DOF(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                 const std::vector<float>& k_d, const std::array<float,6>& u_max, 
                 const std::array<DOFControllerType_E,6>& type) : 
                 PID_x_    (sample_time, k_p[0], k_i[0], k_d[0], u_max[0], type[0]),
                 PID_y_    (sample_time, k_p[1], k_i[1], k_d[1], u_max[1], type[1]),
                 PID_z_    (sample_time, k_p[2], k_i[2], k_d[2], u_max[2], type[2]),
                 PID_phi_  (sample_time, k_p[3], k_i[3], k_d[3], u_max[3], type[3]),
                 PID_theta_(sample_time, k_p[4], k_i[4], k_d[4], u_max[4], type[4]),
                 PID_psi_  (sample_time, k_p[5], k_i[5], k_d[5], u_max[5], type[5])
{
    u_ = Eigen::MatrixXf::Zero(6,1);
}

PID6DOF::~PID6DOF(){};

// void PID6DOF::calculateManipulations(const std::array<float,6>& chi1, const std::array<float,6>& chi2)
void PID6DOF::calculateManipulations(const std::array<float,6>& chi1, const std::array<float,6>& chi2)
{
    PID_x_.calculateManipulation(chi1[0],chi2[0]);
    PID_y_.calculateManipulation(chi1[1],chi2[1]);
    PID_z_.calculateManipulation(chi1[2],chi2[2]);
    PID_phi_.calculateManipulation(chi1[3],chi2[3]);
    PID_theta_.calculateManipulation(chi1[4],chi2[4]);
    PID_psi_.calculateManipulation(chi1[5],chi2[5]);

    u_ << PID_x_.u_,
          PID_y_.u_,
          PID_z_.u_,
          PID_phi_.u_,
          PID_theta_.u_,
          PID_psi_.u_;
}

void PID6DOF::updateReferences(const std::array<float,6>& chi1_d, const std::array<float,6>& chi2_d)
{
    PID_x_.updateReferences(chi1_d[0],chi2_d[0]);
    PID_y_.updateReferences(chi1_d[1],chi2_d[1]);
    PID_z_.updateReferences(chi1_d[2],chi2_d[2]);
    PID_phi_.updateReferences(chi1_d[3],chi2_d[3]);
    PID_theta_.updateReferences(chi1_d[4],chi2_d[4]);
    PID_psi_.updateReferences(chi1_d[5],chi2_d[5]);
}