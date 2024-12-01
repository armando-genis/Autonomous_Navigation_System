/** ----------------------------------------------------------------------------
 * @file: 6dof_antsmc.cpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive non-singular terminal sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#include "6dof_antsmc.hpp"

ANTSMC6DOF::ANTSMC6DOF(float sample_time, const float* alpha, const float* beta_, const float* K2, const float* K_alpha, const float* K_min, const float* K1_init, const float* mu)
                    : ANTSMC_x_(sample_time, alpha[0], beta_[0], K2[0], K_alpha[0], K_min[0], K1_init[0], mu[0], LINEAR_DOF)
                    , ANTSMC_y_(sample_time, alpha[1], beta_[1], K2[1], K_alpha[1], K_min[1], K1_init[1], mu[1], LINEAR_DOF)
                    , ANTSMC_z_(sample_time, alpha[2], beta_[2], K2[2], K_alpha[2], K_min[2], K1_init[2], mu[2], LINEAR_DOF)
                    , ANTSMC_phi_(sample_time, alpha[3], beta_[3], K2[3], K_alpha[3], K_min[3], K1_init[3], mu[3], ANGULAR_DOF)
                    , ANTSMC_theta_(sample_time, alpha[4], beta_[4], K2[4], K_alpha[4], K_min[4], K1_init[4], mu[4], ANGULAR_DOF)
                    , ANTSMC_psi_(sample_time, alpha[5], beta_[5], K2[5], K_alpha[5], K_min[5], K1_init[5], mu[5], ANGULAR_DOF)
{
    u_.resize(6,1);              // Control
    ua_.resize(6,1);             // Auxiliary Control
    delta_.resize(6,1);
    f_.resize(6,1);
    g_.resize(6,6);
    q_dot_dot_.resize(6,1);

    u_ << 0,
         0,
         0,
         0,
         0,
         0;

    ua_ << 0,
          0,
          0,
          0,
          0,
          0;

    delta_ << 0,
             0,
             0,
             0,
             0,
             0;

    q_dot_dot_ << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    g_x_ = Eigen::MatrixXf::Zero(6,6);

    f_ << 0,
         0,
         0,
         0,
         0,
         0;

    MAX_FORCE_X_ = 0.0;
    MAX_FORCE_Y_ = 0.0;
    MAX_FORCE_Z_ = 0.0;
    MAX_TORQUE_K_ = 0.0;
    MAX_TORQUE_M_ = 0.0;
    MAX_TORQUE_N_ = 0.0;

    thrust_.tau_x = 0;
    thrust_.tau_y = 0;
    thrust_.tau_z = 0;
    thrust_.tau_phi = 0;
    thrust_.tau_theta = 0;
    thrust_.tau_psi = 0;
    
    // functs_arriveD_= 0;
}

ANTSMC6DOF::~ANTSMC6DOF(){}

void ANTSMC6DOF::setMaxThrust(float* MAX_TAU){
    MAX_FORCE_X_ = MAX_TAU[0];
    MAX_FORCE_Y_ = MAX_TAU[1];
    MAX_FORCE_Z_ = MAX_TAU[2];
    MAX_TORQUE_K_ = MAX_TAU[3];
    MAX_TORQUE_M_ = MAX_TAU[4];
    MAX_TORQUE_N_ = MAX_TAU[5];
}


void ANTSMC6DOF::updateReferencess(const vanttec_msgs::EtaPose& q_d)//, const vanttec_msgs::EtaPose& q_dot_d)
{
    ANTSMC_x_.updateReferences(q_d.x, 0.0);        //, q_dot_d.x, 0.0);
    ANTSMC_y_.updateReferences(q_d.y, 0.0);        //, q_dot_d.y, 0.0);
    ANTSMC_z_.updateReferences(q_d.z, 0.0);        //, q_dot_d.z, 0.0);
    ANTSMC_phi_.updateReferences(q_d.phi, 0.0);        //, q_dot_d.phi);
    ANTSMC_theta_.updateReferences(q_d.theta, 0.0);        //, q_dot_d.theta);
    ANTSMC_psi_.updateReferences(q_d.psi, 0.0);        //, q_dot_d.psi);
}

void ANTSMC6DOF::updatePose(const vanttec_msgs::EtaPose& q)//, const vanttec_msgs::EtaPose& q_dot)
{
    ANTSMC_x_.calculateAuxControl(q.x, 0.0);     //, q_dot.x);
    ANTSMC_y_.calculateAuxControl(q.y, 0.0);     //, q_dot.y);
    ANTSMC_z_.calculateAuxControl(q.z, 0.0);     //, q_dot.z);
    ANTSMC_phi_.calculateAuxControl(q.phi, 0.0);     //, q_dot.phi);
    ANTSMC_theta_.calculateAuxControl(q.theta, 0.0);     //, q_dot.theta);
    ANTSMC_psi_.calculateAuxControl(q.psi, 0.0);     //, q_dot.psi);
}

void ANTSMC6DOF::calculateManipulation()
{
    Eigen::FullPivLU<Eigen::MatrixXf> glu(g_);
    // if(functs_arrived){
    if (glu.isInvertible()) {
        ua_ << ANTSMC_x_.ua_,
              ANTSMC_y_.ua_,
              ANTSMC_z_.ua_,
              ANTSMC_phi_.ua_,
              ANTSMC_theta_.ua_,
              ANTSMC_psi_.ua_;
        delta_ << ANTSMC_x_.delta_,
                 ANTSMC_y_.delta_,
                 ANTSMC_z_.delta_,
                 ANTSMC_phi_.delta_,
                 ANTSMC_theta_.delta_,
                 ANTSMC_psi_.delta_;
        u_ << g_.inverse()*(q_dot_dot_ - f_ + delta_ - ua_);

        // Saturate for maximum thrust in each degree of freedom
        if(std::fabs(u_(0)) > MAX_FORCE_X_)  u_(0) = u_(0) / std::fabs(u_(0)) * MAX_FORCE_X_;
        if(std::fabs(u_(1)) > MAX_FORCE_Y_)  u_(1) = u_(1) / std::fabs(u_(1)) * MAX_FORCE_Y_;
        if(std::fabs(u_(2)) > MAX_FORCE_Z_)  u_(2) = u_(2) / std::fabs(u_(2)) * MAX_FORCE_Z_;
        if(std::fabs(u_(3)) > MAX_TORQUE_K_) u_(3) = u_(3) / std::fabs(u_(3)) * MAX_TORQUE_K_;
        if(std::fabs(u_(4)) > MAX_TORQUE_M_) u_(4) = u_(4) / std::fabs(u_(4)) * MAX_TORQUE_M_;
        if(std::fabs(u_(5)) > MAX_TORQUE_N_) u_(5) = u_(5) / std::fabs(u_(5)) * MAX_TORQUE_N_;
    
        thrust_.tau_x = u_(0);
        thrust_.tau_y = u_(1);
        thrust_.tau_z = u_(2);
        thrust_.tau_phi = u_(3);
        thrust_.tau_theta = u_(4);
        thrust_.tau_psi = u_(5);
    }
}

void ANTSMC6DOF::updateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions)
{
    uint8_t stride =  (uint8_t) non_linear_functions.g.layout.dim[0].stride;
    uint8_t offset =  (uint8_t) non_linear_functions.g.layout.data_offset;

    for(int i=0; i<6; ++i){
        for(int j=0; j<6; ++j){
            g_(i,j) = non_linear_functions.g.data[offset + i*stride + j];
        }
    }

    f_ << non_linear_functions.f[0],
         non_linear_functions.f[1],
         non_linear_functions.f[2],
         non_linear_functions.f[3],
         non_linear_functions.f[4],
         non_linear_functions.f[5];

    // std::cout << "f:" << f_ << std::endl;
    // std::cout << "g:" << g << std::endl;

    // functs_arriveD_= 1;
}