/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_in_pid.cpp
 * @date: April 27, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a 6DOF PID Trajectory Tracker Controller for the VTec U4 inertial dyn model
 * -----------------------------------------------------------------------------
 **/

#include "controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.hpp"

VTEC_U4_6DOF_PID_TRACKER::VTEC_U4_6DOF_PID_TRACKER(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
        const std::vector<float>& k_d, const std::array<float,6>& u_max,
        const std::array<DOFControllerType_E,6>& type) :
        VTecU4InDynamicModel(sample_time), PID6DOFLin(sample_time, k_p, k_i, k_d, u_max, type)
{}

VTEC_U4_6DOF_PID_TRACKER::~VTEC_U4_6DOF_PID_TRACKER(){}

void VTEC_U4_6DOF_PID_TRACKER::updateNonLinearFunctions()
{
    PID6DOFLin::f_x_ = VTecU4InDynamicModel::f_x_;
    PID6DOFLin::g_x_ = VTecU4InDynamicModel::g_x_;
}

void VTEC_U4_6DOF_PID_TRACKER::calculateControlSignals()
{
    std::array<float,6> chi1, chi2;

    chi1[0] = eta_(0);
    chi1[1] = eta_(1);
    chi1[2] = eta_(2);
    chi1[3] = eta_(3);
    chi1[4] = eta_(4);
    chi1[5] = eta_(5);

    chi2[0] = eta_dot_(0);
    chi2[1] = eta_dot_(1);
    chi2[2] = eta_dot_(2);
    chi2[3] = eta_dot_(3);
    chi2[4] = eta_dot_(4);
    chi2[5] = eta_dot_(5);

    calculateManipulations(chi1, chi2);
}

void VTEC_U4_6DOF_PID_TRACKER::updateControlSignals()
{
    VTecU4InDynamicModel::u_ = PID6DOFLin::u_;
}

size_t idx = 0;
size_t len = 0;

void VTEC_U4_6DOF_PID_TRACKER::updateTrajectoryReference(const vanttec_msgs::Trajectory& trajectory)
{
    if(reference_.execution_time != trajectory.execution_time) // There must be a better way to know if
    {                                                           // two trajectories are different, but works for now
        reference_ = trajectory;
        idx = 0;
        len = reference_.jerk.size();   // jerk, accel, vel and pose arrays should be the same size
    }
}

void VTEC_U4_6DOF_PID_TRACKER::updateCurrentReference()
{
    if(reference_.execution_time != 0 && idx < len)
    {
        geometry_msgs::Accel accel = reference_.accel.at(idx);
        geometry_msgs::Twist vel   = reference_.vel.at(idx);
        vanttec_msgs::EtaPose pose = reference_.eta_pose.at(idx);

        std::array<float,6> chi1_d = {(float) pose.x,
                                    (float) pose.y,
                                    (float) pose.z,
                                    (float) pose.phi,
                                    (float) pose.theta,
                                    (float) pose.psi};

        std::array<float,6> chi2_d = {(float) vel.linear.x,
                                    (float) vel.linear.y,
                                    (float) vel.linear.z,
                                    (float) vel.angular.x,
                                    (float) vel.angular.y,
                                    (float) vel.angular.z};

        std::array<float,6> chi2_dot_d = {(float) accel.linear.x,
                                        (float) accel.linear.y,
                                        (float) accel.linear.z,
                                        (float) accel.angular.x,
                                        (float) accel.angular.y,
                                        (float) accel.angular.z};

        updateReferences(chi1_d, chi2_d, chi2_dot_d);

        idx++;
    }
}