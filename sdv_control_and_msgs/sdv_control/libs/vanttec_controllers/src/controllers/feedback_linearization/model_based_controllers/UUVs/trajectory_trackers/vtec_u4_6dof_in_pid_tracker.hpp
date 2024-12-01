/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_in_pid.hpp
 * @date: April 27, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a 6DOF PID Trajectory Tracker Controller for the VTec U4 inertial dyn model
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.hpp"
#include "controllers/feedback_linearization/PID_based/second_order/fblin_6dof_pid.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "vanttec_msgs/EtaPose.h"
#include "vanttec_msgs/Trajectory.h"

class VTEC_U4_6DOF_PID_TRACKER : public VTecU4InDynamicModel, public PID6DOFLin
{
    public:
        vanttec_msgs::Trajectory reference_;

        VTEC_U4_6DOF_PID_TRACKER(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                const std::vector<float>& k_d, const std::array<float,6>& u_max,
                const std::array<DOFControllerType_E,6>& type);
        ~VTEC_U4_6DOF_PID_TRACKER();

        void updateNonLinearFunctions();

        void calculateControlSignals();

        void updateControlSignals();

        void updateTrajectoryReference(const vanttec_msgs::Trajectory& trajectory);

        void updateCurrentReference();
};