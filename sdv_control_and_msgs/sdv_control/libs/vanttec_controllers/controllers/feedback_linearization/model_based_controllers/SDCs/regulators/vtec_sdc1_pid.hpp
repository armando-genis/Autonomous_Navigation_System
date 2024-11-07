/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_pid.hpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a single DOF PID Controller for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"
#include "controllers/feedback_linearization/PID_based/first_order/fblin_pid.hpp"

class VTEC_SDC1_1DOF_PID : public VTecSDC1DynamicModel, public PIDLin
{
    public:
        VTEC_SDC1_1DOF_PID( float sample_time, float k_p, float k_i, float k_d,
                            float u_max, const DOFControllerType_E& type, uint8_t D_max);
        ~VTEC_SDC1_1DOF_PID();

        void updateNonLinearFunctions();

        // For simulations
        void calculateControlSignals();

        // For real world
        void calculateControlSignals(float chi1);

        void updateControlSignals();

        void updateCurrentReference(float chi1_d, float chi1_dot_d);
};