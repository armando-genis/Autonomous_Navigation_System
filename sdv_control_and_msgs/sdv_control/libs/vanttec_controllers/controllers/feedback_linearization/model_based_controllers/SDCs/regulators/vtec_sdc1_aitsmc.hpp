/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_aitsmc.hpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 * 
 * @brief: Description of a single DOF AITSMC for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"
#include "controllers/feedback_linearization/SMC_based/AITSMC/first_order/fblin_aitsmc.hpp"

class VTEC_SDC1_1DOF_AITSMC : public VTecSDC1DynamicModel, public AITSMCLin
{
    public:
        VTEC_SDC1_1DOF_AITSMC(float sample_time, const AITSMC_Params& config, float u_max, uint8_t D_max);
        ~VTEC_SDC1_1DOF_AITSMC();

        void updateNonLinearFunctions();

        // For simulations
        void calculateControlSignals();

        // For real world
        void calculateControlSignals(float chi1);

        void updateControlSignals();

        void updateCurrentReference(float chi1_d, float chi1_dot_d);
};