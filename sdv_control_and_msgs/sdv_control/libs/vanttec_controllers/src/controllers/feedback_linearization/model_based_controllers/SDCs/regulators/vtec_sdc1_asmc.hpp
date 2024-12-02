/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_asmc.hpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a single DOF ASMC for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"
#include "controllers/feedback_linearization/SMC_based/ASMC/first_order/fblin_asmc.hpp"

class VTEC_SDC1_1DOF_ASMC : public VTecSDC1DynamicModel, public ASMCLin
{
    public:
        VTEC_SDC1_1DOF_ASMC(float sample_time, const ASMC_Config& config, float lambda, float u_max, uint8_t D_max);
        ~VTEC_SDC1_1DOF_ASMC();

        void updateNonLinearFunctions();

        // For simulations
        void calculateControlSignals();

        // For real world
        void calculateControlSignals(float chi1);

        void updateControlSignals();

        void updateCurrentReference(float chi1_d, float chi1_dot_d);
};