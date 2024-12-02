/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_aitsmc.hpp
 * @date: September 12, 2023
 * @author: Andres Sanchez
 * 
 * @brief: Description of a single DOF AITSMC for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/


#include "vtec_sdc1_aitsmc.hpp"

VTEC_SDC1_1DOF_AITSMC::VTEC_SDC1_1DOF_AITSMC(float sample_time, const AITSMC_Params& config, float u_max, uint8_t D_max) :
                    VTecSDC1DynamicModel(sample_time, D_max), AITSMCLin(sample_time, config, u_max)
{}

VTEC_SDC1_1DOF_AITSMC::~VTEC_SDC1_1DOF_AITSMC(){}

void VTEC_SDC1_1DOF_AITSMC::updateNonLinearFunctions()
{
    AITSMCLin::f_x_ = VTecSDC1DynamicModel::f_(0);
    AITSMCLin::g_x_ = VTecSDC1DynamicModel::g_(0);
}

void VTEC_SDC1_1DOF_AITSMC::calculateControlSignals()
{
    float chi1 = nu_(0);

    calculateManipulations(chi1);
    
    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(AITSMCLin::u_ < 0)
        AITSMCLin::u_ = 0;
}

void VTEC_SDC1_1DOF_AITSMC::calculateControlSignals(float chi1)
{
    calculateManipulations(chi1);

    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(AITSMCLin::u_ < 0)
        AITSMCLin::u_ = 0;
}

void VTEC_SDC1_1DOF_AITSMC::updateControlSignals()
{
    VTecSDC1DynamicModel::u_(0) = AITSMCLin::u_;
}

void VTEC_SDC1_1DOF_AITSMC::updateCurrentReference(float chi1_d, float chi1_dot_d)
{
    updateReferences(chi1_d, chi1_dot_d);
}