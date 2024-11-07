/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_asmc.hpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a single DOF ASMC for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/

#include "vtec_sdc1_asmc.hpp"

VTEC_SDC1_1DOF_ASMC::VTEC_SDC1_1DOF_ASMC(float sample_time, const ASMC_Config& config, float lambda, float u_max, uint8_t D_max) :
                    VTecSDC1DynamicModel(sample_time, D_max), ASMCLin(sample_time, config, lambda, u_max)
{}

VTEC_SDC1_1DOF_ASMC::~VTEC_SDC1_1DOF_ASMC(){}

void VTEC_SDC1_1DOF_ASMC::updateNonLinearFunctions()
{
    ASMCLin::f_x_ = VTecSDC1DynamicModel::f_(0);
    ASMCLin::g_x_ = VTecSDC1DynamicModel::g_(0);
}

void VTEC_SDC1_1DOF_ASMC::calculateControlSignals()
{
    float chi1 = nu_(0);

    calculateManipulations(chi1);

    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(ASMCLin::u_ < 0)
        ASMCLin::u_ = 0;
}

void VTEC_SDC1_1DOF_ASMC::calculateControlSignals(float chi1)
{
    calculateManipulations(chi1);

    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(ASMCLin::u_ < 0)
        ASMCLin::u_ = 0;
}

void VTEC_SDC1_1DOF_ASMC::updateControlSignals()
{
    VTecSDC1DynamicModel::u_(0) = ASMCLin::u_;
}

void VTEC_SDC1_1DOF_ASMC::updateCurrentReference(float chi1_d, float chi1_dot_d)
{
    updateReferences(chi1_d, chi1_dot_d);
}