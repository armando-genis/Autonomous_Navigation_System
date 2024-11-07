/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_pid.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a single DOF PID Controller for the VTec SDC1 dynamic model
 * -----------------------------------------------------------------------------
 **/

#include "vtec_sdc1_pid.hpp"

VTEC_SDC1_1DOF_PID::VTEC_SDC1_1DOF_PID( float sample_time, float k_p, float k_i, float k_d,
                                        float u_max, const DOFControllerType_E& type, uint8_t D_max) :
                    VTecSDC1DynamicModel(sample_time, D_max), PIDLin(sample_time, k_p, k_i, k_d, u_max, type)
{}

VTEC_SDC1_1DOF_PID::~VTEC_SDC1_1DOF_PID(){}

void VTEC_SDC1_1DOF_PID::updateNonLinearFunctions()
{
    PIDLin::f_x_ = VTecSDC1DynamicModel::f_(0);
    PIDLin::g_x_ = VTecSDC1DynamicModel::g_(0);
}

void VTEC_SDC1_1DOF_PID::calculateControlSignals()
{
    calculateManipulations(nu_(0));

    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(PIDLin::u_ < 0)
        PIDLin::u_ = 0;
}

void VTEC_SDC1_1DOF_PID::calculateControlSignals(float chi1)
{
    calculateManipulations(chi1);

    // Only in the case of the car, the next condition must be considered, as achieving reverse is not done by
    // computing negative control signals.
    // This must not be programed in any of the base controllers classes, as in the case of the boat and submarine,
    // reverse is straightforward
    if(PIDLin::u_ < 0)
        PIDLin::u_ = 0;
}

void VTEC_SDC1_1DOF_PID::updateControlSignals()
{
    VTecSDC1DynamicModel::u_(0) = PIDLin::u_;
}

void VTEC_SDC1_1DOF_PID::updateCurrentReference(float chi1_d, float chi1_dot_d)
{
    updateReferences(chi1_d, chi1_dot_d);
}