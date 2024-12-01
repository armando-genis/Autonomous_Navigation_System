/** ----------------------------------------------------------------------------
 * @file: vtec_u4_in_6dof_dynamic_model.hpp
 * @date: August 31, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model in the inertial frame
           with Euler Angles.
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.hpp"

VTecU4InDynamicModel::VTecU4InDynamicModel(float sample_time) : Marine6DOFInDynamicModel(sample_time)
{
    /* Physical Parameters */

    m_        = 24;
    W_        = 235.44;
    volume_   = 0.0252;
    B_        = 247.212;
    Ixx_      = 0.900121387;
    Ixy_      = 0;
    Ixz_      = 0;
    Iyx_      = 0;
    Iyy_      = 1.754494427;
    Iyz_      = 0;
    Izx_      = 0;
    Izy_      = 0;
    Izz_      = 1.43389;

    /* Added Mass Parameters */

    X_u_dot_  = 16.8374;
    Y_v_dot_  = 20.2748;
    Z_w_dot_  = 35.3180;
    K_p_dot_  = 0.2165;
    M_q_dot_  = 0.6869;
    N_r_dot_  = 0.6157;

    /* Damping Parameters */

    X_u_      = -0.3431;
    Y_v_      = 0.0518;
    Z_w_      = -0.5841;
    K_p_      = 0.0064;
    M_q_      = 0.04;
    N_r_      = -0.1063;
    X_uu_     = -111.7397;
    Y_vv_     = -44.4058;
    Z_ww_     = -157.1951;
    K_pp_     = -0.4634;
    M_qq_     = -0.2902;
    N_rr_     = -2.2897;

    /* Distance from origin to center of buoyancy  */

    rb_x_ = 0;
    rb_y_ = 0;
    rb_z_ = -0.10726;

    /* Thruster configuration parameters */

    beta_            = 0.349066;
    var_epsilon_     = 0.261799;
    alpha_           = 1.325054;
    delta_           = 0.43301619;
    gamma_           = 1.8326;
    rh_x_            = 0.1867;
    rh_y_            = 0.2347;
    rh_z_            = 0.0175;
    rv_x_            = 0;
    rv_y_            = 0.2384;
    rv_z_            = 0;

    /* Maximum forces and torques per degree of freedom */
    MAX_FORCE_X_ = 127;
    MAX_FORCE_Y_ = 34;
    MAX_FORCE_Z_ = 118;
    MAX_TORQUE_K_ = 28; 
    MAX_TORQUE_M_ = 9.6;
    MAX_TORQUE_N_ = 36.6;
}

VTecU4InDynamicModel::~VTecU4InDynamicModel(){}

void VTecU4InDynamicModel::updateThrust()
{
    float c_delta = std::fabs(std::cos(delta_));
    float c_alpha = std::fabs(std::cos(alpha_));
    float c_gamma = std::fabs(std::cos(gamma_));
    
    L_ <<  c_delta,   c_delta,    -c_delta,   -c_delta,    0,     0,
         -c_alpha,   c_alpha,    -c_alpha,    c_alpha,    0,     0,
         -c_gamma,  -c_gamma,     c_gamma,    c_gamma,   -1,    -1;
         -(rh_y_*c_gamma + rh_z_*c_alpha),    rh_y_*c_gamma + rh_z_*c_alpha,    rh_y_*c_gamma - rh_z_*c_alpha,    -(rh_y_*c_gamma - rh_z_*c_alpha), -rv_y_, rv_y_,
         rh_x_*c_gamma - rh_z_*c_delta,       rh_x_*c_gamma - rh_z_*c_delta,    rh_x_*c_gamma + rh_z_*c_delta,     rh_x_*c_gamma + rh_z_*c_delta,    0,   0,
         -(rh_x_*c_alpha + rh_y_*c_delta),    rh_x_*c_alpha + rh_y_*c_delta,    rh_x_*c_alpha + rh_y_*c_delta,    -(rh_x_*c_alpha + rh_y_*c_delta),   0,   0;

    Thrust_ << (L_.completeOrthogonalDecomposition().pseudoInverse())*u_;
}