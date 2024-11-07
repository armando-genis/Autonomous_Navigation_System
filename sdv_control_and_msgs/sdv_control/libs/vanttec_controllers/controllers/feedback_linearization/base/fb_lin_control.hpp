/** ----------------------------------------------------------------------------
 * @file: fb_lin_control.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */


#ifndef __FBLIN__
#define __FBLIN__

class FBLin
{
    public:
        float f_x_{0};
        float g_x_{0};

        float u_{0};
        float U_MAX_{0};
        float u_aux_{0};
        float u_n_{0};
        float chiX_dot_d_{0};

        // u_ = g_x_^(-1)*(chi1_dot_d - f_x_ + u_n - u_aux)
        
        FBLin(float u_max);
        ~FBLin();

        void updateControlSignal();
};

#endif