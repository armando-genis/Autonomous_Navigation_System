/** ----------------------------------------------------------------------------
 * @file: fb_lin_6dof_control.hpp
 * @date: April 25, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */


#ifndef __FBLIN6DOF__
#define __FBLIN6DOF__

#include <eigen3/Eigen/Dense>

class FBLin6DOF
{
    public:
        Eigen::VectorXf f_x_;
        Eigen::MatrixXf g_x_;

        Eigen::VectorXf u_;
        std::array<float,6> U_MAX_;
        Eigen::VectorXf u_aux_;
        Eigen::VectorXf u_n_;
        Eigen::VectorXf chiX_dot_d_;

        // u_ = g_x_^(-1)*(chi2_dot_d - f_x_ + u_n - u_aux)
        
        FBLin6DOF(const std::array<float,6>& u_max);
        ~FBLin6DOF();

        void updateControlSignal();
};

#endif