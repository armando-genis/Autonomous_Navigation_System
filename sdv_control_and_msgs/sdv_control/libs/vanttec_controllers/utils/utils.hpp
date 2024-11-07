/** ----------------------------------------------------------------------------
 * @file: utils.hpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: common functions and constants used in the package.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <cmath>
#include <eigen3/Eigen/Dense>

typedef enum DOFControllerType_E
{
    LINEAR_DOF = 0,
    ANGULAR_DOF = 1,
} DOFControllerType_E;

namespace utils
{
    /* Helper constants */
    static const float water_rho = 1000;
    static const float air_rho = 1.225;
    static const float g = 9.81;
    
    /* Helper functions */
    // vanttec_msgs::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset);
    void calculateR_z(Eigen::Matrix3f& R, double psi);
    void calculateR_zyx(Eigen::Matrix3f& R, double phi, double theta, double psi);
    void calculate6DOFTransformation(Eigen::Matrix3f& R, Eigen::Matrix3f& T, Eigen::MatrixXf& J, const Eigen::VectorXf& eta);
    void calculate6DOFDifferentialTransform(Eigen::Matrix3f& R, Eigen::MatrixXf& J,  Eigen::MatrixXf& J_inv,
                                            Eigen::Matrix3f& R_dot, Eigen::Matrix3f& T_dot, Eigen::MatrixXf& J_dot,
                                            const Eigen::VectorXf& eta, const Eigen::VectorXf& eta_dot);
    Eigen::Matrix3f Skew(const Eigen::Vector3f& vect);
    float secant(float angle);
    int8_t sign(float e);
    float sig(float e, float a);
}

#endif
