/** ----------------------------------------------------------------------------
 * @file: utils.cpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: common functions and constants used in the package.
 * -----------------------------------------------------------------------------
 * */

#include "utils.hpp"

namespace utils
{
    // vanttec_msgs::GuidanceWaypoints GenerateCircle(float radius, float x_center, float y_center, float z_center, float angle_offset)
    // {
    //     vanttec_msgs::GuidanceWaypoints waypoints;
        
    //     float angle = angle_offset;
    //     uint8_t counter = 0;
        
    //     while (angle <= (angle_offset + M_PI))
    //     {
    //         waypoints.waypoint_list_x.push_back(radius * std::cos(angle) + x_center);
    //         waypoints.waypoint_list_y.push_back(radius * std::sin(angle) + y_center);
    //         waypoints.waypoint_list_z.push_back(z_center);
    //         angle += M_PI / 6;
    //         counter++;
    //     }
        
    //     angle = angle_offset - M_PI;

    //     while (angle <= angle_offset)
    //     {
    //         waypoints.waypoint_list_x.push_back(radius * std::cos(angle) + x_center);
    //         waypoints.waypoint_list_y.push_back(radius * std::sin(angle) + y_center);
    //         waypoints.waypoint_list_z.push_back(z_center);
    //         angle += M_PI / 6;
    //         counter++;
    //     }
        
    //     waypoints.waypoint_list_length = counter;

    //     return waypoints;
    // }

    void calculateR_z(Eigen::Matrix3f& R, double psi)
    {
        R << std::cos(psi), std::sin(psi), 0,
            -std::sin(psi), std::cos(psi), 0,
            0,             0,             1;
    }

    void calculateR_zyx(Eigen::Matrix3f& R, double phi, double theta, double psi)
    {
        R <<    std::cos(psi)*std::cos(theta),      -std::sin(psi)*std::cos(phi) + std::cos(psi)*std::sin(theta)*std::sin(phi),     std::sin(psi)*std::sin(phi) + std::cos(psi)*std::cos(phi)*std::sin(theta),
                std::sin(psi)*std::cos(theta),       std::cos(psi)*std::cos(phi) + std::sin(phi)*std::sin(theta)*std::sin(psi),    -std::cos(psi)*std::sin(phi) + std::sin(theta)*std::sin(psi)*std::cos(phi),
               -std::sin(theta),                     std::cos(theta)*std::sin(phi),                                                 std::cos(theta)*std::cos(phi);
    }

    void calculate6DOFTransformation(Eigen::Matrix3f& R, Eigen::Matrix3f& T, Eigen::MatrixXf& J, const Eigen::VectorXf& eta)
    {
        float phi = eta(3);
        float theta = eta(4);
        float psi = eta(5);

        calculateR_zyx(R, phi, theta, psi);

        T <<   1,     std::sin(phi)*std::tan(theta),  std::cos(phi)*std::tan(theta),
                0,     std::cos(phi),                  -std::sin(phi),
                0,     std::sin(phi)/std::cos(theta),  std::cos(phi)/std::cos(theta);

        J << R,                            Eigen::Matrix3f::Zero(3, 3),
             Eigen::Matrix3f::Zero(3, 3),  T;
    }

    void calculate6DOFDifferentialTransform(Eigen::Matrix3f& R, Eigen::MatrixXf& J,  Eigen::MatrixXf& J_inv,
                                            Eigen::Matrix3f& R_dot, Eigen::Matrix3f& T_dot, Eigen::MatrixXf& J_dot,
                                            const Eigen::VectorXf& eta, const Eigen::VectorXf& eta_dot)
    {

        float phi = eta(3);
        float theta = eta(4);
        float psi = eta(5);

        float phi_dot = eta_dot(3);
        float theta_dot = eta_dot(4);
        float psi_dot = eta_dot(5);

        Eigen::Vector3f vect;

        vect << phi,
                theta,
                psi;

        R_dot = R*Skew(vect);
        T_dot << 0,         std::sin(phi)*utils::secant(theta)*utils::secant(theta)*theta_dot + std::cos(phi)*std::tan(theta)*phi_dot,            std::cos(phi)*utils::secant(theta)*utils::secant(theta)*theta_dot - std::sin(phi)*std::tan(theta)*phi_dot,
                 0,        -std::sin(phi)*phi_dot,                                                                                                  -std::cos(phi)*phi_dot,
                 0,        (std::cos(theta)*std::cos(phi)*phi_dot + std::sin(phi)*std::sin(theta)*theta_dot)/(std::cos(theta)*std::cos(theta)),    (-std::cos(theta)*std::sin(phi)*phi_dot + std::cos(phi)*std::sin(theta)*theta_dot)/(std::cos(theta)*std::cos(theta));

        J_dot << R_dot,                             Eigen::Matrix3f::Zero(3, 3),
                 Eigen::Matrix3f::Zero(3, 3),       T_dot;

        J_inv = J.inverse();
    }

    Eigen::Matrix3f Skew(const Eigen::Vector3f& vect)
    {
        Eigen::Matrix3f skew;
        skew << 0,       -vect(2),  vect(1),
                vect(2),  0,       -vect(0),
                -vect(1), vect(0), 0;
        
        return skew;
    }

    float secant(float angle)
    {
        return 1/std::cos(angle);
    }
    
    int8_t sign(float e)
    {
        int8_t sign = 0;

        if(std::isfinite(e) && !std::isnan(e))
            if (e != 0.0)
                sign = static_cast<int8_t>(e / std::abs(e));
        
        // check what to do if e is NaN or inf

        return sign;
    }

    float sig(float e, float a)
    {
        return static_cast<float>(sign(e))*pow(std::fabs(e),a);
    }
}