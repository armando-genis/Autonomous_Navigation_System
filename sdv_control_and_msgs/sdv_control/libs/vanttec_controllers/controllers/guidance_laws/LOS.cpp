/** ----------------------------------------------------------------------------
 * @file: LOS.cpp
 * @date: November 30, 2022
 * @date: September 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Line-Of-Sight Controller class
 * -----------------------------------------------------------------------------
 * */

#include <stdio.h>
#include <iostream>
#include <cmath>

#include "LOS.hpp"

LOS::LOS(const std::vector<float>& delta_sat, float kappa, float KAPPA_MAX)
{
    DELTA_SAT_ = delta_sat;
    psi_ = 0;
    ey_ = 0;               // Alongtrack error
    ex_ = 0;               // Crosstrack error
    vel_ = 0;             // velocity vector norm
    ak_ = 0;              // path angle
    beta_ = 0;            // desired heading
    kappa_ = kappa;
    KAPPA_MAX_ = KAPPA_MAX;
}

LOS::~LOS(){}

void LOS::calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2){
    float m1;
    float m2;
    float b;
    float c;
    float xp;
    float yp;

    float ex = p2.x - p1.x;
    float ey = p2.y - p1.y;

    // Angle of path frame
    ak_ = std::atan2(ey,ex);

    if(std::isnormal(ex) && std::isnormal(ey)){
        // Slope of path
        m1 = ex/ey;
        b = p2.x - m1*p2.y;

        // Slope of normal line to the path
        m2 = -1/m1;
        c = vehicle_pos.x - m2*vehicle_pos.y;

        // Obtain intersection point
        yp = (c - b)/(m1 - m2);
        xp = m1*yp + b;

    } else {
        if(!std::isnormal(ex)){
            yp = vehicle_pos.y;
            xp = p2.x; // or x1
        }
        if(!std::isnormal(ey)){
            yp = p2.y; // or y1
            xp = vehicle_pos.x;
        }
    }

    // Crosstrack and along-track errors in path frame
    ex_ = (p2.x - xp)*std::cos(ak_) + (p2.y - yp)*std::sin(ak_);
    ey_ = -(vehicle_pos.x - xp)*std::sin(ak_) + (vehicle_pos.y - yp)*std::cos(ak_);

    // std::cout << "xp = " << xp << std::endl;
    // std::cout << "yp = " << yp << std::endl;
    // std::cout << "x = " << vehicle_pos.x << std::endl;
    // std::cout << "y = " << vehicle_pos.y << std::endl;
    // std::cout << "Along-track error = " << ex_ << std::endl;
    // std::cout << "Crosstrack error = " << ey_ << std::endl;
    // std::cout << "ak = " << ak_ << std::endl;
}

void LOS::setYawAngle(float psi){
    psi_ = psi;
}

void LOS::calculateSteering(float vel, float L, uint8_t precision){
    vel_ = vel;

    beta_ = ak_ + std::atan2(-ey_,kappa_);
    delta_ = psi_ - beta_;
    // beta_ = std::atan2(2*L*e_,kappa_*kappa_);
    // delta_ = beta_;

    // To round the float to the nearest tenth (0.1) set precision=10
    // To round the float to the nearest tenth/2 (0.05) set precision=20
    // To round the float to the nearest hundredth (0.01) set precision=100
    // This is intended to help with vibration reduction in steering mechanism
    delta_ = std::round(delta_ * static_cast<float>(precision)) / static_cast<float>(precision);
    
    if (delta_ > DELTA_SAT_[0])
        delta_ = DELTA_SAT_[0];
    else if (delta_ < DELTA_SAT_[1])
        delta_ = DELTA_SAT_[1];
    else {
        // To round the float to the nearest tenth (0.1) set precision=10
        // To round the float to the nearest tenth/2 (0.05) set precision=20
        // To round the float to the nearest hundredth (0.01) set precision=100
        // This is intended to help with vibration reduction in steering mechanism
        delta_ = std::round(delta_ * static_cast<float>(precision)) / static_cast<float>(precision);
    }

    // std::cout << "atan2 = " << std::atan2(k_*ey_,k_soft_ + vel_) << std::endl;
    // std::cout << "Psi = " << psi_ << std::endl;
    // std::cout << "Ak = " << ak_ << std::endl;
    // std::cout << "Delta max = " << DELTA_SAT_[0] << ", Delta min = " << DELTA_SAT_[1] << std::endl;
    // std::cout << "Delta = " << delta_    << std::endl;
}