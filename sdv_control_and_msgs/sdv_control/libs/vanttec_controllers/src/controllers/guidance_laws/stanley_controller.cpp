/** ----------------------------------------------------------------------------
 * @file: stanley_controller.cpp
 * @date: November 30, 2022
 * @date: August 18, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * @author: Max Pacheco
 *
 * @brief: Stanley Controller class
 * -----------------------------------------------------------------------------
 * */

#include <stdio.h>
#include <iostream>
#include <cmath>

#include "stanley_controller.hpp"

StanleyController::StanleyController(double delta_min, double delta_max, double k, double k_soft)
{
    delta_min_ = delta_min;
    delta_max_ = delta_max;
    psi_ = 0;
    k_ = k;
    k_soft_ = k_soft;
}

StanleyController::~StanleyController(){}

// void StanleyController::calculateCrosstrackError(double x, double y, double p1.x, double p1.y, double p2.x, double p2.y){
void StanleyController::calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2){
    double m1;
    double m2;
    double b;
    double c;
    double xp;
    double yp;

    double ex = p2.x - p1.x;
    double ey = p2.y - p1.y;

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

void StanleyController::setYawAngle(double psi){
    psi_ = psi;
}

void StanleyController::calculateSteering(double vel, uint8_t precision){
    vel_ = vel;

    // PI error fixed due to rounding in ak_ angle when the path is vertical that makes it greater than M_PI
    double PI = M_PI + 1e-3;
    if(ak_ >= PI/2 && ak_ <=  PI && psi_ <= -PI/2 && psi_ >= - PI){
        psi_ = psi_ + PI*2;
    } else if (ak_ < -PI/2 && ak_ > - PI && psi_ > PI/2 && psi_ <  PI){
        psi_ = psi_ - PI*2;
    }

    double phi = psi_ - ak_;
    // std::cout << "phi: " << phi << std::endl;
    delta_ = phi + std::atan2(k_*ey_,k_soft_ + vel_);
    // std::cout << "atan: " << std::atan2(k_*ey_,k_soft_ + vel_) << std::endl;
    // std::cout << "delta_: " << delta_ << std::endl;
    delta_ = std::clamp(delta_, delta_min_, delta_max_);
}