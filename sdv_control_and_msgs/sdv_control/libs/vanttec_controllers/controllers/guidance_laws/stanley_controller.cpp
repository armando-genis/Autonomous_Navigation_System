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

StanleyController::StanleyController(const std::vector<float>& delta_sat, float k, float k_soft)
{
    DELTA_SAT_ = delta_sat;
    psi_ = 0;
    k_ = k;
    k_soft_ = k_soft;
    delta_last_ = 0.0;

}

StanleyController::~StanleyController(){}

// void StanleyController::calculateCrosstrackError(float x, float y, float p1.x, float p1.y, float p2.x, float p2.y){
void StanleyController::calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2){
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

void StanleyController::setYawAngle(float psi){
    psi_ = psi;
}

void StanleyController::calculateSteering(float vel, uint8_t precision){
    vel_ = vel;

    // PI error fixed due to rounding in ak_ angle when the path is vertical that makes it greater than M_PI
    double PI = M_PI + 1e-3;
    if(ak_ >= PI/2 && ak_ <=  PI && psi_ <= -PI/2 && psi_ >= - PI){
        psi_ = psi_ + PI*2;
    } else if (ak_ < -PI/2 && ak_ > - PI && psi_ > PI/2 && psi_ <  PI){
        psi_ = psi_ - PI*2;
    }

    float phi = psi_ - ak_;
    delta_ = phi + 2.0*std::atan2(k_*ey_,k_soft_ + vel_) ;

    // You want to reduce psi by delta so ...
    // delta_ = -delta_; 
    // not anymore because of lidar or something, its inverted huh

    
    // NEW CORRECTIONS (TO TEST)
    float min_turn = 0.05;
    float max_turn = 0.3;
    float diff_abs = std::fabs(delta_ - delta_last_);
    if(diff_abs >= max_turn){
        if(delta_ - delta_last_ < -max_turn)
            delta_ = delta_last_ - max_turn;
        else if(delta_ - delta_last_ > max_turn)
            delta_ = delta_last_ + max_turn;
        std::cout << delta_ << std::endl;
    }

    if(diff_abs < min_turn && delta_ != delta_last_){
        delta_ = delta_last_;
    }

    
    // To round the float to the nearest tenth (0.1) set precision=10
    // To round the float to the nearest tenth/2 (0.05) set precision=20
    // To round the float to the nearest hundredth (0.01) set precision=100
    // This is intended to help with vibration reduction in steering mechanism
    // delta_ = std::round(delta_ * static_cast<float>(precision)) / static_cast<float>(precision);

    if (delta_ > DELTA_SAT_[0])
        delta_ = DELTA_SAT_[0];
    else if (delta_ < DELTA_SAT_[1])
        delta_ = DELTA_SAT_[1];

    delta_last_ = delta_;

    // std::cout << "atan2 = " << std::atan2(k_*ey_,k_soft_ + vel_) << std::endl;
    // std::cout << "Psi = " << psi_ << std::endl;
    // std::cout << "Ak = " << ak_ << std::endl;
    // std::cout << "Delta max = " << DELTA_SAT_[0] << ", Delta min = " << DELTA_SAT_[1] << std::endl;
    // std::cout << "Delta = " << delta_    << std::endl;
}