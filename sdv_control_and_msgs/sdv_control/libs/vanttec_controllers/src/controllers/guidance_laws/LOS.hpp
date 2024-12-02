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

#ifndef __LOS__
#define __LOS__

#include <vector>

typedef struct
{
    float x;
    float y;
} Point;

class LOS
{
    public:
        std::vector<float> DELTA_SAT_;       // {max, min} steering
        float delta_;           // Desired steering
        float psi_;             // Current heading
        float vel_;             // velocity vector norm
        float ak_;              // path angle
        float beta_;            // desired heading
        float kappa_;           // look ahead distance
        float KAPPA_MAX_;       // max look ahead distance
        float ex_;              // Along-track error
        float ey_;              // Crosstrack error

        LOS(const std::vector<float>& delta_sat, float kappa, float KAPPA_MAX);
        ~LOS();

        void calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2);
        void setYawAngle(float psi);
        void calculateSteering(float vel, float L, uint8_t precision);
};

#endif