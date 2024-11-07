/** ----------------------------------------------------------------------------
 * @file: stanley_controller.hpp
 * @date: November 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * @author: Max Pacheco
 * 
 * @brief: Stanley Controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __STANLEY_CONTROLLER__
#define __STANLEY_CONTROLLER__

#include <vector>

typedef struct
{
    float x;
    float y;
} Point;

class StanleyController
{
    public:
        std::vector<float> DELTA_SAT_;       // {max, min} steering
        float delta_;           // Desired steering
        float delta_last_;           // Last steering
        float delta_last_2_;           // Last steering
        float delta_last_3_;           // Last steering
        float psi_;             // Current heading
        float k_;               // Controller gain
        float k_soft_;          // Soft gain
        float ex_;              // Along-track error
        float ey_;              // Crosstrack error
        float vel_;             // velocity vector norm
        float ak_;              // path angle

        StanleyController(const std::vector<float>& delta_sat, float k, float k_soft);
        virtual ~StanleyController();

        // void calculateCrosstrackError(float x, float y, float x0, float y0, float x1, float y1);
        void calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2);
        void setYawAngle(float psi);
        void calculateSteering(float vel, uint8_t precision);
};

#endif