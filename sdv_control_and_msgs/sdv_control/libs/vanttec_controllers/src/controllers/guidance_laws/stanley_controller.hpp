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
    double x;
    double y;
} Point;

class StanleyController
{
    public:
        double delta_min_;
        double delta_max_;
        double delta_;           // Desired steering
        double psi_;             // Current heading
        double k_;               // Controller gain
        double k_soft_;          // Soft gain
        double ex_;              // Along-track error
        double ey_;              // Crosstrack error
        double vel_;             // velocity vector norm
        double ak_;              // path angle

        StanleyController();
        StanleyController(double delta_min, double delta_max, double k, double k_soft);
        virtual ~StanleyController();

        void calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2);
        void setYawAngle(double psi);
        void calculateSteering(double vel, uint8_t precision);
};

#endif