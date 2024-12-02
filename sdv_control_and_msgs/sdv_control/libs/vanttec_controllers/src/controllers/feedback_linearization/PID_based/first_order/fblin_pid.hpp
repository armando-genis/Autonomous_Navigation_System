/** ----------------------------------------------------------------------------
 * @file: fblin_pid.hpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_PID_H__
#define __FBLIN_PID_H__

#include "controllers/control_laws/PID/first_order/pid.hpp"
#include "controllers/feedback_linearization/base/fb_lin_control.hpp"

class PIDLin : public FBLin {
public:
  PIDLin(double sample_time, double k_p, double k_i, double k_d, double u_max,
         const DOFControllerType_E &type);
  ~PIDLin();

  void calculateManipulations(double chi1);
  void updateReferences(double chi1_d, double chi1_dot_d);

private:
  PID control_law_;
};

#endif