#include <gtest/gtest.h>
#include "controllers/control_laws/PID/first_order/pid.hpp"

TEST(PID, SimpleModel){
  PIDParameters param;
  param.kP = 10;
  param.kI = 1;
  param.kD = 0.1;
  param.kDt = 0.01;
  PID pid(param);

  double position = 0;
  double velocity = 0;

  // Run a simple simulation, check that setpoint is reached with full PID impl.
  for(int i = 0; i < 1000; i++){
    double u = pid.update(position, 5);

    position += (velocity + u) * param.kDt;
    velocity += 0.1 * param.kDt;
  }

  // Position should be near setpoint.
  EXPECT_NEAR(position, 5, 0.1);

  pid = PID(param);
  position = 0;
  velocity = 0;
  // Run a simple simulation, check that setpoint is reached with full PID impl.
  for(int i = 0; i < 1000; i++){
    double u = pid.update(position, -5);

    position += (velocity + u) * param.kDt;
    velocity += 0.1 * param.kDt;
  }


  EXPECT_NEAR(position, -5, 0.1);
}

TEST(PID, ClampU) {
  PIDParameters param;
  param.kP = 1000;
  param.kUMax = 100;
  param.kUMin = -100;
  param.enable_ramp_rate_limit = false;

  PID pid(param);
  EXPECT_LE(pid.update(0, 1), 100);
  EXPECT_GE(pid.update(0, -1), -100);

  PIDParameters no_negative_param = param;
  no_negative_param.kUMin = 0;
  PID no_negative_pid(no_negative_param);
  EXPECT_GE(no_negative_pid.update(0, -1), 0);
}

TEST(PID, RampRateLimit){
  PIDParameters param;
  param.kP = 1;
  param.enable_ramp_rate_limit = true;
  param.ramp_rate = 1;
  param.kDt = 0.1;

  PID pid(param);

  // U should only be allowed to rise by 0.1
  EXPECT_NEAR(pid.update(0, 10), 0.1, 0.01);

  pid = PID(param);
  // U should rise to any value less than 0.1
  EXPECT_NEAR(pid.update(0, 0.05), 0.05, 0.01);
}