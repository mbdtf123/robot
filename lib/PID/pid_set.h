#ifndef PIDSETTING_H
#define PIDSETTING_H

#include "pid.h"

struct PIDValues {
  float linear_vel_kp;
  float linear_balance_kp;
  float linear_PIDstable_kd;
  float linear_robot_kp;
};

void uartReceiveUserCommand(void);  // ← 改 void，不用 String
PIDValues heightFit(int y_height);
float clampToRange(float value, float minVal, float maxVal);

#endif