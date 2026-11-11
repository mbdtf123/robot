#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "bipedal_data.h"
#include "pid_set.h"
#include "pid.h"
#include "gatt_svc.h"
#include "Kms.h" 



// 全局变量声明
extern float roll_kp, roll_kd;
extern float motor1_vel, motor2_vel;
extern float roll, pitch, yaw;
extern float gyroX, gyroY, gyroZ;
extern float balance_offset;
extern int ZeparamremoteValue;
extern float wheelTorqueTarget1, wheelTorqueTarget2;
extern float remoteBalanceOffset;
extern int speed_limit;

// PID 对象声明
extern PIDController PIDvel;
extern PIDController PIDstable;
extern PIDController PIDx;

// 函数声明
void legControl(Node* left_target, Node* right_target);  
void bleParseCommand(const char *cmd, int len);  // ← 蓝牙命令解析
void wheel_control(void);

#endif