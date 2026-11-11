/**
 * @file pid_control.cpp
 * @brief 机器人PID控制主逻辑 - 轮足复合机器人控制系统
 * 
 * 功能说明：
 * 1. 接收蓝牙命令，解析控制指令（高度、速度、转向、PID参数）
 * 2. 轮子平衡控制：速度环 + 直立环串级PID
 * 3. 腿部轨迹控制：根据速度和高度计算足端目标位置
 */

#include "pid_control.h"
#include "com_uart.h"
#include <string.h>
#include <stdlib.h>

/* ==================== 全局变量定义 ==================== */

// 运动学参数（左右腿独立）
motionControlParams LeftMotionControlParams;   // 左腿运动学参数（L1/L2长度、目标坐标等）
motionControlParams RightMotionControlParams;  // 右腿运动学参数
motorsparam motorsParam;                       // 电机角度和零点校准值

// PID控制器对象
PIDController PIDvel = PIDController{0,0,0,0,5};     // 速度环PID
PIDController PIDstable = PIDController{0,0,0,0,5};  // 直立环PID（平衡）
PIDController PIDx = PIDController{0,0,0,0,5};       // 位置环PID（腿部X轴）

// 腿部高度控制变量
int ZeparamremoteValue = 0;      // 当前高度偏移量（0-150mm）
int lastZeparamremoteValue = 0;  // 上次高度偏移（用于增量平滑）

// 运动控制变量
float velTarget = 0;             // 目标速度（m/s）
float motor1_vel, motor2_vel;    // 左右轮当前速度
float steering = 0;              // 转向量（-10 ~ 10）
float remoteBalanceOffset = 0;   // 重心偏置

// 机器人姿态
float roll, pitch, yaw;          // 欧拉角（度）
float gyroX, gyroY, gyroZ;       // 角速度（度/秒）

// 腿部目标坐标
float leftY = 150, rightY = 150; // Y轴高度（mm），默认150mm
float leftX = 60, rightX = 60;   // X轴前倾（mm），默认60mm

// 轮子力矩输出
float wheelTorqueTarget1 = 0;    // 左轮力矩
float wheelTorqueTarget2 = 0;    // 右轮力矩

/* ==================== 工具函数 ==================== */

/**
 * @brief 数值限幅函数
 * @param val 输入值
 * @param minV 最小值
 * @param maxV 最大值
 * @return 限幅后的值
 */
static float constrainValue(float val, float minV, float maxV) {
    if (val < minV) return minV;
    if (val > maxV) return maxV;
    return val;
}

/* ==================== 蓝牙命令解析 ==================== */

/**
 * @brief 解析蓝牙串口命令，替代原遥控器PPM信号
 * 
 * 命令格式：
 * - "H:120"      设置高度 120mm（范围0-150）
 * - "V:1.5"      设置前进速度 1.5（正=前进，负=后退）
 * - "S:0.5"      设置转向量 0.5（正=右转，负=左转）
 * - "B:0.3"      设置重心偏置 0.3
 * - "PID:0.5,0.2,0.01"  设置PID参数（P,I,D）
 * - "PID:0.5,0.01"      设置PID参数（P,D）
 * 
 * @param cmd 接收到的命令字符串
 * @param len 命令长度
 */
void bleParseCommand(const char *cmd, int len) {
    // 复制命令到临时缓冲区
    char temp[64];
    int temp_len = (len < sizeof(temp)-1) ? len : sizeof(temp)-1;
    memcpy(temp, cmd, temp_len);
    temp[temp_len] = '\0';
    
    // 查找冒号分隔符
    char *colon = strchr(temp, ':');
    if (!colon) return;
    
    *colon = '\0';           // 截断，type指向命令类型
    char *type = temp;       // 命令类型（H/V/S/B/PID）
    char *value = colon + 1; // 参数值
    
    // ----- 高度控制 -----
    if (strcmp(type, "H") == 0) {
        int height = atoi(value);
        ZeparamremoteValue = constrainValue(height, 0, 150);
        com_send_printf("Height set to: %d\n", ZeparamremoteValue);
    }
    // ----- 速度控制 -----
    else if (strcmp(type, "V") == 0) {
        velTarget = atof(value);
        com_send_printf("Velocity set to: %.2f\n", velTarget);
    }
    // ----- 转向控制 -----
    else if (strcmp(type, "S") == 0) {
        steering = atof(value);
        steering = constrainValue(steering, -10, 10);
        com_send_printf("Steering set to: %.2f\n", steering);
    }
    // ----- 重心偏置 -----
    else if (strcmp(type, "B") == 0) {
        remoteBalanceOffset = atof(value);
        com_send_printf("Balance offset set to: %.2f\n", remoteBalanceOffset);
    }
    // ----- PID调参 -----
    else if (strcmp(type, "PID") == 0) {
        char *comma1 = strchr(value, ',');
        if (comma1) {
            *comma1 = '\0';
            float kp = atof(value);
            char *comma2 = strchr(comma1 + 1, ',');
            if (comma2) {
                // 三个参数：P, I, D
                *comma2 = '\0';
                float ki = atof(comma1 + 1);
                float kd = atof(comma2 + 1);
                PIDstable.P = kp;
                PIDstable.I = ki;
                PIDstable.D = kd;
                com_send_printf("PID set: P=%.3f I=%.3f D=%.3f\n", kp, ki, kd);
            } else {
                // 两个参数：P, D
                float kd = atof(comma1 + 1);
                PIDstable.P = kp;
                PIDstable.D = kd;
                com_send_printf("PID set: P=%.3f D=%.3f\n", kp, kd);
            }
        }
    }
    // ----- 未知命令 -----
    else {
        com_send_printf("Unknown command: %s\n", type);
    }
}

/* ==================== 轮子平衡控制 ==================== */

/**
 * @brief 轮子平衡控制（倒立摆模型）
 * 
 * 串级PID结构：
 *   外环（速度环）：目标速度 -> PIDvel -> 目标倾角 pitchTarget
 *   内环（直立环）：目标倾角 -> PD控制 -> 轮子力矩
 * 
 * 转向控制：
 *   左右轮力矩差动实现转向
 */
void wheel_control() {
    // ----- 速度环：计算目标倾角 -----
    // 当前速度 = 左右轮平均速度（取负是因为电机方向定义）
    float current_vel = -(motor1_vel + motor2_vel) / 2.0f;
    float pitchTarget = PIDvel(velTarget - current_vel);
    
    // ----- 直立环：PD控制计算基础力矩 -----
    // P项：角度误差  D项：角速度阻尼
    float base_torque = PIDstable.P * (pitchTarget - pitch) + PIDstable.D * gyroY;
    base_torque = clampToRange(base_torque, -5.0f, 5.0f);
    
    wheelTorqueTarget1 = base_torque;
    wheelTorqueTarget2 = base_torque;
    
    // ----- 转向差动补偿 -----
    // 左轮减速，右轮加速 = 向右转
    wheelTorqueTarget1 -= 0.65f * steering;
    wheelTorqueTarget2 += 0.65f * steering;
    
    // ----- 最终限幅 -----
    wheelTorqueTarget1 = clampToRange(wheelTorqueTarget1, -speed_limit, speed_limit);
    wheelTorqueTarget2 = clampToRange(wheelTorqueTarget2, -speed_limit, speed_limit);
}

/* ==================== 腿部轨迹控制 ==================== */

/**
 * @brief 腿部轨迹控制
 * 
 * 根据速度和高度指令，计算左右腿足端的目标坐标（X, Y）
 * - Y轴（高度）：基础高度 + 遥控/蓝牙调节量
 * - X轴（前倾）：基础前倾 ± 位置PID补偿（速度越快，前倾越大）
 * 
 * @param left_target  左腿目标坐标（输出）
 * @param right_target 右腿目标坐标（输出）
 */
void legControl(Node* left_target, Node* right_target) {
    // ----- 计算X轴偏移（速度前馈）-----
    // 速度越快，身体越前倾，用于抵消惯性
    float XOffset = -velTarget - (motor1_vel + motor2_vel) / 2.0f;
    
    // ----- Y轴（高度）计算 -----
    // 基础高度 + 蓝牙调节量
    left_target->y = leftY + ZeparamremoteValue;
    right_target->y = rightY + ZeparamremoteValue;
    
    // ----- X轴（前倾）计算 -----
    // 左腿：向后移（负方向）  右腿：向前移（正方向）
    // PIDx用于平滑过渡
    left_target->x = leftX - PIDx(XOffset);
    right_target->x = rightX + PIDx(XOffset);
    
    // ----- 安全限幅（防止逆解无解）-----
    left_target->y = constrainValue(left_target->y, 130.0f, 300.0f);
    right_target->y = constrainValue(right_target->y, 130.0f, 300.0f);
}