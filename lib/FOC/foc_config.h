#ifndef FOC_CONFIG_H_
#define FOC_CONFIG_H_

// ======== 核心电机参数 ========
#define VBUS                12.0f    // 母线电压 (V)
#define POLAR_PAIRS         7        // 极对数
#define CALI_TORQUE         1.0f

// ======== 硬件引脚定义,这里增加到两个电机。 ========
//0号电机及编码器
#define MOTOR_PHASE_A_GPIO_0    4
#define MOTOR_PHASE_B_GPIO_0    5
#define MOTOR_PHASE_C_GPIO_0    6

#define ENCODER_I2C_SDA_GPIO_0  7
#define ENCODER_I2C_SCL_GPIO_0  15

#define AS5600_I2C_ADDR_0      0x36

//1号电机及编码器
#define MOTOR_PHASE_A_GPIO_1    16
#define MOTOR_PHASE_B_GPIO_1    17
#define MOTOR_PHASE_C_GPIO_1    18
#define ENCODER_I2C_SDA_GPIO_1  8
#define ENCODER_I2C_SCL_GPIO_1  3
#define AS5600_I2C_ADDR_1      0x36

#define ANGLE_RESOLUTION     4096.0f

// ======== 控制频率 ========
#define USE_SVPWM           1        // 启用SVPWM (中点注入法, 比SPWM提高15.47%利用率)
#define PWM_FREQUENCY_HZ     20000
#define FOC_LOOP_FREQ       10000
#define PID_LOOP_FREQ       1000
#define SPEED_UP_FREQ       1000

// ======== 数学常数 ========
#define TWO_PI  6.283185307179586476925286766559
#define _PI     3.14159265359f
#define _PI_2   1.57079632679f
#define _PI_3   1.0471975512f
#define _SQRT3  1.73205080757f
#define _SQRT3_2 0.86602540378f

// ======== PWM参数 ========
#define PWM_DUTY_MAX        1023
#define MAX_DUTY_RATIO      0.95f    // 最大占空比
#define MIN_DUTY_RATIO      0.05f    // 最小占空比 (防止直通)

// ======== PID参数 - 速度环 (电压制式, 输出单位: V) ========
// 2026-11-08 标定后调整: 原 KP=0.2 太弱(目标5rad/s时只出 1V), KI=0.1 积分冻结
#define SPEED_KP    0.8f
#define SPEED_KI    0.5f
#define SPEED_KD    0.001f
#define SPEED_RAMP  5.0f
#define SPEED_LIMIT         4.0f     // 速度环最大输出电压

// ======== PID参数 - 位置环 (电压制式, 输出单位: V) ========
#define ANGLE_KP    1.0f
#define ANGLE_KI    0.1f
#define ANGLE_KD    0.0f
#define ANGLE_RAMP  100.0f
#define ANGLE_LIMIT         10.0f    // 位置环最大输出电压
#define TORQUE_MAX_V        8.0f     // 力矩模式最大电压

// ======== 积分分离带阈值 (Anti-windup) ========
#define SPEED_INT_BAND      0.0f     // 0 = 不启用积分分离(始终积分). 原来 5.0 会导致
                                     // 目标速度 5rad/s 时 |error|=5 不满足 <5, 积分被永久冻结
#define ANGLE_INT_BAND      0.5f     // 位置环: 误差超过此值时暂停积分 (rad)

// ======== 前馈增益 (Feedforward) ========
#define SPEED_FF_GAIN       0.0f     // 速度环前馈 (暂不使用)
#define ANGLE_FF_GAIN       0.01f    // 位置环速度阻尼前馈系数 (先关闭, 调通PID后再开)

// ======== 电角度校准偏移 (rad) ========
// 编码器零位与电机q轴的夹角, 需要根据实际机械安装调整
// 常见值: 0, π/2(1.57), -π/2(-1.57), π(3.14)
// 如果电机不转或抖动, 尝试调整此值
// 2026-11-08 开环标定实测: th0=5.3873rad, PP=7.002 -> OFFSET = π/2 - PP*th0 = 1.5588
#define ELEC_ANGLE_OFFSET   1.55881f

#endif // FOC_CONFIG_H_
