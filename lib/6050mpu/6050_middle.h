#ifndef MPU6050_MIDDLE_H
#define MPU6050_MIDDLE_H

#ifdef __cplusplus
extern "C" {
#endif

// 初始化MPU6050
void mpu6050_init(void);

// 获取姿态数据
void mpu6050_get_data(float* roll, float* pitch, float* yaw);

// 获取当前姿态数据（非阻塞）
void mpu6050_get_current_data(float* roll, float* pitch, float* yaw);

void mpu6050_get_data(float* roll, float* pitch, float* yaw);

void mpu6050_get_gyro(float* gx, float* gy, float* gz);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_MIDDLE_H