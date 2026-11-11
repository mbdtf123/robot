#include "6050_middle.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"  // esp_rom_delay_ns / esp_rom_delay_us

// ==================== 配置 ====================
#define I2C_SDA_PIN   9
#define I2C_SCL_PIN   10

// [FIX-11] 俯仰角硬偏置（原代码里写死 +0.6°）。
//   影响 1：让直立平衡点偏 -0.6°（机器人"歪着站"）。
//   影响 2：位置环启用后产生稳态位置误差 = PITCH_BIAS_DEG / robot_kp。
//           实测 robot_kp=2 时稳态偏 0.30 m，与 0.6/2 严格吻合。
//   若传感器确实存在安装倾角，应在标定环节消掉，不要在这里加常数。
//   确认安装无倾角时保持 0.0。
#define PITCH_BIAS_DEG   0.0f
#define MPU_ADDR      0x68

// MPU6050 寄存器
#define PWR_MGMT_1    0x6B
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43
#define WHO_AM_I      0x75

static const char *TAG = "MPU6050_SW";

// 滑动平均窗口大小（越大越平滑，但响应越慢）
#define SMA_WINDOW 5

// ========== Kalman 滤波类 ==========
class Kalman {
public:
    Kalman() { Q_angle=0.001f; Q_bias=0.003f; R_measure=3.0f; angle=bias=0; P[0][0]=P[0][1]=P[1][0]=P[1][1]=0; }
    float getAngle(float newAngle, float newRate, float dt) {
        float rate = newRate - bias;
        angle += dt * rate;
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        float S = P[0][0] + R_measure;
        if(S == 0) return angle;
        float K_0 = P[0][0]/S, K_1 = P[1][0]/S;
        float y = newAngle - angle;
        angle += K_0*y;
        bias += K_1*y;
        float p0=P[0][0],p1=P[0][1];
        P[0][0]-=K_0*p0; P[0][1]-=K_0*p1; P[1][0]-=K_1*p0; P[1][1]-=K_1*p1;
        return angle;
    }
private:
    float Q_angle,Q_bias,R_measure,angle,bias,P[2][2];
};

// ========== 简单滑动平均滤波器 ==========
class SimpleMovingAvg {
public:
    float update(float val) {
        buf[idx] = val;
        idx = (idx + 1) % SMA_WINDOW;
        if (count < SMA_WINDOW) count++;
        float sum = 0;
        for (int i = 0; i < count; i++) sum += buf[i];
        return sum / count;
    }
    void reset() { idx = count = 0; memset(buf, 0, sizeof(buf)); }
private:
    float buf[SMA_WINDOW] = {};
    int idx = 0, count = 0;
};

static Kalman kalX, kalY;
static SimpleMovingAvg smaX, smaY;  // 滑动平均

int16_t offX=0, offY=0, offZ=0;

// 当前姿态数据（全局变量，供外部访问）
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;

// 当前陀螺仪数据（全局变量，供外部访问）
static float current_gyro_x = 0.0f;
static float current_gyro_y = 0.0f;
static float current_gyro_z = 0.0f;

// ========== 软件 I2C 延时 ==========
// 用 esp_rom_delay_ns 纳秒级延时替代 NOP，更精准、不受编译器优化影响
// ESP32-S3 @ 240MHz: 约5us 对应 ~100kHz I2C 时钟
void i2c_delay() {
    esp_rom_delay_us(5);  // 5微秒，约100kHz
}

// 产生起始信号
void sw_i2c_start() {
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)I2C_SCL_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, 1);
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, 0); // SDA 从高变低，SCL 为高
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
}

// 产生停止信号
void sw_i2c_stop() {
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, 0);
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, 1); // SDA 从低变高，SCL 为高
    i2c_delay();
}

// 发送一个字节
bool sw_i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
        i2c_delay();
        // 设置 SDA 数据 (高位在前)
        gpio_set_level((gpio_num_t)I2C_SDA_PIN, (byte & 0x80) ? 1 : 0);
        byte <<= 1;
        i2c_delay();
        gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1); // 拉高 SCL
        i2c_delay();
    }
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
    
    // 读取 ACK (第9个脉冲)
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_INPUT);
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
    i2c_delay();
    bool ack = gpio_get_level((gpio_num_t)I2C_SDA_PIN) == 0; // 0 为 ACK
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    
    return ack; // 返回是否收到应答
}

// 读取一个字节 (参数 send_ack: 1=发送ACK(继续读), 0=发送NAK(最后字节))
uint8_t sw_i2c_read_byte(bool send_ack) {
    uint8_t byte = 0;
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_INPUT);
    
    for (int i = 0; i < 8; i++) {
        i2c_delay();
        gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
        i2c_delay();
        byte <<= 1;
        if (gpio_get_level((gpio_num_t)I2C_SDA_PIN)) byte |= 1;
        gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
    }
    
    // 发送 ACK/NAK
    gpio_set_direction((gpio_num_t)I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, send_ack ? 0 : 1); // 0=ACK, 1=NAK
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 0);
    
    return byte;
}

// ========== 封装读写API (替代原来的 wr/rd) ==========
// 写寄存器: Start -> Dev_Write -> Reg -> Data -> Stop
bool sw_mpu_write(uint8_t reg, uint8_t val) {
    sw_i2c_start();
    if (!sw_i2c_write_byte((MPU_ADDR << 1) | 0)) { // 写操作
        sw_i2c_stop();
        return false;
    }
    if (!sw_i2c_write_byte(reg)) {
        sw_i2c_stop();
        return false;
    }
    if (!sw_i2c_write_byte(val)) {
        sw_i2c_stop();
        return false;
    }
    sw_i2c_stop();
    return true;
}

// 读寄存器: Start -> Dev_Write -> Reg -> Start -> Dev_Read -> Data... -> Stop
bool sw_mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    // 第一步：发送要读的寄存器地址
    sw_i2c_start();
    if (!sw_i2c_write_byte((MPU_ADDR << 1) | 0)) { // 写操作
        sw_i2c_stop();
        return false;
    }
    if (!sw_i2c_write_byte(reg)) {
        sw_i2c_stop();
        return false;
    }
    
    // 第二步：重复起始信号，开始读
    sw_i2c_start();
    if (!sw_i2c_write_byte((MPU_ADDR << 1) | 1)) { // 读操作
        sw_i2c_stop();
        return false;
    }
    
    // 第三步：读取数据
    for (size_t i = 0; i < len; i++) {
        data[i] = sw_i2c_read_byte(i < len - 1); // 最后一个字节发 NAK
    }
    
    sw_i2c_stop();
    return true;
}

// ========== 初始化 + 校准 ==========
static void init_mpu() {
    // 初始化 GPIO 为开漏输出 (标准 I2C 模式)
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << I2C_SDA_PIN) | (1ULL << I2C_SCL_PIN);
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD; // 开漏
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;   // 必须上拉
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);

    // 初始化时钟为低
    gpio_set_level((gpio_num_t)I2C_SCL_PIN, 1);
    gpio_set_level((gpio_num_t)I2C_SDA_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // WHO_AM_I 检查
    uint8_t who = 0;
    if (sw_mpu_read_bytes(WHO_AM_I, &who, 1)) {
        // 删除打印
    } else {
        // 删除打印
    }

    // 配置 MPU
    sw_mpu_write(PWR_MGMT_1, 0);       // 唤醒
    vTaskDelay(10 / portTICK_PERIOD_MS);
    sw_mpu_write(CONFIG,     0x05);    // DLPF=5 → 10Hz带宽，大幅降低噪声
    sw_mpu_write(GYRO_CONFIG, 0);      // ±250°/s
    sw_mpu_write(ACCEL_CONFIG, 0);     // ±2g
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // 校准
    int32_t sx=0, sy=0, sz=0;
    for(int i=0; i<200; i++) {
        uint8_t d[6] = {0};
        if (sw_mpu_read_bytes(GYRO_XOUT_H, d, 6)) {
            sx += (int16_t)(d[0] << 8 | d[1]);
            sy += (int16_t)(d[2] << 8 | d[3]);
            sz += (int16_t)(d[4] << 8 | d[5]);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    offX = sx / 200; offY = sy / 200; offZ = sz / 200;
}

// ========== 主循环任务 ==========
static void mpu_task(void*) {
    uint8_t raw[14];
    float roll = 0, pitch = 0, yaw = 0;
    const float RAD2DEG = 57.29577951308232f;
    const float G_S = 131.f, A_S = 16384.f;
    uint64_t t0 = esp_timer_get_time();

    for(;;) {
        int64_t now = esp_timer_get_time();
        float dt = (now - t0) / 1e6f;
        t0 = now;
        if (dt <= 0 || dt > 0.1) dt = 0.002f;

        // 读取数据
        if (sw_mpu_read_bytes(ACCEL_XOUT_H, raw, 14)) {
            // 加速度处理
            int16_t ax = (int16_t)(raw[0] << 8 | raw[1]);
            int16_t ay = (int16_t)(raw[2] << 8 | raw[3]);
            int16_t az = (int16_t)(raw[4] << 8 | raw[5]);
            float ax_f = ax / A_S, ay_f = ay / A_S, az_f = az / A_S;

            // 陀螺仪处理 (去零偏)
            int16_t gx = (int16_t)(raw[8] << 8 | raw[9]) - offX;
            int16_t gy = (int16_t)(raw[10] << 8 | raw[11]) - offY;
            int16_t gz = (int16_t)(raw[12] << 8 | raw[13]) - offZ;
            
            float gx_d = gx / G_S, gy_d = gy / G_S, gz_d = gz / G_S;
            
            // 保存原始陀螺仪数据（带单位：度/秒）
            current_gyro_x = gx_d;
            current_gyro_y = gy_d;
            current_gyro_z = gz_d;

            // 核心算法：保留你指定的修正参数
            float acc_x = atan2(ay_f, az_f) * RAD2DEG;
            float acc_y = atan2(ax_f, az_f) * RAD2DEG;

            // Kalman 滤波
            float kx = kalX.getAngle(acc_x, gx_d, dt);
            float ky = kalY.getAngle(acc_y, gy_d, dt);

            // 滑动平均平滑输出（消除残余抖动）
            roll  = (smaX.update(kx) + 0.05f) * 1.003f;
            pitch = smaY.update(ky) + PITCH_BIAS_DEG;   // [FIX-11] 偏置改为可配置，默认 0
            yaw  += gz_d * dt;
            if (yaw > 180) yaw -= 360;
            if (yaw < -180) yaw += 360;
        }
        // 如果读取失败，roll/pitch/yaw 保持上一次的值 (防止乱跳)

        // 更新全局变量
        current_roll = roll;
        current_pitch = pitch;
        current_yaw = yaw;
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // ~100Hz
    }
}

// ==================== C接口函数实现 ====================

void mpu6050_init(void) {
    init_mpu();
    xTaskCreate(mpu_task, "mpu", 4096, NULL, 5, NULL);
}

void mpu6050_get_data(float* roll, float* pitch, float* yaw) {
    *roll = current_roll;
    *pitch = current_pitch;
    *yaw = current_yaw;
}

void mpu6050_get_current_data(float* roll, float* pitch, float* yaw) {
    *roll = current_roll;
    *pitch = current_pitch;
    *yaw = current_yaw;
}

void mpu6050_get_gyro(float* gx, float* gy, float* gz) {
    *gx = current_gyro_x;
    *gy = current_gyro_y;
    *gz = current_gyro_z;
}