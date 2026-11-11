#include "pid.h"
#include "esp_timer.h"   // 乐鑫高精度定时器
#include "esp_log.h"     // 可选：用于调试输出

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static const char *TAG = "PID";  // 日志标签

// 构造函数：创建PID对象时自动调用，用传入的参数初始化所有成员变量
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)
    , limit(limit)
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    // 乐鑫框架：使用 esp_timer_get_time() 获取微秒时间戳
    timestamp_prev = esp_timer_get_time();
}

// 重载函数调用运算符()：让PID对象可以像函数一样调用，输入误差值error，内部执行PID计算并返回控制输出量
float PIDController::operator()(float error)
{
    // 计算距离上次调用的时间间隔
    int64_t timestamp_now = esp_timer_get_time();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;  // 微秒转秒
    
    // 异常处理：防止除零和 micros 溢出
    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;  // 默认 1ms
    }

    // 比例项：P * e(k)
    float proportional = P * error;

    // 积分项：Tustin变换（梯形积分）
    // u_ik = u_ik_1 + I * Ts / 2 * (e_k + e_k_1)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    
    // 积分抗饱和：限制在 [-limit/3, limit/3]
    integral = _constrain(integral, -limit / 3.0f, limit / 3.0f);

    // 微分项：D * (e_k - e_k_1) / Ts
    float derivative = D * (error - error_prev) / Ts;

    // PID输出 = P项 + I项 + D项
    float output = proportional + integral + derivative;
    
    // 输出限幅
    output = _constrain(output, -limit, limit);

    // 输出斜坡限制（防止突变）
    if (output_ramp > 0.0f) {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    // 保存状态，供下次调用使用
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;

    return output;
}