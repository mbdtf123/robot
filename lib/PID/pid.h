#ifndef PID_H
#define PID_H

#include <stdint.h>  // ← 新增：提供 int64_t 类型定义

class PIDController
{
public:
    /**
     *  构造函数：初始化PID控制器参数
     *  
     * @param P     比例增益
     * @param I     积分增益
     * @param D     微分增益
     * @param ramp  输出变化率上限
     * @param limit 输出绝对值上限
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    // 重载()运算符：输入误差，返回PID控制量
    float operator() (float error);

    // PID参数（可运行时修改）
    float P;           //!< 比例增益
    float I;           //!< 积分增益
    float D;           //!< 微分增益
    float output_ramp; //!< 输出变化率限制
    float limit;       //!< 输出限幅值

protected:
    float error_prev;           //!< 上一次的误差值
    float output_prev;          //!< 上一次的PID输出值
    float integral_prev;        //!< 上一次的积分分量
    int64_t timestamp_prev;     //!< 上一次调用的时间戳（微秒）← 改成 int64_t
};

#endif // PID_H