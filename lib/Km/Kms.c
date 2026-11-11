#include "Kms.h" //    里面定义了 Node, JointAngles 结构体和 L1-L5 常量
#include <math.h>       // 引入 C 标准数学库，ESP-IDF 内置支持
#include <stdint.h>     // 引入标准整数类型

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 数值限幅函数
 * 确保数值在指定范围内
 */
static float constrain_value(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

/**
 * @brief Y轴坐标限幅
 * 防止Y轴坐标过小或过大导致逆解无解或奇异点
 */
static void limit_y(float *y1, float *y2)
{
    // 注意：这里传入指针以修改原值，或者你可以选择只返回限制后的值
    // 这里的逻辑参考原代码，直接对传入的变量进行限制
    // 由于原函数是传值调用，这里为了保持逻辑一致，我们假设外部调用时已经处理了临时变量
    // 但在 C 语言中，为了修改外部变量，通常传指针。
    // 下面的实现是直接计算并返回限制值，更符合原代码的传值逻辑，但在函数内部无法直接修改外部变量。
    // 为了完全复刻原代码逻辑，我们在逆解函数内部直接调用 constrain_value。
}

/**
 * @brief 角度折到 (-PI, PI]
 *        注意：不能用 "if(a<0) a+=2PI" 的做法，那漏掉了 a 已接近 +2PI 的情况
 *        （本机构在 leg_x≈53mm 时 beta 由 -359.55deg 变成 +359.02deg）。
 */
static float wrap_pi(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

/**
 * @brief 并联五连杆的 beta 选支
 *
 * 闭链机构两个曲柄必须落在**同一装配构型**，不能各自独立选支。
 * 实测该机构两支分别约为 190deg 和 ~0deg，因此取落在 [-PI/2, PI/2]
 * 内的一支即可唯一确定装配构型。
 *
 * @return true = 找到合法解
 */
static bool select_beta(float b1, float b2, float *out)
{
    bool ok1 = (b1 >= -PI / 2.0f && b1 <= PI / 2.0f);
    bool ok2 = (b2 >= -PI / 2.0f && b2 <= PI / 2.0f);

    if (ok1 && ok2) {
        *out = (fabsf(b1) <= fabsf(b2)) ? b1 : b2;   // 都在范围内取更小倾角
        return true;
    }
    if (ok1) { *out = b1; return true; }
    if (ok2) { *out = b2; return true; }

    *out = 0.0f;
    return false;
}

/**
 * @brief 运动学逆解函数（并联五连杆）
 * @param left_target 左腿目标坐标指针
 * @param right_target 右腿目标坐标指针
 * @param joint_angles 计算出的关节角度指针
 * @return true = 左右腿均解出；false = 至少一侧无解（输出已清零）
 */
bool inverse_kinematics(Node *left_target, Node *right_target, JointAngles *joint_angles)
{
    float alpha1, alpha2, beta1, beta2;
    bool ok_right = false;
    bool ok_left  = false;

    // [FIX] 输出先清零：原版本无解时完全不写结构体，调用方会用到栈上垃圾值
    joint_angles->alpha_left  = 0.0f;
    joint_angles->beta_left   = 0.0f;
    joint_angles->alpha_right = 0.0f;
    joint_angles->beta_right  = 0.0f;
    
    // 1. 限幅处理 (复刻原逻辑)
    // 注意：原代码 limit_Y 是传值调用，修改的是副本。
    // 这里我们直接在计算前对指针指向的值进行限幅，或者使用临时变量。
    // 为了安全，我们使用临时变量存储限幅后的值用于计算，不破坏原数据（除非这是预期行为）
    float y_left = constrain_value(left_target->y, 70.0f, 180.0f);
    float y_right = constrain_value(right_target->y, 70.0f, 180.0f);

    // --- 右腿逆解运算 ---
    float a_right = 2.0f * right_target->x * L1;
    float b_right = 2.0f * y_right * L1; // 使用限幅后的 y
    float c_right = right_target->x * right_target->x + y_right * y_right + L1 * L1 - L2 * L2;
    
    float d_right = 2.0f * L4 * (right_target->x - L5);
    float e_right = 2.0f * L4 * y_right;
    float f_right = ((right_target->x - L5) * (right_target->x - L5) + L4 * L4 + y_right * y_right - L3 * L3);

    // 计算平方根项，防止负数导致 NaN
    float sqrt_val_alpha_r = (a_right * a_right) + (b_right * b_right) - (c_right * c_right);
    float sqrt_val_beta_r = (d_right * d_right) + e_right * e_right - (f_right * f_right);

    if (sqrt_val_alpha_r >= 0 && sqrt_val_beta_r >= 0) {
        sqrt_val_alpha_r = sqrtf(sqrt_val_alpha_r);
        sqrt_val_beta_r = sqrtf(sqrt_val_beta_r);

        alpha1 = 2.0f * atan2f((b_right + sqrt_val_alpha_r), (a_right + c_right));
        alpha2 = 2.0f * atan2f((b_right - sqrt_val_alpha_r), (a_right + c_right));
        
        beta1 = 2.0f * atan2f((e_right + sqrt_val_beta_r), (d_right + f_right));
        beta2 = 2.0f * atan2f((e_right - sqrt_val_beta_r), (d_right + f_right));

        // alpha 归一化到 [0, 2PI)（大腿舵机映射 s0 = alpha-90 需要正值区间）
        alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);
        alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);

        // [FIX] beta 折到 (-PI, PI]，不能归一化到 [0,2PI)：
        //       五连杆正常工作时 beta 会穿过 0，原写法把 -0.98deg 变成 359.02deg，
        //       再被 main.cpp 的 beta_deg∈[-20,80] 限幅截成 80deg，
        //       小腿舵机脉冲瞬间跳变 ~589us（≈甩 80deg），是机械冲击源。
        beta1 = wrap_pi(beta1);
        beta2 = wrap_pi(beta2);

        // 选择逻辑
        if (alpha1 >= PI / 2.0f)
            joint_angles->alpha_right = alpha1;
        else
            joint_angles->alpha_right = alpha2;

        // [FIX] 两曲柄必须同装配构型，取 [-PI/2, PI/2] 内的解
        ok_right = select_beta(beta1, beta2, &joint_angles->beta_right);
    }

    // --- 左腿逆解运算 ---
    float a_left = 2.0f * left_target->x * L1;
    float b_left = 2.0f * y_left * L1;
    float c_left = left_target->x * left_target->x + y_left * y_left + L1 * L1 - L2 * L2;

    float d_left = 2.0f * L4 * (left_target->x - L5);
    float e_left = 2.0f * L4 * y_left;
    float f_left = ((left_target->x - L5) * (left_target->x - L5) + L4 * L4 + y_left * y_left - L3 * L3);

    float sqrt_val_alpha_l = (a_left * a_left) + (b_left * b_left) - (c_left * c_left);
    float sqrt_val_beta_l = (d_left * d_left) + e_left * e_left - (f_left * f_left);

    if (sqrt_val_alpha_l >= 0 && sqrt_val_beta_l >= 0) {
        sqrt_val_alpha_l = sqrtf(sqrt_val_alpha_l);
        sqrt_val_beta_l = sqrtf(sqrt_val_beta_l);

        alpha1 = 2.0f * atan2f((b_left + sqrt_val_alpha_l), (a_left + c_left));
        alpha2 = 2.0f * atan2f((b_left - sqrt_val_alpha_l), (a_left + c_left));
        
        beta1 = 2.0f * atan2f((e_left + sqrt_val_beta_l), (d_left + f_left));
        beta2 = 2.0f * atan2f((e_left - sqrt_val_beta_l), (d_left + f_left));

        // alpha 归一化到 [0, 2PI)
        alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);
        alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);

        // [FIX] beta 折到 (-PI, PI]，保留负角（见右腿处说明）
        beta1 = wrap_pi(beta1);
        beta2 = wrap_pi(beta2);

        if (alpha1 >= PI / 2.0f)
            joint_angles->alpha_left = alpha1;
        else
            joint_angles->alpha_left = alpha2;

        // [FIX] 取 [-PI/2, PI/2] 内的解，保证两曲柄同装配构型
        ok_left = select_beta(beta1, beta2, &joint_angles->beta_left);
    }

    return ok_right && ok_left;
}

/**
 * @brief 运动学正解函数
 */
void forward_kinematics(Node *left_target, Node *right_target, JointAngles *joint_angles)
{
    float alpha, beta, theta1, theta2, selected_theta;
    Node A, C; // 不需要 B，原代码定义了但未用

    // --- 右腿正解 ---
    alpha = joint_angles->alpha_right;
    beta = joint_angles->beta_right;

    A.x = L1 * cosf(alpha);
    A.y = L1 * sinf(alpha);
    C.x = L5 + L4 * cosf(beta);
    C.y = L4 * sinf(beta);

    float a_right = 2.0f * (A.x - C.x) * L2;
    float b_right = 2.0f * (A.y - C.y) * L2;
    
    // 计算 A 与 C 的距离平方
    float dist_sq = (A.x - C.x) * (A.x - C.x) + (A.y - C.y) * (A.y - C.y);
    float c_right = (L3 * L3 - L2 * L2 - dist_sq);

    float sqrt_val_r = (b_right * b_right) + (a_right * a_right) - (c_right * c_right);

    if (sqrt_val_r >= 0) {
        sqrt_val_r = sqrtf(sqrt_val_r);
        theta1 = 2.0f * atan2f((b_right + sqrt_val_r), (a_right + c_right));
        theta2 = 2.0f * atan2f((b_right - sqrt_val_r), (a_right + c_right));

        // 归一化
        theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2.0f * PI);
        theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2.0f * PI);

        // 选择逻辑
        if (theta1 >= PI / 2.0f) {
            selected_theta = theta2;
        } else {
            selected_theta = theta1;
        }

        right_target->x = A.x + L2 * cosf(selected_theta);
        right_target->y = A.y + L2 * sinf(selected_theta);
    }

    // --- 左腿正解 ---
    alpha = joint_angles->alpha_left;
    beta = joint_angles->beta_left;

    A.x = L1 * cosf(alpha);
    A.y = L1 * sinf(alpha);
    C.x = L5 + L4 * cosf(beta);
    C.y = L4 * sinf(beta);

    float a_left = 2.0f * (A.x - C.x) * L2;
    float b_left = 2.0f * (A.y - C.y) * L2;
    
    float dist_sq_l = (A.x - C.x) * (A.x - C.x) + (A.y - C.y) * (A.y - C.y);
    float c_left = (L3 * L3 - L2 * L2 - dist_sq_l);

    float sqrt_val_l = (a_left * a_left) + (b_left * b_left) - (c_left * c_left);

    if (sqrt_val_l >= 0) {
        sqrt_val_l = sqrtf(sqrt_val_l);
        theta1 = 2.0f * atan2f((b_left + sqrt_val_l), (a_left + c_left));
        theta2 = 2.0f * atan2f((b_left - sqrt_val_l), (a_left + c_left));

        // 归一化
        theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2.0f * PI);
        theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2.0f * PI);

        if (theta1 >= PI / 2.0f) {
            selected_theta = theta2;
        } else {
            selected_theta = theta1;
        }

        left_target->x = A.x + L2 * cosf(selected_theta);
        left_target->y = A.y + L2 * sinf(selected_theta);
    }
}
