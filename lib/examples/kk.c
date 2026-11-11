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
 * @brief 运动学逆解函数
 * @param left_target 左腿目标坐标指针
 * @param right_target 右腿目标坐标指针
 * @param joint_angles 计算出的关节角度指针
 */
void inverse_kinematics(Node *left_target, Node *right_target, JointAngles *joint_angles)
{
    float alpha1, alpha2, beta1, beta2;
    
    // 1. 限幅处理 (复刻原逻辑)
    // 注意：原代码 limit_Y 是传值调用，修改的是副本。
    // 这里我们直接在计算前对指针指向的值进行限幅，或者使用临时变量。
    // 为了安全，我们使用临时变量存储限幅后的值用于计算，不破坏原数据（除非这是预期行为）
    float y_left = constrain_value(left_target->y, 110.0f, 180.0f);
    float y_right = constrain_value(right_target->y, 110.0f, 180.0f);

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

        // 角度归一化到 0 ~ 2PI
        alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);
        alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);

        // 选择逻辑
        if (alpha1 >= PI / 2.0f)
            joint_angles->alpha_right = alpha1;
        else
            joint_angles->alpha_right = alpha2;

        if (beta1 >= 0 && beta1 <= PI / 2.0f)
            joint_angles->beta_right = beta1;
        else
            joint_angles->beta_right = beta2;
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

        // 角度归一化
        alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);
        alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);

        if (alpha1 >= PI / 2.0f)
            joint_angles->alpha_left = alpha1;
        else
            joint_angles->alpha_left = alpha2;

        if (beta1 >= 0 && beta1 <= PI / 2.0f)
            joint_angles->beta_left = beta1;
        else
            joint_angles->beta_left = beta2;
    }
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