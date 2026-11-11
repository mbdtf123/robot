/**
 * @file pid_set.cpp
 * @brief PID参数管理模块 - 高度自适应调参 + 串口实时调参
 */

#include "pid_set.h"      // 自己的头文件（PIDValues结构体、函数声明）
#include "com_uart.h"     // 串口通信模块（com_read_line, com_send_printf）
#include <string.h>       // 字符串处理函数（strchr）
#include <stdlib.h>       // 标准库函数（atof）

// ==================== 全局PID参数变量 ====================
float vel_kp = -0;        // 速度环Kp（负值表示方向）
float balance_kp = -0;    // 直立环Kp（负值表示方向）
int speed_limit = 5;      // 轮毂电机速度限制（力矩限幅）
float robot_kp = 0;       // 机器人整体Kp（位置环）

// ==================== 外部PID对象引用 ====================
// 这三个对象在 pid_control.cpp 中定义，这里通过extern引用
extern PIDController PIDvel;     // 速度环PID控制器
extern PIDController PIDstable;  // 直立环PID控制器（平衡）
extern PIDController PIDx;       // 位置环PID控制器（腿部X轴）

/**
 * @brief 根据机器人高度自适应调整PID参数
 * 
 * 使用线性插值，在三个预设高度节点（0mm, 75mm, 150mm）之间平滑过渡PID参数
 * 
 * @param y_height 当前机器人高度（mm）
 * @return PIDValues 插值后的PID参数（但实际直接修改了全局PID对象）
 */
PIDValues heightFit(int y_height) {
      // 三个高度节点的预设值
      float y0 = 0, y1 = 75, y2 = 150;
      
      // y=0mm 时的PID参数
      PIDValues pid0 = {-0.525, -0.17, 0.015, 3.0};
      // y=75mm 时的PID参数
      PIDValues pid1 = {-0.525, -0.155, 0.015, 7};
      // y=150mm 时的PID参数
      PIDValues pid2 = {-0.55, -0.16, 0.025, 9};
      PIDValues result;  // 返回值（实际未被完全填充，有bug）
    
      // ----- 高度 ≤ 75mm：在 pid0 和 pid1 之间插值 -----
      if (y_height <= y1) {
          speed_limit = 5;  // 低位时速度限制较宽松
          
          // 计算插值系数 t ∈ [0, 1]
          float t = (y_height - y0) / (y1 - y0);
          
          // 线性插值并更新全局PID参数
          PIDvel.P = -(pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp));
          PIDstable.P = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
          PIDstable.D = pid0.linear_PIDstable_kd + t * (pid1.linear_PIDstable_kd - pid0.linear_PIDstable_kd);
          PIDx.P = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp);
      } 
      // ----- 高度 > 75mm：在 pid1 和 pid2 之间插值 -----
      else {
          speed_limit = 3;  // 高位时速度限制更严格
          
          // 计算插值系数 t ∈ [0, 1]
          float t = (y_height - y1) / (y2 - y1);
          
          // 线性插值并更新全局PID参数
          PIDvel.P = -(pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp));
          PIDstable.P = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp);
          PIDstable.D = pid1.linear_PIDstable_kd + t * (pid2.linear_PIDstable_kd - pid1.linear_PIDstable_kd);
          robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
      }
      
      return result;  // BUG: result未被填充，返回的是垃圾值
}

/**
 * @brief 串口接收用户命令，实时调整PID参数
 * 
 * 命令格式：
 *   - "kp,balance_kp,kd\n"  三个参数：速度Kp，平衡Kp，平衡Kd
 *   - "kp,robot_kp\n"       两个参数：速度Kp，机器人Kp
 */
void uartReceiveUserCommand() {
    char buf[128];  // 接收缓冲区
    
    // 从串口读取一行（以换行符结束）
    int len = com_read_line(buf, sizeof(buf));
    if (len == 0) return;  // 没有数据则返回
    
    // 查找第一个逗号
    char* comma1 = strchr(buf, ',');
    if (comma1) {
        *comma1 = '\0';  // 将逗号替换为字符串结束符，分割出第一个参数
        
        // 解析第一个参数：速度环Kp
        float first = atof(buf);
        vel_kp = first;
        com_send_printf("vel_kp:%.3f\n", vel_kp);
        
        // 查找第二个逗号
        char* comma2 = strchr(comma1 + 1, ',');
        if (comma2) {
            // ----- 有三个参数 -----
            *comma2 = '\0';  // 分割出第二个参数
            
            // 解析第二个参数：直立环Kp
            balance_kp = atof(comma1 + 1);
            // 解析第三个参数：直立环Kd
            PIDstable.D = atof(comma2 + 1);
            
            com_send_printf("balance_kp:%.3f\n", balance_kp);
            com_send_printf("PIDstable.D:%.3f\n", PIDstable.D);
        } else {
            // ----- 只有两个参数 -----
            // 解析第二个参数：机器人Kp
            robot_kp = atof(comma1 + 1);
            com_send_printf("robot_kp:%.3f\n", robot_kp);
        }
    }
}

/**
 * @brief 数值限幅函数
 * 
 * @param value 输入值
 * @param minVal 最小值
 * @param maxVal 最大值
 * @return 限幅后的值
 */
float clampToRange(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;  // 小于下限返回下限
  if (value > maxVal) return maxVal;  // 大于上限返回上限
  return value;                        // 在范围内返回原值
}