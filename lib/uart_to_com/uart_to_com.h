#ifndef UART_TO_COM_H
#define UART_TO_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/uart.h" // 包含相关类型定义，如 uart_port_t

// ==================== 硬件引脚配置 ====================
// 定义UART1的TXD (发送) 和 RXD (接收) 引脚
// 此串口通常连接至电脑的USB转串口芯片
#define UART_TO_COM_UART1_TXD  (17)  
#define UART_TO_COM_UART1_RXD  (16)  

// 定义UART2的TXD (发送) 和 RXD (接收) 引脚
// 此串口通常连接至总线舵机或其他外设
#define UART_TO_COM_UART2_TXD  (11)  
#define UART_TO_COM_UART2_RXD  (12)  

// ==================== UART参数配置 ====================
#define UART_TO_COM_BAUD_RATE       (115200)          
#define UART_TO_COM_BUF_SIZE        (256)             

/**
 * @brief 启动UART透传桥接功能
 * 
 * 该函数会初始化UART1和UART2，并创建两个FreeRTOS任务，
 * 实现两个串口之间的双向透明数据传输。
 * 
 * @note 此函数会永久占用UART1(TX:17, RX:16) 和 UART2(TX:11, RX:12)。
 *       波特率为115200。
 */
void uart_to_com_start_bridge(void);

/**
 * @brief FreeRTOS任务：处理UART1到UART2的数据转发
 * 
 * 该任务会持续检查UART1的接收缓冲区，一旦发现数据，
 * 立即将其读取并写入UART2的发送缓冲区，实现U1->U2的数据透传。
 * 
 * 此函数可在库外被调用，以便用户创建自定义的任务。
 * 
 * @param pvParameters 任务参数，此任务未使用，传入NULL
 */
void task_process_uart1_to_uart2(void *pvParameters);

/**
 * @brief FreeRTOS任务：处理UART2到UART1的数据转发
 * 
 * 该任务会持续检查UART2的接收缓冲区，一旦发现数据，
 * 立即将其读取并写入UART1的发送缓冲区，实现U2->U1的数据透传。
 * 
 * 此函数可在库外被调用，以便用户创建自定义的任务。
 * 
 * @param pvParameters 任务参数，此任务未使用，传入NULL
 */
void task_process_uart2_to_uart1(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // UART_TO_COM_H