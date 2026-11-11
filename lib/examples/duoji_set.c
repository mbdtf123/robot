#include "com_uart.h"
#include "duoji_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

void servo_cmd_task(void *arg)
{
    char buf[128];
    com_send_printf("\r\n====== 舵机上电位置校准 ======\r\n");
    com_send_printf("11=解锁1    21=校准1零点并保存\r\n");
    com_send_printf("12=解锁2    22=校准2零点并保存\r\n");
    com_send_printf("13=解锁3    23=校准3零点并保存\r\n");
    com_send_printf("14=解锁4    24=校准4零点并保存\r\n");
    com_send_printf("m11=1顺     m21=1逆\r\n");
    com_send_printf("m12=2顺     m22=2逆\r\n");
    com_send_printf("m13=3顺     m23=3逆\r\n");
    com_send_printf("m14=4顺     m24=4逆\r\n");
    com_send_printf("==============================\r\n");
    com_send_printf("使用方法：\r\n");
    com_send_printf("1. 发送 11 解锁舵机1\r\n");
    com_send_printf("2. 手动把舵机转到期望的竖直位置\r\n");
    com_send_printf("3. 发送 21 校准零点并保存为上电位置\r\n");
    com_send_printf("==============================\r\n\r\n");

    while (1)
    {
        int len = com_read_line(buf, sizeof(buf));
        if (len <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        com_send_printf("指令: %s\r\n", buf);

        // ========== 1号舵机 ==========
        if (strcmp(buf, "11") == 0) { 
            duoji_unlock(1); 
            com_send_printf("✅ 1已解锁，请手动转到竖直位置\r\n"); 
        }
        else if (strcmp(buf, "21") == 0) { 
            duoji_set_midpoint(1);
            vTaskDelay(pdMS_TO_TICKS(100));
            duoji_set_boot_position(1);
            duoji_lock(1);
            com_send_printf("✅ 1已校准零点(1500)并保存为上电位置\r\n"); 
        }
        else if (strcmp(buf, "m11") == 0) { 
            duoji_set_mode_270_cw(1); 
            com_send_printf("✅ 1顺时针270\r\n"); 
        }
        else if (strcmp(buf, "m21") == 0) { 
            duoji_set_mode_270_ccw(1); 
            com_send_printf("✅ 1逆时针270\r\n"); 
        }

        // ========== 2号舵机 ==========
        else if (strcmp(buf, "12") == 0) { 
            duoji_unlock(2); 
            com_send_printf("✅ 2已解锁，请手动转到竖直位置\r\n"); 
        }
        else if (strcmp(buf, "22") == 0) { 
            duoji_set_midpoint(2);
            vTaskDelay(pdMS_TO_TICKS(100));
            duoji_set_boot_position(2);
            duoji_lock(2);
            com_send_printf("✅ 2已校准零点(1500)并保存为上电位置\r\n"); 
        }
        else if (strcmp(buf, "m12") == 0) { 
            duoji_set_mode_270_cw(2); 
            com_send_printf("✅ 2顺时针270\r\n"); 
        }
        else if (strcmp(buf, "m22") == 0) { 
            duoji_set_mode_270_ccw(2); 
            com_send_printf("✅ 2逆时针270\r\n"); 
        }

        // ========== 3号舵机 ==========
        else if (strcmp(buf, "13") == 0) { 
            duoji_unlock(3); 
            com_send_printf("✅ 3已解锁，请手动转到竖直位置\r\n"); 
        }
        else if (strcmp(buf, "23") == 0) { 
            duoji_set_midpoint(3);
            vTaskDelay(pdMS_TO_TICKS(100));
            duoji_set_boot_position(3);
            duoji_lock(3);
            com_send_printf("✅ 3已校准零点(1500)并保存为上电位置\r\n"); 
        }
        else if (strcmp(buf, "m13") == 0) { 
            duoji_set_mode_270_cw(3); 
            com_send_printf("✅ 3顺时针270\r\n"); 
        }
        else if (strcmp(buf, "m23") == 0) { 
            duoji_set_mode_270_ccw(3); 
            com_send_printf("✅ 3逆时针270\r\n"); 
        }

        // ========== 4号舵机 ==========
        else if (strcmp(buf, "14") == 0) { 
            duoji_unlock(4); 
            com_send_printf("✅ 4已解锁，请手动转到竖直位置\r\n"); 
        }
        else if (strcmp(buf, "24") == 0) { 
            duoji_set_midpoint(4);
            vTaskDelay(pdMS_TO_TICKS(100));
            duoji_set_boot_position(4);
            duoji_lock(4);
            com_send_printf("✅ 4已校准零点(1500)并保存为上电位置\r\n"); 
        }
        else if (strcmp(buf, "m14") == 0) { 
            duoji_set_mode_270_cw(4); 
            com_send_printf("✅ 4顺时针270\r\n"); 
        }
        else if (strcmp(buf, "m24") == 0) { 
            duoji_set_mode_270_ccw(4); 
            com_send_printf("✅ 4逆时针270\r\n"); 
        }

        else { 
            com_send_printf("❌ 未知指令\r\n"); 
        }
    }
}

void app_main(void)
{
    com_uart_init();
    duoji_init(115200);
    xTaskCreate(servo_cmd_task, "servo_task", 4096, NULL, 10, NULL);
}