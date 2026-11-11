/**
 * @file main.cpp
 * @brief 2804 云台电机 FOC 开环标定 + 分级自检
 *
 * APP_MODE 说明:
 *   2 = 开环标定（默认）: 对齐 + 扫 3 圈电角度，测出极对数/方向/电角度偏移
 *   1 = 分级自检        : 编码器 → 力矩锁轴 → 速度环
 *   0 = 只观测编码器    : 不动电机，最安全
 *   9 = 原总线舵机 ID 扫描
 *
 * 为什么标定要自己驱动 LEDC:
 *   库里的电角度 = angle_accum × POLAR_PAIRS + ELEC_ANGLE_OFFSET，
 *   POLAR_PAIRS / ELEC_ANGLE_OFFSET 都是编译期宏，运行时改不了；
 *   而且 foc_output() 是 static，外部拿不到。
 *   标定需要"任意指定电角度"，所以这里直接自己写 SVPWM（数学与库里完全一致）。
 *
 * 硬件前提:
 *   电机 A/B/C 相  → GPIO4 / GPIO5 / GPIO6
 *   编码器 SDA/SCL → GPIO7  / GPIO15
 *   驱动板 EN      → 3.3V 常使能（硬件，软件不管）
 *   驱动板 VM      → 12V 独立电源，必须共地
 */

// ================================================================
// 参数区 —— 改这里
// ================================================================
#define APP_MODE              1     // 5=AS5600诊断 2=开环标定 1=自检(默认) 0=只看编码器 9=舵机扫描

#define ENC_SDA_GPIO          7     // AS5600 SDA
#define ENC_SCL_GPIO          15    // AS5600 SCL

#define ENC_PROBE_SAMPLES     20
#define ENC_PROBE_INTERVAL_MS 100

// ---- 标定参数 ----
#define CAL_ALIGN_V           2.5f  // 对齐电压 (V)，云台电机 2~3V 合适
#define CAL_ALIGN_MS          1500  // 对齐等待时间
#define CAL_SWEEP_V           2.5f  // 扫描电压 (V)，转子跟不上就加到 4.0
#define CAL_STEPS_PER_EREV    12    // 每圈电角度分几步 (30°/步)
#define CAL_EREVS             3     // 扫几圈电角度
#define CAL_SETTLE_MS         400   // 每步等待转子稳定

// ---- 自检参数 (APP_MODE=1) ----
#define TORQUE_TEST_VALUE     0.10f  // 低速验证: 0.10 (0.8V)
#define TORQUE_TEST_SECONDS   1
#define SPEED_TEST_VALUE      2.0f   // 目标 2 rad/s (低速, 避开高转速下编码器失真)
#define SPEED_TEST_SECONDS    12

#define CTRL_STEP_US          1000
// ================================================================

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_hw_link.h"
#include "foc_config.h"

#if APP_MODE == 9
#include "duoji_control.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "CAL";

// ================================================================
// 通用小工具
// ================================================================
static inline float clampf_(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float enc_read_retry(void)
{
    for (int i = 0; i < 3; i++) {
        float a = enc_read_raw(0);
        if (a >= 0.0f) return a;
        esp_rom_delay_us(2000);
    }
    return -1.0f;
}

// ================================================================
// 独立的三相 PWM 驱动 (标定专用, 数学与 lib/FOC 的 foc_output 完全一致)
// ================================================================
static void cal_pwm_init(void)
{
    ledc_timer_config_t tc = {};
    tc.speed_mode      = LEDC_LOW_SPEED_MODE;
    tc.timer_num       = LEDC_TIMER_0;
    tc.duty_resolution = LEDC_TIMER_10_BIT;
    tc.freq_hz         = 20000;
    tc.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tc);

    const int pins[3] = {
        MOTOR_PHASE_A_GPIO_0,
        MOTOR_PHASE_B_GPIO_0,
        MOTOR_PHASE_C_GPIO_0
    };

    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t cc = {};
        cc.gpio_num   = (gpio_num_t)pins[i];
        cc.speed_mode = LEDC_LOW_SPEED_MODE;
        cc.channel    = (ledc_channel_t)i;
        cc.intr_type  = LEDC_INTR_DISABLE;
        cc.duty       = 512;         // 50% = 0V
        cc.hpoint     = 0;
        ledc_channel_config(&cc);
    }
    ESP_LOGI(TAG, "标定 PWM 就绪: A=%d B=%d C=%d, 20kHz/10bit",
             pins[0], pins[1], pins[2]);
}

/**
 * @brief 输出指定电角度方向的电压矢量 (SVPWM 中点注入)
 * @param elec_angle 电角度 (rad)
 * @param volts      矢量幅值 (V)
 */
static void cal_set_vector(float elec_angle, float volts)
{
    float uq = volts / (VBUS * 0.5f);      // 归一化到 [-1, 1] 基准

    float s = sinf(elec_angle);
    float c = cosf(elec_angle);

    // 逆 Park (Id=0) + 逆 Clarke —— 与 lib/FOC 完全相同的符号约定
    float ualpha = -uq * s;
    float ubeta  =  uq * c;
    float Ua =  ualpha;
    float Ub = -0.5f * ualpha + _SQRT3_2 * ubeta;
    float Uc = -0.5f * ualpha - _SQRT3_2 * ubeta;

    // SVPWM 中点注入
    float vmax = fmaxf(fmaxf(Ua, Ub), Uc);
    float vmin = fminf(fminf(Ua, Ub), Uc);
    float vz   = -(vmax + vmin) * 0.5f;

    float ta = clampf_((Ua + vz + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);
    float tb = clampf_((Ub + vz + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);
    float tc = clampf_((Uc + vz + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);

    uint32_t d0 = (uint32_t)(ta * 1023.0f);
    uint32_t d1 = (uint32_t)(tb * 1023.0f);
    uint32_t d2 = (uint32_t)(tc * 1023.0f);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0, d0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1, d1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)2, d2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)2);
}

// ================================================================
// 开环标定
// ================================================================
static void calibrate(void)
{
    ESP_LOGW(TAG, "==================================================");
    ESP_LOGW(TAG, " 开环标定开始: 对齐 %.1fV + 扫 %d 圈电角度 (%.0f°/步)",
             (double)CAL_ALIGN_V, CAL_EREVS, 360.0 / CAL_STEPS_PER_EREV);
    ESP_LOGW(TAG, "==================================================");

    // ---- 0) 编码器必须先在标定流程里初始化（enc_read_raw 未初始化时恒返回 -1）----
    if (!enc_init(0, ENC_SDA_GPIO, ENC_SCL_GPIO)) {
        ESP_LOGE(TAG, "enc_init 失败: I2C 总线建不起来, 中止");
        return;
    }
    ESP_LOGI(TAG, "编码器就绪 (SDA=%d SCL=%d)", ENC_SDA_GPIO, ENC_SCL_GPIO);

    cal_pwm_init();

    // ---- 1) 对齐: 把矢量固定在电角度 0，让转子转到与矢量重合 ----
    cal_set_vector(0.0f, CAL_ALIGN_V);
    ESP_LOGI(TAG, "对齐中... (%.1f 秒)", CAL_ALIGN_MS / 1000.0f);
    vTaskDelay(pdMS_TO_TICKS(CAL_ALIGN_MS));

    float th0 = enc_read_retry();
    if (th0 < 0.0f) {
        ESP_LOGE(TAG, "对齐后读不到编码器, 中止");
        cal_set_vector(0.0f, 0.0f);
        return;
    }
    ESP_LOGI(TAG, "ALIGN th0 = %.4f rad (%.2f deg)", th0, th0 * 180.0f / (float)M_PI);

    // ---- 2) 扫描: 电角度每步 +30°，共 3 圈 ----
    float acc  = th0;
    float prev = th0;
    const int steps = CAL_STEPS_PER_EREV * CAL_EREVS;
    int ok_cnt = 0;

    for (int k = 1; k <= steps; k++) {
        float elec = (float)k * (2.0f * (float)M_PI / (float)CAL_STEPS_PER_EREV);
        cal_set_vector(elec, CAL_SWEEP_V);
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

        float raw = enc_read_retry();
        if (raw < 0.0f) {
            ESP_LOGW(TAG, "STEP %02d: 编码器读取失败", k);
            continue;
        }

        float d = raw - prev;
        if (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
        if (d < -(float)M_PI) d += 2.0f * (float)M_PI;
        acc += d;
        prev = raw;
        ok_cnt++;

        ESP_LOGI(TAG, "STEP %02d  elec=%7.1f deg  mech=%8.4f rad (%8.2f deg)",
                 k, elec * 180.0f / (float)M_PI, acc, acc * 180.0f / (float)M_PI);
    }

    // ---- 3) 关输出 ----
    cal_set_vector(0.0f, 0.0f);

    float mech_total = acc - th0;
    float elec_total = (float)steps * (2.0f * (float)M_PI / (float)CAL_STEPS_PER_EREV);

    ESP_LOGW(TAG, "==================================================");
    ESP_LOGW(TAG, " 采样成功 %d/%d", ok_cnt, steps);
    ESP_LOGW(TAG, " 电角度总变化 = %.3f rad", (double)elec_total);
    ESP_LOGW(TAG, " 机械角总变化 = %.4f rad (%.2f deg)",
             (double)mech_total, mech_total * 180.0f / (float)M_PI);

    if (fabsf(mech_total) < 0.10f) {
        ESP_LOGE(TAG, " 转子几乎没跟上 -> 电压不够或矢量方向完全不对");
        ESP_LOGE(TAG, " 处理: 把 CAL_SWEEP_V 提到 4.0 ~ 5.0 再试一次");
        ESP_LOGW(TAG, "==================================================");
        return;
    }

    // ---- 4) 反算参数 ----
    // 一圈电角度 ℓ 对应机械角 ℓ/PP  →  PP = Δelec / Δmech (带符号)
    float pp_signed = elec_total / mech_total;
    float pp_abs    = fabsf(pp_signed);
    float pp_round  = roundf(pp_abs);

    // OFFSET = π/2 - PP × θ0   (PP 用带符号值: 负值代表编码器方向反)
    float offset = _PI_2 - pp_signed * th0;
    while (offset < 0.0f)              offset += 2.0f * (float)M_PI;
    while (offset >= 2.0f * (float)M_PI) offset -= 2.0f * (float)M_PI;

    ESP_LOGW(TAG, "--------------------------------------------------");
    ESP_LOGW(TAG, " 结果: 极对数 PP = %.3f  (取整 = %.0f)", (double)pp_abs, (double)pp_round);
    ESP_LOGW(TAG, " 结果: 编码器方向 = %s", pp_signed > 0.0f ? "正 (递增)" : "反 (递减)");
    ESP_LOGW(TAG, " 结果: ELEC_ANGLE_OFFSET = %.5f rad (%.2f deg)",
             (double)offset, (double)(offset * 180.0f / (float)M_PI));
    ESP_LOGW(TAG, "--------------------------------------------------");

    if (pp_signed > 0.0f) {
        ESP_LOGW(TAG, " 建议写入 foc_config.h:");
        ESP_LOGW(TAG, "   #define POLAR_PAIRS          %.0f", (double)pp_round);
        ESP_LOGW(TAG, "   #define ELEC_ANGLE_OFFSET    %.5ff", (double)offset);
    } else {
        ESP_LOGW(TAG, " 编码器方向为负 -> 二选一:");
        ESP_LOGW(TAG, "   A. 物理交换电机任意两根相线 (最干脆)");
        ESP_LOGW(TAG, "   B. POLAR_PAIRS 填 -%.0f (库里 normalize 支持负值)", (double)pp_round);
        ESP_LOGW(TAG, "   然后 ELEC_ANGLE_OFFSET = %.5ff", (double)offset);
    }
    ESP_LOGW(TAG, "==================================================");

    // ---- 5) 待机观测 ----
    while (1) {
        enc_update(0);
        ESP_LOGI(TAG, "idle: raw=%.4f rad", enc_get_cached_angle(0));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ================================================================
// 编码器观测 (APP_MODE = 0)
// ================================================================
static bool enc_probe(void)
{
    ESP_LOGI(TAG, "--- 编码器自检 (I2C0: SDA=%d, SCL=%d) ---", ENC_SDA_GPIO, ENC_SCL_GPIO);

    if (!enc_init(0, ENC_SDA_GPIO, ENC_SCL_GPIO)) {
        ESP_LOGE(TAG, "enc_init 失败");
        return false;
    }

    int   ok_cnt = 0;
    float amin = 99.0f, amax = -99.0f;

    for (int i = 0; i < ENC_PROBE_SAMPLES; i++) {
        float a = enc_read_raw(0);
        if (a >= 0.0f) {
            ok_cnt++;
            if (a < amin) amin = a;
            if (a > amax) amax = a;
            ESP_LOGI(TAG, "  ENC[%02d] %.4f rad (%.1f deg)", i, a, a * 180.0f / (float)M_PI);
        } else {
            ESP_LOGW(TAG, "  ENC[%02d] 读取失败", i);
        }
        vTaskDelay(pdMS_TO_TICKS(ENC_PROBE_INTERVAL_MS));
    }

    if (ok_cnt == 0) {
        ESP_LOGE(TAG, "编码器 0/%d 应答 -> 检查 SDA/SCL、供电、上拉", ENC_PROBE_SAMPLES);
        return false;
    }
    ESP_LOGI(TAG, "编码器 OK: %d/%d, 角度 %.4f ~ %.4f rad", ok_cnt, ENC_PROBE_SAMPLES, amin, amax);
    return true;
}

// ================================================================
// 分级自检 (APP_MODE = 1)
// ================================================================
static void self_test(void)
{
    if (!enc_probe()) {
        ESP_LOGE(TAG, "编码器不可用, 中止");
        return;
    }

    motor_foc_init();
    motor_torque(0, 0.0f);

    ESP_LOGW(TAG, "--- 力矩锁轴: %.2f (约 %.2fV), %d 秒 ---",
             TORQUE_TEST_VALUE, TORQUE_TEST_VALUE * TORQUE_MAX_V, TORQUE_TEST_SECONDS);
    motor_torque(0, TORQUE_TEST_VALUE);
    for (int ms = 0; ms < TORQUE_TEST_SECONDS * 1000; ms++) {
        motor_update(0);
        esp_rom_delay_us(CTRL_STEP_US);
    }
    motor_torque(0, 0.0f);
    for (int ms = 0; ms < 100; ms++) {
        motor_update(0);
        esp_rom_delay_us(CTRL_STEP_US);
    }

    ESP_LOGW(TAG, "--- 速度环: %.1f rad/s, %d 秒 ---", SPEED_TEST_VALUE, SPEED_TEST_SECONDS);
    motor_speed(0, SPEED_TEST_VALUE);
    for (int ms = 0; ms < SPEED_TEST_SECONDS * 1000; ms++) {
        motor_update(0);
        esp_rom_delay_us(CTRL_STEP_US);
    }
    motor_torque(0, 0.0f);
}

#if APP_MODE == 9
// ================================================================
// 原有功能: 总线舵机 ID 扫描
// ================================================================
static void servo_id_scan(void)
{
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "=== Scanning IDs 0-10 at 115200 ===");

    int found = 0;
    for (int id = 0; id <= 10; id++) {
        if (duoji_id_exists((uint8_t)id)) {
            ESP_LOGI(TAG, "ID %d EXISTS", id);
            found++;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (found == 0) {
        ESP_LOGE(TAG, "No servo found at 115200!");
    } else {
        ESP_LOGI(TAG, "Found %d servo(s).", found);
    }
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

// ================================================================
// 入口
// ================================================================
extern "C" void app_main(void)
{
#if APP_MODE == 9
    servo_id_scan();
    return;
#else
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, " 2804 FOC   VBUS=%.1fV  PP=%d  OFFSET=%.4f rad",
             (double)VBUS, POLAR_PAIRS, (double)ELEC_ANGLE_OFFSET);
    ESP_LOGI(TAG, " 相线 A=%d B=%d C=%d",
             MOTOR_PHASE_A_GPIO_0, MOTOR_PHASE_B_GPIO_0, MOTOR_PHASE_C_GPIO_0);
    ESP_LOGI(TAG, "=================================================");
    vTaskDelay(pdMS_TO_TICKS(500));

#if APP_MODE == 2
    calibrate();
#elif APP_MODE == 1
    self_test();
#elif APP_MODE == 5
    {
        // ============ AS5600 寄存器诊断 ============
        // STATUS(0x0B): bit5=MD 检出磁铁, bit4=ML 磁场太弱, bit3=MH 磁场太强
        // AGC(0x1A): 自动增益 0~255, 越小表示磁场越强
        // MAG(0x1B): 磁场幅值
        // ANGLE(0x0C) / RAW(0x0E): 角度寄存器
        i2c_master_bus_config_t bc = {};
        bc.i2c_port = (i2c_port_t)0;
        bc.sda_io_num = (gpio_num_t)ENC_SDA_GPIO;
        bc.scl_io_num = (gpio_num_t)ENC_SCL_GPIO;
        bc.clk_source = I2C_CLK_SRC_DEFAULT;
        bc.glitch_ignore_cnt = 7;
        bc.flags.enable_internal_pullup = true;

        i2c_master_bus_handle_t bus = NULL;
        esp_err_t err = i2c_new_master_bus(&bc, &bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus 失败 err=%d (SDA=%d SCL=%d)",
                     (int)err, ENC_SDA_GPIO, ENC_SCL_GPIO);
            return;
        }

        i2c_device_config_t dc = {};
        dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dc.device_address  = 0x36;
        dc.scl_speed_hz    = 100000;

        i2c_master_dev_handle_t dev = NULL;
        err = i2c_master_bus_add_device(bus, &dc, &dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add_device 失败 err=%d", (int)err);
            return;
        }

        ESP_LOGW(TAG, "AS5600 诊断开始 (每 300ms 一次)");
        ESP_LOGW(TAG, "正常判据: MD=1 ML=0 MH=0, AGC 约 100~200, ANGLE 随手转变化");

        while (1) {
            uint8_t reg;
            uint8_t b1[1] = {0xFF};
            uint8_t b2[2] = {0xFF, 0xFF};
            int status = -1, agc = -1, mag = -1, ang = -1, rawang = -1;

            reg = 0x0B; if (i2c_master_transmit_receive(dev, &reg, 1, b1, 1, 50) == ESP_OK) status = b1[0];
            reg = 0x1A; if (i2c_master_transmit_receive(dev, &reg, 1, b1, 1, 50) == ESP_OK) agc = b1[0];
            reg = 0x1B; if (i2c_master_transmit_receive(dev, &reg, 1, b2, 2, 50) == ESP_OK) mag = (b2[0] << 8) | b2[1];
            reg = 0x0E; if (i2c_master_transmit_receive(dev, &reg, 1, b2, 2, 50) == ESP_OK) rawang = (b2[0] << 8) | b2[1];
            reg = 0x0C; if (i2c_master_transmit_receive(dev, &reg, 1, b2, 2, 50) == ESP_OK) ang = (b2[0] << 8) | b2[1];

            ESP_LOGI(TAG, "STATUS=0x%02X MD=%d ML=%d MH=%d | AGC=%3d MAG=%5d | ANGLE=%4d RAW=%4d",
                     status & 0xFF, (status >> 5) & 1, (status >> 4) & 1, (status >> 3) & 1,
                     agc, mag, ang, rawang);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
#else
    if (enc_probe()) {
        ESP_LOGW(TAG, "APP_MODE=0: 只观测编码器, 不驱动电机");
    }
    while (1) {
        enc_update(0);
        ESP_LOGI(TAG, "idle: raw=%.4f rad  speed=%.2f",
                 enc_get_cached_angle(0), enc_get_speed(0));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
#endif
}
