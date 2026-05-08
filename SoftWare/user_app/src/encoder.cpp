/*********************************************************************************************************************
 * 文件名称：encoder.cpp
 * 功能描述：编码器模块实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 完整数据处理流程：
 *
 *   enc->encoder_get_count()       ← 硬件原始值（float，含正负方向）
 *           │
 *           ▼
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 分支 1：保留方向 → encoder_L/R_signed                         │
 *   │   用途：姿态估计、里程计、轨迹追踪                              │
 *   └──────────────────────────────────────────────────────────────┘
 *           │
 *           ▼
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 分支 2：取绝对值                                                │
 *   │           ↓                                                     │
 *   │   encoder_L/R_oringinal （原始速率）                            │
 *   │           ↓                                                     │
 *   │       × 100 → uint32_t  （定点化，避免浮点累积误差）            │
 *   │           ↓                                                     │
 *   │   一阶低通滤波（α=10%，旧值90%）                                 │
 *   │           ↓                                                     │
 *   │       ÷ 100 → float                                             │
 *   │           ↓                                                     │
 *   │   encoder_L/R_final  ← 闭环 PID 用这个                          │
 *   │                                                                 │
 *   │   同时进入滑动窗口（窗口=5）→ encoder_L/R_smooth                │
 *   └──────────────────────────────────────────────────────────────┘
 *           │
 *           ▼
 *   累加到 encoder_L/R_total → 用于里程估算
 *
 * ─────────────────────────────────────────────────────────────────────
 * Bug修复：
 *   - encoderL_100 改为 uint32_t（原 uint16_t 在高速时会溢出）
 *   - encoder_update 内部增加运行状态守卫，停车后不再累加里程
 ********************************************************************************************************************/

#include "encoder.h"
#include <stdio.h>
#include <math.h>

// ══════════════════════════════════════════════════════════════
// 全局变量定义
// ══════════════════════════════════════════════════════════════

// 原始绝对值（向下兼容旧代码）
float encoderL_oringinal = 0.0f;
float encoderR_oringinal = 0.0f;

// 一阶低通滤波后的绝对值（业务主用）
float encoder_L_final = 0.0f;
float encoder_R_final = 0.0f;

// 带符号速度（新增，方向保留）
float encoder_L_signed = 0.0f;
float encoder_R_signed = 0.0f;

// 滑动窗口滤波后的值（新增，更平滑）
float encoder_L_smooth = 0.0f;
float encoder_R_smooth = 0.0f;

// 累积计数（用于里程估算）
float encoder_L_total = 0.0f;
float encoder_R_total = 0.0f;

// ══════════════════════════════════════════════════════════════
// 内部状态变量
// ══════════════════════════════════════════════════════════════

// 一阶低通滤波中间变量
// 修复：由 uint16_t 升级为 uint32_t，防止高速值溢出
static uint32_t encoderL_100 = 0;
static uint32_t encoderR_100 = 0;
static uint32_t encoderL_last = 0;
static uint32_t encoderR_last = 0;

// 滑动窗口缓冲区
static float window_L[ENCODER_SMOOTH_WINDOW] = {0};
static float window_R[ENCODER_SMOOTH_WINDOW] = {0};
static int window_idx = 0; // 环形索引

// 编码器设备实例
static ls_encoder_pwm *enc1 = nullptr; // 左电机（4号PWM）
static ls_encoder_pwm *enc4 = nullptr; // 右电机（1号PWM）

// ══════════════════════════════════════════════════════════════
// 内部辅助：滑动窗口求平均
// ══════════════════════════════════════════════════════════════
static float window_average(const float *buf)
{
    float sum = 0.0f;
    for (int i = 0; i < ENCODER_SMOOTH_WINDOW; i++)
        sum += buf[i];
    return sum / (float)ENCODER_SMOOTH_WINDOW;
}

// ══════════════════════════════════════════════════════════════
// 编码器初始化
// ══════════════════════════════════════════════════════════════
void encoder_Init(void)
{
    // 防止重复初始化
    if (!enc4)
        enc4 = new ls_encoder_pwm(ENC_PWM0_PIN64, PIN_72); // 右电机，1号编码器

    if (!enc1)
        enc1 = new ls_encoder_pwm(ENC_PWM3_PIN67, PIN_75); // 左电机，4号编码器

    // 备用编码器（当前未使用，按需启用）
    // if (!enc2) enc2 = new ls_encoder_pwm(ENC_PWM1_PIN65, PIN_73);
    // if (!enc3) enc3 = new ls_encoder_pwm(ENC_PWM2_PIN66, PIN_74);

    // 清空所有状态
    encoder_reset();

    printf("[Encoder] Init done. window=%d alpha=%d/10 mm_per_unit=%.3f\n",
           ENCODER_SMOOTH_WINDOW, ENCODER_LPF_ALPHA, ENCODER_MM_PER_UNIT);
}

// ══════════════════════════════════════════════════════════════
// 状态清空
// ══════════════════════════════════════════════════════════════
void encoder_reset(void)
{
    encoderL_oringinal = 0.0f;
    encoderR_oringinal = 0.0f;
    encoder_L_final = 0.0f;
    encoder_R_final = 0.0f;
    encoder_L_signed = 0.0f;
    encoder_R_signed = 0.0f;
    encoder_L_smooth = 0.0f;
    encoder_R_smooth = 0.0f;
    encoder_L_total = 0.0f;
    encoder_R_total = 0.0f;

    encoderL_100 = 0;
    encoderR_100 = 0;
    encoderL_last = 0;
    encoderR_last = 0;

    for (int i = 0; i < ENCODER_SMOOTH_WINDOW; i++)
    {
        window_L[i] = 0.0f;
        window_R[i] = 0.0f;
    }
    window_idx = 0;
}

void encoder_reset_total(void)
{
    encoder_L_total = 0.0f;
    encoder_R_total = 0.0f;
}

// ══════════════════════════════════════════════════════════════
// 编码器数据更新（定时器周期调用，典型 5ms）
// ══════════════════════════════════════════════════════════════
void encoder_update(void)
{
    // ── 守卫 1：系统未运行（停车中）───────────────────────────
    if (!ls_system_running.load())
        return;

    // ── 守卫 2：编码器未初始化 ────────────────────────────────
    if (!enc1 || !enc4)
        return;

    // ── 步骤 1：读取硬件原始值（带符号）──────────────────────
    float raw_L = enc1->encoder_get_count();
    float raw_R = enc4->encoder_get_count();

    // ── 步骤 2：更新带符号值（保留方向，新增分支）────────────
    encoder_L_signed = raw_L;
    encoder_R_signed = raw_R;

    // ── 步骤 3：取绝对值（业务主分支）────────────────────────
    encoderL_oringinal = (raw_L < 0.0f) ? -raw_L : raw_L;
    encoderR_oringinal = (raw_R < 0.0f) ? -raw_R : raw_R;

    // ── 步骤 4：定点化（×100），避免浮点累积误差 ─────────────
    encoderL_100 = (uint32_t)(encoderL_oringinal * 100.0f);
    encoderR_100 = (uint32_t)(encoderR_oringinal * 100.0f);

    // ── 步骤 5：一阶低通滤波 ─────────────────────────────────
    // 公式： y[n] = α · x[n] + (1-α) · y[n-1]
    //        其中 α = ENCODER_LPF_ALPHA / 10
    // 写法上：先乘后除，减少整数除法的精度损失
    const uint32_t alpha = ENCODER_LPF_ALPHA;
    const uint32_t one_minus_alpha = 10 - ENCODER_LPF_ALPHA;
    uint32_t encL_new = (encoderL_100 * alpha + encoderL_last * one_minus_alpha) / 10;
    uint32_t encR_new = (encoderR_100 * alpha + encoderR_last * one_minus_alpha) / 10;

    encoderL_last = encL_new;
    encoderR_last = encR_new;

    // ── 步骤 6：转回浮点，写入业务接口 ───────────────────────
    encoder_L_final = encL_new / 100.0f;
    encoder_R_final = encR_new / 100.0f;

    // ── 步骤 7：滑动窗口滤波 ─────────────────────────────────
    window_L[window_idx] = encoderL_oringinal;
    window_R[window_idx] = encoderR_oringinal;
    window_idx = (window_idx + 1) % ENCODER_SMOOTH_WINDOW;
    encoder_L_smooth = window_average(window_L);
    encoder_R_smooth = window_average(window_R);

    // ── 步骤 8：累积里程（用带符号值，可识别倒车）─────────────
    encoder_L_total += encoder_L_signed;
    encoder_R_total += encoder_R_signed;
}

// ══════════════════════════════════════════════════════════════
// 里程换算接口
// ══════════════════════════════════════════════════════════════
float encoder_get_left_distance_mm(void)
{
    return encoder_L_total * ENCODER_MM_PER_UNIT;
}

float encoder_get_right_distance_mm(void)
{
    return encoder_R_total * ENCODER_MM_PER_UNIT;
}

float encoder_get_avg_distance_mm(void)
{
    // 取双轮绝对值平均（直行时近似车体行驶距离）
    float l = encoder_get_left_distance_mm();
    float r = encoder_get_right_distance_mm();
    if (l < 0)
        l = -l;
    if (r < 0)
        r = -r;
    return (l + r) * 0.5f;
}

// ══════════════════════════════════════════════════════════════
// 调试打印
// ══════════════════════════════════════════════════════════════
void encoder_print(void)
{
    printf("[ENC] L_signed=%7.2f R_signed=%7.2f | L_final=%6.2f R_final=%6.2f | L_smooth=%6.2f R_smooth=%6.2f | dist_avg=%.1fmm\n",
           encoder_L_signed, encoder_R_signed,
           encoder_L_final, encoder_R_final,
           encoder_L_smooth, encoder_R_smooth,
           encoder_get_avg_distance_mm());
}