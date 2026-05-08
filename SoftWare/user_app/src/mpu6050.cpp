/*********************************************************************************************************************
 * 文件名称：mpu6050.cpp
 * 功能描述：陀螺仪模块（MPU6050）实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 关键算法：
 *
 *   1. 零偏校准（Bias Calibration）
 *      - 静止状态下 MPU 输出应该是：ax≈0, ay≈0, az≈1g, gx≈gy≈gz≈0
 *      - 实际硬件总有零偏（比如 gz 静止时也输出 +5）
 *      - 解决：boot 时采样 200 次取平均作为零偏
 *      - 注意 az 不能完全减掉重力，要保留 az - 16384（即减重力分量）
 *
 *   2. 互补滤波（Complementary Filter）
 *      pitch = α·(pitch + gyro_x·dt) + (1-α)·atan2(ay, az)
 *      - 高通滤波陀螺仪（短期响应快但会漂移）
 *      - 低通滤波加速度计（无漂移但抗扰差）
 *      - 两者融合得到稳定且响应快的姿态角
 *
 *   3. yaw 角说明
 *      yaw 没有像 pitch/roll 那样能用加速度修正（因为重力不影响 yaw）
 *      只能纯陀螺积分：yaw += gz·dt
 *      没有磁力计辅助会有漂移，仅适合短时间使用
 ********************************************************************************************************************/

#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ══════════════════════════════════════════════════════════════
// 全局变量定义
// ══════════════════════════════════════════════════════════════

// 原始 ADC 值（向下兼容）
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

// 物理量
float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;

// 零偏值
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

// 姿态角
float g_pitch = 0.0f;
float g_roll = 0.0f;
float g_yaw = 0.0f;

// 状态标志
bool g_mpu_online = false;
bool g_mpu_calibrated = false;

// ══════════════════════════════════════════════════════════════
// 内部状态变量
// ══════════════════════════════════════════════════════════════
static bool s_init_done = false;  // 已尝试过初始化
static uint16_t s_fail_count = 0; // 连续读取失败次数

// ══════════════════════════════════════════════════════════════
// 设备单例
// ══════════════════════════════════════════════════════════════
static lq_i2c_mpu6050 &mpu6050_dev()
{
    // 修复原代码 bug：原来这里是 lq_i2c_mpu6500 类型不一致
    static lq_i2c_mpu6050 dev;
    return dev;
}

// ══════════════════════════════════════════════════════════════
// 内部函数：执行零偏校准
// 调用前需确保设备在线
// ══════════════════════════════════════════════════════════════
static bool perform_calibration(void)
{
    printf("[MPU6050] Calibrating bias... please keep the car STILL.\n");

    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid_count = 0;

    int16_t tax, tay, taz, tgx, tgy, tgz;

    for (int i = 0; i < MPU_CALIB_SAMPLES; i++)
    {
        if (mpu6050_dev().get_mpu6050_gyro(&tax, &tay, &taz, &tgx, &tgy, &tgz))
        {
            sum_ax += tax;
            sum_ay += tay;
            sum_az += taz;
            sum_gx += tgx;
            sum_gy += tgy;
            sum_gz += tgz;
            valid_count++;
        }
        // 简单延迟，避免读太快（约 1ms）
        for (volatile int j = 0; j < 1000; j++)
            ;
    }

    if (valid_count < MPU_CALIB_SAMPLES / 2)
    {
        printf("[MPU6050] Calibration FAILED. Valid samples: %d/%d\n",
               valid_count, MPU_CALIB_SAMPLES);
        return false;
    }

    ax_offset = (int16_t)(sum_ax / valid_count);
    ay_offset = (int16_t)(sum_ay / valid_count);
    // az 不全减，要保留重力分量（理想情况 az = 16384 = 1g）
    // 这里减掉的是 (实测值 - 1g)，运行时再减 az_offset，相当于把"实际重力"对齐到 1g
    az_offset = (int16_t)(sum_az / valid_count) - (int16_t)MPU_ACC_LSB_PER_G;

    gx_offset = (int16_t)(sum_gx / valid_count);
    gy_offset = (int16_t)(sum_gy / valid_count);
    gz_offset = (int16_t)(sum_gz / valid_count);

    printf("[MPU6050] Calibration done. Offsets:\n");
    printf("          accel: ax=%d ay=%d az=%d\n",
           ax_offset, ay_offset, az_offset);
    printf("          gyro:  gx=%d gy=%d gz=%d\n",
           gx_offset, gy_offset, gz_offset);

    return true;
}

// ══════════════════════════════════════════════════════════════
// 内部函数：互补滤波更新姿态角
// ══════════════════════════════════════════════════════════════
static void update_attitude(void)
{
    // ── 1. 由加速度计算 pitch / roll（仅在静止/低加速度时准确）─
    // pitch 绕 X 轴：用 ay/az
    // roll  绕 Y 轴：用 ax/az
    // 注意 atan2(y, x) 输出弧度，要转角度
    float acc_pitch = atan2f((float)ay_g, (float)az_g) * 180.0f / (float)M_PI;
    float acc_roll = atan2f((float)ax_g, (float)az_g) * 180.0f / (float)M_PI;

    // ── 2. 陀螺积分得到角度变化（短期准确）────────────────────
    // gyro 是角速度（°/s），乘以 dt 得到本周期角度变化
    float dpitch = gx_dps * MPU_DT;
    float droll = gy_dps * MPU_DT;
    float dyaw = gz_dps * MPU_DT;

    // ── 3. 互补滤波融合 ──────────────────────────────────────
    // α 越大 → 越信任陀螺（响应快）
    // (1-α) 越大 → 越信任加速度计（抗漂移）
    g_pitch = MPU_COMP_ALPHA * (g_pitch + dpitch) + (1.0f - MPU_COMP_ALPHA) * acc_pitch;
    g_roll = MPU_COMP_ALPHA * (g_roll + droll) + (1.0f - MPU_COMP_ALPHA) * acc_roll;

    // yaw 无加速度参考，纯积分（会漂移）
    g_yaw += dyaw;

    // yaw 角归一化到 [-180, 180]
    if (g_yaw > 180.0f)
        g_yaw -= 360.0f;
    if (g_yaw < -180.0f)
        g_yaw += 360.0f;
}

// ══════════════════════════════════════════════════════════════
// 公共接口：初始化（含校准）
// ══════════════════════════════════════════════════════════════
void mpu6050_init(void)
{
    if (s_init_done)
        return;
    s_init_done = true;

    // 检测设备是否在线
    if (mpu6050_dev().get_mpu6050_id() == 0)
    {
        printf("[MPU6050] Device not found. Disabled.\n");
        g_mpu_online = false;
        return;
    }

    g_mpu_online = true;
    printf("[MPU6050] Device online. ID OK.\n");

    // 执行零偏校准
    g_mpu_calibrated = perform_calibration();
}

// ══════════════════════════════════════════════════════════════
// 公共接口：周期任务
// ══════════════════════════════════════════════════════════════
void mpu6050_loop(void)
{
    // 第一次调用：自动初始化（懒初始化）
    if (!s_init_done)
    {
        mpu6050_init();
    }

    if (!g_mpu_online)
        return;

    // ── 1. 读取原始数据 ──────────────────────────────────────
    if (!mpu6050_dev().get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz))
    {
        if (s_fail_count < 0xFFFF)
            s_fail_count++;
        if (s_fail_count >= MPU_FAIL_THRESHOLD)
        {
            g_mpu_online = false;
            printf("[MPU6050] Too many failures, marked offline.\n");
        }
        return;
    }
    s_fail_count = 0;

    // ── 2. 减去零偏 ──────────────────────────────────────────
    int16_t ax_cal = ax - ax_offset;
    int16_t ay_cal = ay - ay_offset;
    int16_t az_cal = az - az_offset;
    int16_t gx_cal = gx - gx_offset;
    int16_t gy_cal = gy - gy_offset;
    int16_t gz_cal = gz - gz_offset;

    // ── 3. 单位换算 ──────────────────────────────────────────
    ax_g = (float)ax_cal / MPU_ACC_LSB_PER_G;
    ay_g = (float)ay_cal / MPU_ACC_LSB_PER_G;
    az_g = (float)az_cal / MPU_ACC_LSB_PER_G;

    gx_dps = (float)gx_cal / MPU_GYRO_LSB_PER_DPS;
    gy_dps = (float)gy_cal / MPU_GYRO_LSB_PER_DPS;
    gz_dps = (float)gz_cal / MPU_GYRO_LSB_PER_DPS;

    // ── 4. 姿态融合（互补滤波）───────────────────────────────
    if (g_mpu_calibrated)
    {
        update_attitude();
    }
}

// ══════════════════════════════════════════════════════════════
// 重置接口
// ══════════════════════════════════════════════════════════════
void mpu6050_reset_yaw(void)
{
    g_yaw = 0.0f;
    printf("[MPU6050] yaw reset to 0\n");
}

void mpu6050_reset_attitude(void)
{
    g_pitch = 0.0f;
    g_roll = 0.0f;
    g_yaw = 0.0f;
    printf("[MPU6050] attitude reset (pitch/roll/yaw = 0)\n");
}

void mpu6050_recalibrate(void)
{
    if (!g_mpu_online)
    {
        printf("[MPU6050] Device offline, cannot recalibrate.\n");
        return;
    }
    g_mpu_calibrated = perform_calibration();
    mpu6050_reset_attitude();
}

// ══════════════════════════════════════════════════════════════
// 调试打印
// ══════════════════════════════════════════════════════════════
void mpu6050_print(void)
{
    if (!g_mpu_online)
    {
        printf("[MPU6050] OFFLINE\n");
        return;
    }
    printf("[MPU6050] acc(g)=[%+.2f %+.2f %+.2f]  gyro(dps)=[%+.1f %+.1f %+.1f]  attitude=[P=%+.1f R=%+.1f Y=%+.1f]\n",
           ax_g, ay_g, az_g,
           gx_dps, gy_dps, gz_dps,
           g_pitch, g_roll, g_yaw);
}

// ══════════════════════════════════════════════════════════════
// 预留扩展接口
// ══════════════════════════════════════════════════════════════
void mpu6050_update(float fax, float fay, float faz,
                    float fgx, float fgy, float fgz)
{
    // 预留：用户可在此处实现自定义滤波/卡尔曼等
    (void)fax;
    (void)fay;
    (void)faz;
    (void)fgx;
    (void)fgy;
    (void)fgz;
}