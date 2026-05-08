/*********************************************************************************************************************
 * 文件名称：distance_sensor.cpp
 * 功能描述：VL53L0X 激光测距传感器封装实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 关键改进：
 *
 *   1. 复合滤波（中位数 + 均值）
 *      - 原版本只有 20%/80% 一阶低通，遇到尖刺会被慢慢"吃进"滤波器
 *      - 新方案：5点缓冲，每次去掉最大最小，剩下 3 个求平均
 *      - 效果：尖刺直接被丢弃，不会污染输出
 *
 *   2. 区间分类
 *      - CRITICAL/NEAR/MID/FAR/SAFE 五级
 *      - 业务层不用自己写阈值判断
 *
 *   3. 临界回调
 *      - 进入 CRITICAL 区时自动触发（边沿触发，不会重复调）
 *      - 典型绑定：close_loop_emergency_stop()
 *
 *   4. 变化率检测
 *      - 维护历史距离，判断是否在快速接近
 *      - 用于"还没到危险距离但快撞了"的预警场景
 *
 *   5. 故障计数
 *      - 连续读取失败 >= 阈值 → 标记设备离线
 *      - 业务层可查询 distance_is_online() 决定是否信任数据
 ********************************************************************************************************************/

#include "distance_sensor.h"
#include <stdio.h>
#include <string.h>

// ══════════════════════════════════════════════════════════════
// 全局变量定义
// ══════════════════════════════════════════════════════════════
uint16_t g_distance = 0;
uint16_t g_distance_raw = 0;
bool g_distance_valid = false;
DistZone_t g_distance_zone = DIST_ZONE_INVALID;

// ══════════════════════════════════════════════════════════════
// 内部状态变量
// ══════════════════════════════════════════════════════════════

// 滑动窗口缓冲（用于中位数+均值滤波）
static uint16_t s_window[DIST_FILTER_WINDOW] = {0};
static int s_window_idx = 0;
static int s_window_filled = 0; // 已填入数据数量（避免初期空槽位干扰）

// 上一次的区间状态（用于边沿触发回调）
static DistZone_t s_last_zone = DIST_ZONE_INVALID;

// 历史距离（用于变化率检测）
static uint16_t s_history[DIST_APPROACH_TICKS] = {0};
static int s_history_idx = 0;
static int s_history_filled = 0;

// 临界回调
static dist_critical_cb_t s_critical_cb = nullptr;

// 故障计数
static uint16_t s_fail_count = 0;

// 滤波后的最大/最小/均值（供查询接口使用）
static uint16_t s_filt_max = 0;
static uint16_t s_filt_min = 0;
static uint16_t s_filt_avg = 0;

// ══════════════════════════════════════════════════════════════
// 设备单例
// ══════════════════════════════════════════════════════════════
static lq_i2c_vl53l0x &vl53l0x_dev()
{
    static lq_i2c_vl53l0x dev;
    return dev;
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：从窗口中找最大、最小、均值
// 同时计算去掉首尾后的修剪均值（更抗噪）
// ══════════════════════════════════════════════════════════════
static uint16_t window_trimmed_mean(uint16_t *out_max, uint16_t *out_min, uint16_t *out_avg)
{
    int n = (s_window_filled < DIST_FILTER_WINDOW) ? s_window_filled : DIST_FILTER_WINDOW;
    if (n == 0)
    {
        if (out_max)
            *out_max = 0;
        if (out_min)
            *out_min = 0;
        if (out_avg)
            *out_avg = 0;
        return 0;
    }

    // 拷贝一份用于排序
    uint16_t tmp[DIST_FILTER_WINDOW];
    memcpy(tmp, s_window, sizeof(uint16_t) * n);

    // 简单冒泡排序（n <= 5，开销可忽略）
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = 0; j < n - 1 - i; j++)
        {
            if (tmp[j] > tmp[j + 1])
            {
                uint16_t t = tmp[j];
                tmp[j] = tmp[j + 1];
                tmp[j + 1] = t;
            }
        }
    }

    if (out_min)
        *out_min = tmp[0];
    if (out_max)
        *out_max = tmp[n - 1];

    // 普通均值
    uint32_t sum = 0;
    for (int i = 0; i < n; i++)
        sum += tmp[i];
    if (out_avg)
        *out_avg = (uint16_t)(sum / n);

    // 修剪均值：n>=3 时去掉头尾，否则直接返回均值
    if (n >= 3)
    {
        uint32_t trim_sum = 0;
        for (int i = 1; i < n - 1; i++)
            trim_sum += tmp[i];
        return (uint16_t)(trim_sum / (n - 2));
    }
    else
    {
        return (uint16_t)(sum / n);
    }
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：根据距离值确定所属区间
// ══════════════════════════════════════════════════════════════
static DistZone_t classify_zone(uint16_t d, bool valid)
{
    if (!valid)
        return DIST_ZONE_INVALID;
    if (d < DIST_CRITICAL_THRESHOLD)
        return DIST_ZONE_CRITICAL;
    if (d < DIST_NEAR_THRESHOLD)
        return DIST_ZONE_NEAR;
    if (d < DIST_MID_THRESHOLD)
        return DIST_ZONE_MID;
    if (d < DIST_FAR_THRESHOLD)
        return DIST_ZONE_FAR;
    return DIST_ZONE_SAFE;
}

// ══════════════════════════════════════════════════════════════
// 测距数据更新（定时器周期调用，典型 5ms）
// ══════════════════════════════════════════════════════════════
void distance_sensor_update(void)
{
    // ── 1. 读取硬件 ───────────────────────────────────────────
    uint16_t raw = vl53l0x_dev().get_vl53l0x_dis();
    g_distance_raw = raw;

    // ── 2. 有效性检查 ─────────────────────────────────────────
    if (raw == 0 || raw >= DIST_INVALID)
    {
        // 0 通常表示读取失败（如 I2C 通信失败）
        // 8190 表示超量程
        g_distance_valid = false;
        if (s_fail_count < 0xFFFF)
            s_fail_count++;
        // 不更新滤波值，保留上次有效数据
        // 但区间需要重新分类
        g_distance_zone = DIST_ZONE_INVALID;
        return;
    }

    // 读取成功 → 清零故障计数
    g_distance_valid = true;
    s_fail_count = 0;

    // ── 3. 推入滑动窗口 ───────────────────────────────────────
    s_window[s_window_idx] = raw;
    s_window_idx = (s_window_idx + 1) % DIST_FILTER_WINDOW;
    if (s_window_filled < DIST_FILTER_WINDOW)
        s_window_filled++;

    // ── 4. 复合滤波（中位数+均值）─────────────────────────────
    g_distance = window_trimmed_mean(&s_filt_max, &s_filt_min, &s_filt_avg);

    // ── 5. 区间分类 ───────────────────────────────────────────
    g_distance_zone = classify_zone(g_distance, true);

    // ── 6. 临界回调（边沿触发）────────────────────────────────
    // 只在"刚刚进入 CRITICAL 区"的瞬间触发一次
    // 防止持续在临界区时回调被高频调用
    if (g_distance_zone == DIST_ZONE_CRITICAL && s_last_zone != DIST_ZONE_CRITICAL)
    {
        if (s_critical_cb != nullptr)
        {
            printf("[DIST] !! CRITICAL ENTERED !! distance=%u mm, calling cb\n", g_distance);
            s_critical_cb(g_distance);
        }
    }
    s_last_zone = g_distance_zone;

    // ── 7. 历史缓冲（用于变化率检测）──────────────────────────
    s_history[s_history_idx] = g_distance;
    s_history_idx = (s_history_idx + 1) % DIST_APPROACH_TICKS;
    if (s_history_filled < DIST_APPROACH_TICKS)
        s_history_filled++;
}

// ══════════════════════════════════════════════════════════════
// 调试打印
// ══════════════════════════════════════════════════════════════
void distance_sensor_print(void)
{
    printf("[DIST] raw=%4u filt=%4u  zone=%-9s  valid=%s  fail=%u  [min=%u max=%u avg=%u]\n",
           g_distance_raw, g_distance,
           distance_zone_str(),
           g_distance_valid ? "YES" : "NO",
           s_fail_count,
           s_filt_min, s_filt_max, s_filt_avg);
}

// ══════════════════════════════════════════════════════════════
// 基础查询接口
// ══════════════════════════════════════════════════════════════
uint16_t distance_get(void)
{
    return g_distance_valid ? g_distance : DIST_INVALID;
}

uint16_t distance_get_raw(void)
{
    return g_distance_raw;
}

// ══════════════════════════════════════════════════════════════
// 区间判断接口
// ══════════════════════════════════════════════════════════════
bool distance_has_obstacle(void)
{
    if (!g_distance_valid)
        return false;
    return g_distance < DIST_MID_THRESHOLD;
}

bool distance_is_near(void)
{
    if (!g_distance_valid)
        return false;
    return g_distance < DIST_NEAR_THRESHOLD;
}

bool distance_is_far(void)
{
    if (!g_distance_valid)
        return false;
    return g_distance > DIST_FAR_THRESHOLD;
}

bool distance_is_critical(void)
{
    if (!g_distance_valid)
        return false;
    return g_distance < DIST_CRITICAL_THRESHOLD;
}

DistZone_t distance_get_zone(void)
{
    return g_distance_zone;
}

const char *distance_zone_str(void)
{
    switch (g_distance_zone)
    {
    case DIST_ZONE_CRITICAL:
        return "CRITICAL";
    case DIST_ZONE_NEAR:
        return "NEAR";
    case DIST_ZONE_MID:
        return "MID";
    case DIST_ZONE_FAR:
        return "FAR";
    case DIST_ZONE_SAFE:
        return "SAFE";
    case DIST_ZONE_INVALID:
        return "INVALID";
    default:
        return "UNKNOWN";
    }
}

// ══════════════════════════════════════════════════════════════
// 高级特性
// ══════════════════════════════════════════════════════════════

bool distance_is_approaching_fast(void)
{
    // 历史数据不足时无法判断
    if (s_history_filled < DIST_APPROACH_TICKS)
        return false;
    if (!g_distance_valid)
        return false;

    // 取窗口最早的那一帧（即 (idx) 位置，因为环形写入后下一个就是最旧）
    int oldest_idx = s_history_idx; // 当前 idx 指向下一个要写的位置 = 最旧的位置
    uint16_t oldest = s_history[oldest_idx];

    if (oldest == 0)
        return false; // 旧数据无效

    // 判断：从过去到现在距离减少超过阈值
    if (oldest > g_distance && (oldest - g_distance) >= DIST_APPROACH_DELTA)
    {
        return true;
    }
    return false;
}

void distance_register_critical_cb(dist_critical_cb_t cb)
{
    s_critical_cb = cb;
    if (cb)
        printf("[DIST] Critical callback registered.\n");
    else
        printf("[DIST] Critical callback cleared.\n");
}

uint16_t distance_get_max(void) { return s_filt_max; }
uint16_t distance_get_min(void) { return s_filt_min; }
uint16_t distance_get_avg(void) { return s_filt_avg; }

uint16_t distance_get_fail_count(void) { return s_fail_count; }

bool distance_is_online(void)
{
    return s_fail_count < DIST_FAIL_THRESHOLD;
}