/*********************************************************************************************************************
 * 文件名称：pid.cpp
 * 功能描述：通用 PID 控制器实现
 *
 * 算法原理回顾：
 *   ┌────────────────────────────────────────────────────────┐
 *   │  连续域 PID:  u(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt    │
 *   ├────────────────────────────────────────────────────────┤
 *   │  位置式（直接离散化）:                                  │
 *   │  u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k)-e(k-1)]         │
 *   ├────────────────────────────────────────────────────────┤
 *   │  增量式（前后两次相减）:                                │
 *   │  Δu(k) = Kp·[e(k)-e(k-1)]                             │
 *   │       + Ki·e(k)                                        │
 *   │       + Kd·[e(k)-2e(k-1)+e(k-2)]                       │
 *   └────────────────────────────────────────────────────────┘
 ********************************************************************************************************************/

#include "pid.h"

// ══════════════════════════════════════════════════════════════
// 全局实例定义（声明在 pid.h）
// ══════════════════════════════════════════════════════════════
PID_TypeDef LSpeed_PID;
PID_TypeDef RSpeed_PID;

// ══════════════════════════════════════════════════════════════
// 内部辅助：浮点数限幅
// ══════════════════════════════════════════════════════════════
static inline float pid_clamp(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// ══════════════════════════════════════════════════════════════
// PID 初始化
// ══════════════════════════════════════════════════════════════
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    if (pid == nullptr)
        return;

    // 三个核心参数
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;

    // 默认限幅：足够宽松，避免误限制
    // 使用方应通过 PID_SetOutputLimit() 覆盖
    pid->out_max = 10000.0f;
    pid->out_min = -10000.0f;
    pid->imax = 0.0f; // 0 = 积分不限制（位置式建议手动配置）

    // 高级特性默认关闭
    pid->i_separate_thresh = 0.0f; // 0 = 禁用积分分离
    pid->d_filter_alpha = 0.0f;    // 0 = 不滤波

    // 清空所有运行时状态
    PID_Reset(pid);
}

// ══════════════════════════════════════════════════════════════
// PID 状态重置（保留参数，清空过程量）
// ══════════════════════════════════════════════════════════════
void PID_Reset(PID_TypeDef *pid)
{
    if (pid == nullptr)
        return;

    pid->out = 0.0f;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;

    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->last_derivative = 0.0f;
    pid->filtered_d = 0.0f;
}

// ══════════════════════════════════════════════════════════════
// 配置接口
// ══════════════════════════════════════════════════════════════
void PID_SetOutputLimit(PID_TypeDef *pid, float out_min, float out_max)
{
    if (pid == nullptr)
        return;
    if (out_min > out_max)
        return; // 参数非法直接忽略，避免限幅错乱
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetISeparate(PID_TypeDef *pid, float thresh)
{
    if (pid == nullptr)
        return;
    pid->i_separate_thresh = (thresh < 0.0f) ? 0.0f : thresh;
}

void PID_SetDFilter(PID_TypeDef *pid, float alpha)
{
    if (pid == nullptr)
        return;
    // 限制在 [0, 1) 范围，1 会导致永远不更新
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 0.95f)
        alpha = 0.95f;
    pid->d_filter_alpha = alpha;
}

// ══════════════════════════════════════════════════════════════
// 增量式 PID
// 公式： Δu = Kp·(e - e₁) + Ki·e + Kd·[(e - e₁) - (e₁ - e₂)]
//        u  = u + Δu
// ══════════════════════════════════════════════════════════════
float PID_IncCtrl(PID_TypeDef *pid, float error)
{
    if (pid == nullptr)
        return 0.0f;

    // ── 1. 计算"误差变化率"（一阶差分）───────────────────────
    float derivative = error - pid->last_error;

    // ── 2. P 分量 = Kp × 误差变化率 ──────────────────────────
    pid->out_p = pid->kp * derivative;

    // ── 3. I 分量 = Ki × 当前误差（含积分分离）──────────────
    // 积分分离：误差太大时（远离目标），不积分
    // 这样可以避免大幅超调（积分一旦累起来就来不及刹车）
    if (pid->i_separate_thresh > 0.0f &&
        (error > pid->i_separate_thresh || error < -pid->i_separate_thresh))
    {
        pid->out_i = 0.0f; // 误差超阈值，本周期不贡献积分
    }
    else
    {
        pid->out_i = pid->ki * error;
    }

    // ── 4. D 分量 = Kd × 二阶差分（变化率的变化率）──────────
    // 增量式 PID 的 D 项天然是二阶差分，对噪声放大严重
    // 这里加一阶低通滤波平滑
    float raw_d = pid->kd * (derivative - pid->last_derivative);
    if (pid->d_filter_alpha > 0.0f)
    {
        // y[n] = α·y[n-1] + (1-α)·x[n]
        pid->filtered_d = pid->d_filter_alpha * pid->filtered_d + (1.0f - pid->d_filter_alpha) * raw_d;
        pid->out_d = pid->filtered_d;
    }
    else
    {
        pid->out_d = raw_d;
    }

    // ── 5. 累加到总输出 ──────────────────────────────────────
    pid->out += pid->out_p + pid->out_i + pid->out_d;

    // ── 6. 输出限幅 ──────────────────────────────────────────
    pid->out = pid_clamp(pid->out, pid->out_min, pid->out_max);

    // ── 7. 保存历史状态 ──────────────────────────────────────
    pid->last_error = error;
    pid->last_derivative = derivative;

    return pid->out;
}

// ══════════════════════════════════════════════════════════════
// 位置式 PID
// 公式： u = Kp·e + Ki·∑e + Kd·(e - e₁)
// ══════════════════════════════════════════════════════════════
float PID_PosCtrl(PID_TypeDef *pid, float error)
{
    if (pid == nullptr)
        return 0.0f;

    // ── 1. 积分累加（含积分分离）─────────────────────────────
    if (pid->i_separate_thresh > 0.0f &&
        (error > pid->i_separate_thresh || error < -pid->i_separate_thresh))
    {
        // 误差太大，不积分（但也不清零，保留之前的累积）
    }
    else
    {
        pid->integrator += error;
    }

    // ── 2. 积分抗饱和（Anti-windup）──────────────────────────
    // 积分项无限累积会让 I 分量越来越大，最后 PID 失控
    // 这里硬限幅是最简单的抗饱和方式
    if (pid->imax > 0.0f)
    {
        pid->integrator = pid_clamp(pid->integrator, -pid->imax, pid->imax);
    }

    // ── 3. 三项分量 ──────────────────────────────────────────
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;

    // 微分项：一阶差分 + 可选低通滤波
    float raw_d = pid->kd * (error - pid->last_error);
    if (pid->d_filter_alpha > 0.0f)
    {
        pid->filtered_d = pid->d_filter_alpha * pid->filtered_d + (1.0f - pid->d_filter_alpha) * raw_d;
        pid->out_d = pid->filtered_d;
    }
    else
    {
        pid->out_d = raw_d;
    }

    // ── 4. 总输出 + 限幅 ─────────────────────────────────────
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    pid->out = pid_clamp(pid->out, pid->out_min, pid->out_max);

    // ── 5. 保存历史误差 ──────────────────────────────────────
    pid->last_error = error;

    return pid->out;
}