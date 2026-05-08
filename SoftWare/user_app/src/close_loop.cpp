/*********************************************************************************************************************
 * 文件名称：close_loop.cpp
 * 功能描述：电机闭环控制实现
 *
 * 控制框图：
 *
 *   目标速度 ──► [误差] ──► [增量式PID] ──► [限幅] ──► Set_Motor_PWM
 *                  ▲                                          │
 *                  │                                          │
 *              编码器反馈 ◄────────────────────────────────────
 *
 * 调用顺序：
 *   1. close_loop_init()          → main() 里初始化一次
 *   2. close_loop_set_target()    → 需要改变速度时调用
 *   3. close_loop_update()        → 定时器回调里周期调用
 *   4. close_loop_stop()          → 系统退出或紧急停车时调用
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * 文件名称：close_loop.cpp
 * 功能描述：电机闭环控制实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 与原版本的关键差异：
 *
 *   1. 移除内部的 pid_reset() 私有函数，改用 pid.h 的 PID_Reset()
 *      （原代码因为 pid.h 没提供 reset 接口，只能在 close_loop.cpp 内部偷偷写一份）
 *
 *   2. PID 输出限幅由 ±2000（pid.cpp 默认值）改为 [0, 9900]，匹配 PWM_MAX
 *      原版本 PID 永远不可能输出超过 2000，相当于车子最多只能跑 20% 的速度
 *
 *   3. 增加死区控制：误差很小（< 0.5）时跳过 PID 计算
 *      避免编码器抖动导致的高频小幅 PWM 振荡
 *
 *   4. 增加堵转检测：PID 满输出但编码器没反应 → 100ms 内连续触发 → 紧急停车
 *      防止电机长时间堵转烧毁驱动板
 *
 *   5. 增加状态机：IDLE / RUNNING / STALLED / EMERGENCY
 *      上层可以查询当前状态做相应处理
 ********************************************************************************************************************/

#include "close_loop.h"

// ══════════════════════════════════════════════════════════════
// 内部状态变量
// ══════════════════════════════════════════════════════════════
static float s_left_target = 0.0f;
static float s_right_target = 0.0f;
static CloseLoopState_t s_state = CLOSE_LOOP_IDLE;

// 堵转检测计数器（左右轮独立）
static uint16_t s_stall_count_L = 0;
static uint16_t s_stall_count_R = 0;

// ══════════════════════════════════════════════════════════════
// 内部辅助：浮点数限幅
// ══════════════════════════════════════════════════════════════
static float clamp_float(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// ══════════════════════════════════════════════════════════════
// 闭环初始化
// ══════════════════════════════════════════════════════════════
void close_loop_init(void)
{
    // ── 1. 配置 PID 参数 ──────────────────────────────────────
    PID_Init(&LSpeed_PID, CL_DEFAULT_KP, CL_DEFAULT_KI, CL_DEFAULT_KD);
    PID_Init(&RSpeed_PID, CL_DEFAULT_KP, CL_DEFAULT_KI, CL_DEFAULT_KD);

    // ── 2. 配置 PID 输出限幅（匹配 PWM_MAX）────────────────
    // 这一步是关键修复：原版默认限幅 ±2000 会让 PWM 永远不超过 2000
    PID_SetOutputLimit(&LSpeed_PID, CL_PID_OUT_MIN, CL_PID_OUT_MAX);
    PID_SetOutputLimit(&RSpeed_PID, CL_PID_OUT_MIN, CL_PID_OUT_MAX);

    // ── 3. 可选：开启微分项滤波（默认关闭，需要时打开）─────
    // PID_SetDFilter(&LSpeed_PID, 0.7f);
    // PID_SetDFilter(&RSpeed_PID, 0.7f);

    // ── 4. 清零状态 ──────────────────────────────────────────
    s_left_target = 0.0f;
    s_right_target = 0.0f;
    s_stall_count_L = 0;
    s_stall_count_R = 0;
    s_state = CLOSE_LOOP_IDLE;

    // ── 5. 停止电机，确保初始静止 ────────────────────────────
    Set_Motor_PWM(0, 0);

    printf("[CloseLoop] Init done. Kp=%.1f Ki=%.1f Kd=%.1f Limit=[%.0f,%.0f]\n",
           CL_DEFAULT_KP, CL_DEFAULT_KI, CL_DEFAULT_KD,
           CL_PID_OUT_MIN, CL_PID_OUT_MAX);
}

// ══════════════════════════════════════════════════════════════
// 设置目标速度
// ══════════════════════════════════════════════════════════════
void close_loop_set_target(float left_speed, float right_speed)
{
    // 异常状态下不接受新目标，必须先 resume
    if (s_state == CLOSE_LOOP_STALLED || s_state == CLOSE_LOOP_EMERGENCY)
    {
        printf("[CloseLoop] Refused: in %s state, call resume() first.\n",
               close_loop_state_str());
        return;
    }

    // 限幅
    s_left_target = clamp_float(left_speed, 0.0f, (float)PWM_MAX);
    s_right_target = clamp_float(right_speed, 0.0f, (float)PWM_MAX);

    // 状态切换：有非零目标 → RUNNING；都是 0 → IDLE
    if (s_left_target > 0.0f || s_right_target > 0.0f)
        s_state = CLOSE_LOOP_RUNNING;
    else
        s_state = CLOSE_LOOP_IDLE;
}

// ══════════════════════════════════════════════════════════════
// 设置目标速度（差速形式）
// ══════════════════════════════════════════════════════════════
void close_loop_set_target_diff(float base, float diff)
{
    // base + diff 形式更接近转向直觉
    // diff > 0 → 左轮慢、右轮快 → 左转
    close_loop_set_target(base - diff, base + diff);
}

// ══════════════════════════════════════════════════════════════
// 闭环更新（定时器调用）
// ══════════════════════════════════════════════════════════════
void close_loop_update(void)
{
    // ── 守卫 1：系统已退出 ────────────────────────────────────
    if (!ls_system_running.load())
    {
        close_loop_stop();
        return;
    }

    // ── 守卫 2：异常状态 → 保持停车 ──────────────────────────
    if (s_state == CLOSE_LOOP_STALLED || s_state == CLOSE_LOOP_EMERGENCY)
    {
        Set_Motor_PWM(0, 0);
        return;
    }

    // ── 守卫 3：IDLE 状态 → 不输出 PWM（节能 + 防误动）──────
    if (s_state == CLOSE_LOOP_IDLE)
    {
        Set_Motor_PWM(0, 0);
        return;
    }

    // ── 步骤 1：读取编码器反馈 ───────────────────────────────
    float fb_L = encoder_L_final;
    float fb_R = encoder_R_final;

    // ── 步骤 2：计算误差 ─────────────────────────────────────
    float err_L = s_left_target - fb_L;
    float err_R = s_right_target - fb_R;

    // ── 步骤 3：应用死区（误差太小直接当 0）──────────────────
    // 这一步避免编码器小幅抖动引起 PWM 不停微调
    if (err_L > -CL_ERROR_DEADZONE && err_L < CL_ERROR_DEADZONE)
        err_L = 0.0f;
    if (err_R > -CL_ERROR_DEADZONE && err_R < CL_ERROR_DEADZONE)
        err_R = 0.0f;

    // ── 步骤 4：PID 计算 ─────────────────────────────────────
    float out_L = PID_IncCtrl(&LSpeed_PID, err_L);
    float out_R = PID_IncCtrl(&RSpeed_PID, err_R);

    // ── 步骤 5：输出限幅（PID 内部已限，这里再保险一次）──────
    int pwm_L = (int)clamp_float(out_L, 0.0f, (float)PWM_MAX);
    int pwm_R = (int)clamp_float(out_R, 0.0f, (float)PWM_MAX);

    Set_Motor_PWM(pwm_L, pwm_R);

    // ── 步骤 6：堵转检测 ─────────────────────────────────────
#if CL_STALL_DETECT_ENABLE
    // 条件：PID 输出已经很高，但编码器反馈仍然很低
    // 说明电机可能被卡住或线断了，不能再硬撑
    if (s_left_target > 0.0f &&
        out_L >= CL_STALL_PWM_THRESH &&
        fb_L <= CL_STALL_FEEDBACK_THRESH)
    {
        s_stall_count_L++;
    }
    else
    {
        s_stall_count_L = 0;
    }

    if (s_right_target > 0.0f &&
        out_R >= CL_STALL_PWM_THRESH &&
        fb_R <= CL_STALL_FEEDBACK_THRESH)
    {
        s_stall_count_R++;
    }
    else
    {
        s_stall_count_R = 0;
    }

    // 任一轮连续堵转超过阈值 → 紧急停车保护
    if (s_stall_count_L >= CL_STALL_COUNT_THRESH ||
        s_stall_count_R >= CL_STALL_COUNT_THRESH)
    {
        printf("[CloseLoop] !! STALL DETECTED !! L_cnt=%u R_cnt=%u\n",
               s_stall_count_L, s_stall_count_R);
        printf("[CloseLoop]    target(L=%.1f R=%.1f)  feedback(L=%.2f R=%.2f)  pwm(L=%d R=%d)\n",
               s_left_target, s_right_target, fb_L, fb_R, pwm_L, pwm_R);

        s_state = CLOSE_LOOP_STALLED;

        // 堵转后清零目标 + Reset PID + 立即停车
        s_left_target = 0.0f;
        s_right_target = 0.0f;
        PID_Reset(&LSpeed_PID);
        PID_Reset(&RSpeed_PID);
        Motor_Brake(); // 硬刹，避免继续给电流烧驱动
    }
#endif
}

// ══════════════════════════════════════════════════════════════
// 正常停止
// ══════════════════════════════════════════════════════════════
void close_loop_stop(void)
{
    s_left_target = 0.0f;
    s_right_target = 0.0f;
    s_stall_count_L = 0;
    s_stall_count_R = 0;

    PID_Reset(&LSpeed_PID);
    PID_Reset(&RSpeed_PID);

    Set_Motor_PWM(0, 0); // 滑行停车
    s_state = CLOSE_LOOP_IDLE;
}

// ══════════════════════════════════════════════════════════════
// 紧急停车（硬刹车）
// ══════════════════════════════════════════════════════════════
void close_loop_emergency_stop(void)
{
    s_left_target = 0.0f;
    s_right_target = 0.0f;
    s_stall_count_L = 0;
    s_stall_count_R = 0;

    PID_Reset(&LSpeed_PID);
    PID_Reset(&RSpeed_PID);

    Motor_Brake(); // 硬刹，比 Stop 更强力
    s_state = CLOSE_LOOP_EMERGENCY;

    printf("[CloseLoop] Emergency stop triggered.\n");
}

// ══════════════════════════════════════════════════════════════
// 从异常状态恢复
// ══════════════════════════════════════════════════════════════
void close_loop_resume(void)
{
    if (s_state != CLOSE_LOOP_STALLED && s_state != CLOSE_LOOP_EMERGENCY)
    {
        printf("[CloseLoop] resume() called in normal state, ignored.\n");
        return;
    }

    PID_Reset(&LSpeed_PID);
    PID_Reset(&RSpeed_PID);
    s_left_target = 0.0f;
    s_right_target = 0.0f;
    s_stall_count_L = 0;
    s_stall_count_R = 0;
    s_state = CLOSE_LOOP_IDLE;

    // 恢复电机方向到前进（紧急停车时 DIR 被拉低了）
    Motor_Stop();
    Motor_Forward(0); // 这会重置 DIR 到 FORWARD

    printf("[CloseLoop] Resumed. State = IDLE\n");
}

// ══════════════════════════════════════════════════════════════
// 查询接口
// ══════════════════════════════════════════════════════════════
float close_loop_get_left_target(void) { return s_left_target; }
float close_loop_get_right_target(void) { return s_right_target; }
CloseLoopState_t close_loop_get_state(void) { return s_state; }

const char *close_loop_state_str(void)
{
    switch (s_state)
    {
    case CLOSE_LOOP_IDLE:
        return "IDLE";
    case CLOSE_LOOP_RUNNING:
        return "RUNNING";
    case CLOSE_LOOP_STALLED:
        return "STALLED";
    case CLOSE_LOOP_EMERGENCY:
        return "EMERGENCY";
    default:
        return "UNKNOWN";
    }
}

// ══════════════════════════════════════════════════════════════
// 动态修改 PID 参数
// ══════════════════════════════════════════════════════════════
void close_loop_set_pid(float kp, float ki, float kd)
{
    LSpeed_PID.kp = kp;
    LSpeed_PID.ki = ki;
    LSpeed_PID.kd = kd;
    RSpeed_PID.kp = kp;
    RSpeed_PID.ki = ki;
    RSpeed_PID.kd = kd;

    // 修改参数后重置历史状态，避免新旧参数冲突
    PID_Reset(&LSpeed_PID);
    PID_Reset(&RSpeed_PID);

    printf("[CloseLoop] PID params updated: Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
}