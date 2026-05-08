/*********************************************************************************************************************
 * 文件名称：motor.cpp
 * 功能描述：电机驱动控制模块实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 内部状态变量说明：
 *   s_left_pwm / s_right_pwm   ← 当前实际输出到硬件的 PWM 值
 *   s_left_target / s_right_target ← 业务层设定的目标值
 *   s_accel_limit              ← 加速度限制（每周期最大变化量）
 *
 *   未启用加速度限制时（accel_limit=0）：
 *     Set_Motor_PWM 直接修改 s_left/right_pwm 并写入硬件
 *
 *   启用加速度限制时（accel_limit>0）：
 *     Set_Motor_PWM 只更新 target，硬件输出由 Motor_UpdateRamp 平滑过渡
 *
 * ─────────────────────────────────────────────────────────────────────
 * 方向控制说明（重要）：
 *   左轮和右轮的 DIR 电平是反的（看初始化代码）：
 *     左轮前进：LEFT_DIR  = HIGH
 *     右轮前进：RIGHT_DIR = LOW
 *   原因：两个电机物理装配方向相反，要让车子直走必须设置为相反电平
 *   想倒车时把这两个电平都翻转即可
 ********************************************************************************************************************/

#include "motor.h"
#include <stdio.h>
#include <unistd.h>

// ══════════════════════════════════════════════════════════════
// 硬件对象指针（文件作用域）
// ══════════════════════════════════════════════════════════════
static ls_atim_pwm *left_pwm_ctrl = nullptr;
static ls_atim_pwm *right_pwm_ctrl = nullptr;
static ls_gpio *left_dir_ctrl = nullptr;
static ls_gpio *right_dir_ctrl = nullptr;

// ══════════════════════════════════════════════════════════════
// 内部状态变量
// ══════════════════════════════════════════════════════════════
static int s_left_pwm = 0; // 当前实际输出的 PWM
static int s_right_pwm = 0;
static int s_left_target = 0; // 目标 PWM（用于加速度限制）
static int s_right_target = 0;
static int s_accel_limit = MOTOR_ACCEL_LIMIT_DEFAULT; // 加速度限制
static MotorDir_t s_direction = MOTOR_DIR_FORWARD;    // 当前方向

// ══════════════════════════════════════════════════════════════
// 内部辅助函数
// ══════════════════════════════════════════════════════════════
static int clamp_int(int val, int min_val, int max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

/*******************************************************************
 * @brief   底层方向设置（内部函数，不对外暴露）
 *          根据 dir 设置左右轮 DIR 引脚
 ******************************************************************/
static void motor_apply_direction(MotorDir_t dir)
{
    if (left_dir_ctrl == nullptr || right_dir_ctrl == nullptr)
        return;

    switch (dir)
    {
    case MOTOR_DIR_FORWARD:
        // 前进：左HIGH，右LOW（两个电机装配方向相反）
        left_dir_ctrl->gpio_level_set(GPIO_HIGH);
        right_dir_ctrl->gpio_level_set(GPIO_LOW);
        break;

    case MOTOR_DIR_REVERSE:
        // 反转（预留接口，当前业务不会触发）
        left_dir_ctrl->gpio_level_set(GPIO_LOW);
        right_dir_ctrl->gpio_level_set(GPIO_HIGH);
        break;

    case MOTOR_DIR_BRAKE:
        // 刹车：两端都拉低，电机两端短接形成电磁阻尼
        left_dir_ctrl->gpio_level_set(GPIO_LOW);
        right_dir_ctrl->gpio_level_set(GPIO_LOW);
        break;
    }
    s_direction = dir;
}

/*******************************************************************
 * @brief   底层硬件输出（内部函数）
 *          只负责把 PWM 值写到硬件，不做任何加工
 ******************************************************************/
static void motor_write_hardware(int left_pwm, int right_pwm)
{
    if (left_pwm_ctrl == nullptr || right_pwm_ctrl == nullptr)
        return;

    left_pwm_ctrl->atim_pwm_set_duty((uint32_t)left_pwm);
    right_pwm_ctrl->atim_pwm_set_duty((uint32_t)right_pwm);

    s_left_pwm = left_pwm;
    s_right_pwm = right_pwm;
}

// ══════════════════════════════════════════════════════════════
// 电机初始化
// ══════════════════════════════════════════════════════════════
void Motor_Init(void)
{
    if (left_pwm_ctrl != nullptr)
        return; // 防止重复初始化

    // ── 初始化 PWM 通道（占空比 0，极性反转）──────────────────
    left_pwm_ctrl = new ls_atim_pwm(LEFT_PWM_PIN, MOTOR_PWM_FREQ, 0, ATIM_PWM_POL_INV);
    right_pwm_ctrl = new ls_atim_pwm(RIGHT_PWM_PIN, MOTOR_PWM_FREQ, 0, ATIM_PWM_POL_INV);

    // ── 初始化方向控制 GPIO ───────────────────────────────────
    left_dir_ctrl = new ls_gpio(LEFT_DIR_PIN, GPIO_MODE_OUT);
    right_dir_ctrl = new ls_gpio(RIGHT_DIR_PIN, GPIO_MODE_OUT);

    // ── 默认方向：前进 ────────────────────────────────────────
    motor_apply_direction(MOTOR_DIR_FORWARD);

    // ── 启动时确保电机静止 ────────────────────────────────────
    motor_write_hardware(0, 0);
    s_left_target = 0;
    s_right_target = 0;
    s_accel_limit = MOTOR_ACCEL_LIMIT_DEFAULT;

    printf("[Motor] Init done. PWM_FREQ=%d PWM_MAX=%d DIR=FORWARD\n",
           MOTOR_PWM_FREQ, PWM_MAX);
}

// ══════════════════════════════════════════════════════════════
// 设置左右电机 PWM（无符号，仅前进）
// ══════════════════════════════════════════════════════════════
void Set_Motor_PWM(int left_pwm, int right_pwm)
{
    if (left_pwm_ctrl == nullptr)
        return;

    // 限幅：负值截断为 0，超出 PWM_MAX 截断到上限
    left_pwm = clamp_int(left_pwm, PWM_MIN, PWM_MAX);
    right_pwm = clamp_int(right_pwm, PWM_MIN, PWM_MAX);

    if (s_accel_limit <= 0)
    {
        // 未启用加速度限制：直接输出
        motor_write_hardware(left_pwm, right_pwm);
        s_left_target = left_pwm;
        s_right_target = right_pwm;
    }
    else
    {
        // 启用加速度限制：只更新目标值，由 Motor_UpdateRamp 平滑过渡
        s_left_target = left_pwm;
        s_right_target = right_pwm;
    }
}

// ══════════════════════════════════════════════════════════════
// 设置左右电机 PWM（有符号，预留倒车接口）
// ══════════════════════════════════════════════════════════════
void Set_Motor_PWM_Signed(int left_pwm, int right_pwm)
{
    // ─────────────────────────────────────────────────────────
    // 当前策略：保持只前进，负值截断为 0
    // 未来要支持倒车，把下面这段改成：
    //   if (left_pwm < 0 && right_pwm < 0) {
    //       motor_apply_direction(MOTOR_DIR_REVERSE);
    //       Set_Motor_PWM(-left_pwm, -right_pwm);
    //   } else if (left_pwm >= 0 && right_pwm >= 0) {
    //       motor_apply_direction(MOTOR_DIR_FORWARD);
    //       Set_Motor_PWM(left_pwm, right_pwm);
    //   } else {
    //       // 一正一负：原地转向，需要分轮独立控制方向（硬件支持时）
    //   }
    // ─────────────────────────────────────────────────────────
    if (left_pwm < 0)
        left_pwm = 0;
    if (right_pwm < 0)
        right_pwm = 0;
    Set_Motor_PWM(left_pwm, right_pwm);
}

// ══════════════════════════════════════════════════════════════
// 立即停止（PWM 清零，电机滑行）
// ══════════════════════════════════════════════════════════════
void Motor_Stop(void)
{
    s_left_target = 0;
    s_right_target = 0;
    motor_write_hardware(0, 0);
    printf("[Motor] Stop (coast)\n");
}

// ══════════════════════════════════════════════════════════════
// 硬刹车（DIR 拉低 + PWM=0，电磁刹车）
// ══════════════════════════════════════════════════════════════
void Motor_Brake(void)
{
    // 先停 PWM，再翻转方向引脚形成短接刹车
    motor_write_hardware(0, 0);
    motor_apply_direction(MOTOR_DIR_BRAKE);
    s_left_target = 0;
    s_right_target = 0;
    printf("[Motor] Brake (electric brake)\n");
}

// ══════════════════════════════════════════════════════════════
// 直行
// ══════════════════════════════════════════════════════════════
void Motor_Forward(int speed)
{
    speed = clamp_int(speed, PWM_MIN, PWM_MAX);

    // 如果之前是刹车状态，恢复前进方向
    if (s_direction != MOTOR_DIR_FORWARD)
        motor_apply_direction(MOTOR_DIR_FORWARD);

    Set_Motor_PWM(speed, speed);
}

// ══════════════════════════════════════════════════════════════
// 差速转向
// diff > 0 → 左轮慢、右轮快 → 左转
// diff < 0 → 左轮快、右轮慢 → 右转
// ══════════════════════════════════════════════════════════════
void Motor_Turn(int base_speed, int diff)
{
    if (s_direction != MOTOR_DIR_FORWARD)
        motor_apply_direction(MOTOR_DIR_FORWARD);

    int left = base_speed - diff;
    int right = base_speed + diff;
    Set_Motor_PWM(left, right);
}

// ══════════════════════════════════════════════════════════════
// 软启动（阻塞式平滑加速）
// ══════════════════════════════════════════════════════════════
void Motor_SoftStart(int target_speed)
{
    target_speed = clamp_int(target_speed, PWM_MIN, PWM_MAX);

    if (s_direction != MOTOR_DIR_FORWARD)
        motor_apply_direction(MOTOR_DIR_FORWARD);

    // 取当前两轮 PWM 的较大值作为起点（如果两轮不一致，从大的那个开始）
    int current = (s_left_pwm > s_right_pwm) ? s_left_pwm : s_right_pwm;

    printf("[Motor] SoftStart: %d → %d\n", current, target_speed);

    if (current >= target_speed)
    {
        // 反向（需要减速到目标）
        for (int v = current; v > target_speed; v -= MOTOR_SOFT_STEP)
        {
            motor_write_hardware(v, v);
            usleep(MOTOR_SOFT_INTERVAL_US);
        }
    }
    else
    {
        // 正向加速
        for (int v = current; v < target_speed; v += MOTOR_SOFT_STEP)
        {
            motor_write_hardware(v, v);
            usleep(MOTOR_SOFT_INTERVAL_US);
        }
    }

    // 最后精确到目标值
    motor_write_hardware(target_speed, target_speed);
    s_left_target = target_speed;
    s_right_target = target_speed;
}

// ══════════════════════════════════════════════════════════════
// 软停止（阻塞式平滑减速到 0）
// ══════════════════════════════════════════════════════════════
void Motor_SoftStop(void)
{
    int current = (s_left_pwm > s_right_pwm) ? s_left_pwm : s_right_pwm;

    printf("[Motor] SoftStop: %d → 0\n", current);

    for (int v = current; v > 0; v -= MOTOR_SOFT_STEP)
    {
        motor_write_hardware(v, v);
        usleep(MOTOR_SOFT_INTERVAL_US);
    }
    motor_write_hardware(0, 0);
    s_left_target = 0;
    s_right_target = 0;
}

// ══════════════════════════════════════════════════════════════
// 加速度限制配置
// ══════════════════════════════════════════════════════════════
void Motor_SetAccelLimit(int limit)
{
    if (limit < 0)
        limit = 0;
    s_accel_limit = limit;
    printf("[Motor] Accel limit set to %d (0=unlimited)\n", limit);
}

// ══════════════════════════════════════════════════════════════
// 加速度限制更新（非阻塞，定时器周期调用）
// ══════════════════════════════════════════════════════════════
void Motor_UpdateRamp(void)
{
    if (s_accel_limit <= 0)
        return; // 未启用，直接退出

    int new_left = s_left_pwm;
    int new_right = s_right_pwm;

    // ── 左轮逐步逼近目标 ──────────────────────────────────────
    int delta_l = s_left_target - s_left_pwm;
    if (delta_l > s_accel_limit)
        new_left = s_left_pwm + s_accel_limit;
    else if (delta_l < -s_accel_limit)
        new_left = s_left_pwm - s_accel_limit;
    else
        new_left = s_left_target;

    // ── 右轮逐步逼近目标 ──────────────────────────────────────
    int delta_r = s_right_target - s_right_pwm;
    if (delta_r > s_accel_limit)
        new_right = s_right_pwm + s_accel_limit;
    else if (delta_r < -s_accel_limit)
        new_right = s_right_pwm - s_accel_limit;
    else
        new_right = s_right_target;

    // ── 限幅后输出 ────────────────────────────────────────────
    new_left = clamp_int(new_left, PWM_MIN, PWM_MAX);
    new_right = clamp_int(new_right, PWM_MIN, PWM_MAX);
    motor_write_hardware(new_left, new_right);
}

// ══════════════════════════════════════════════════════════════
// 状态读取接口
// ══════════════════════════════════════════════════════════════
int Motor_Get_Left_PWM(void) { return s_left_pwm; }
int Motor_Get_Right_PWM(void) { return s_right_pwm; }
MotorDir_t Motor_Get_Direction(void) { return s_direction; }