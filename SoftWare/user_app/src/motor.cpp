#include "motor.h"
#include <iostream>

// 底层硬件对象的指针
static ls_atim_pwm *left_pwm_ctrl = nullptr;
static ls_atim_pwm *right_pwm_ctrl = nullptr;
static ls_gpio *left_dir_ctrl = nullptr;
static ls_gpio *right_dir_ctrl = nullptr;

/******************************************************************
 * @brief 电机初始化 (焊死在前进挡)
 ******************************************************************/
void Motor_Init(void)
{
    // 1. 初始化 PWM 通道
    left_pwm_ctrl = new ls_atim_pwm(LEFT_PWM_PIN, MOTOR_PWM_FREQ, 0, ATIM_PWM_POL_INV);
    right_pwm_ctrl = new ls_atim_pwm(RIGHT_PWM_PIN, MOTOR_PWM_FREQ, 0, ATIM_PWM_POL_INV);

    // 2. 初始化方向控制 GPIO
    left_dir_ctrl = new ls_gpio(LEFT_DIR_PIN, GPIO_MODE_OUT);
    right_dir_ctrl = new ls_gpio(RIGHT_DIR_PIN, GPIO_MODE_OUT);

    // 3. 【核心优化】初始化时直接焊死方向，永远前进！
    left_dir_ctrl->gpio_level_set(GPIO_HIGH); // 左轮前进
    right_dir_ctrl->gpio_level_set(GPIO_LOW); // 右轮前进

    std::cout << "[Motor] Initialization Complete (Forward Only Mode)!" << std::endl;
}

/******************************************************************
 * @brief 设置电机速度 (只踩油门)
 * @param left_pwm  左轮目标PWM (0 到 10000)
 * @param right_pwm 右轮目标PWM (0 到 10000)
 ******************************************************************/
void Set_Motor_PWM(int left_pwm, int right_pwm)
{
    // 1. 容错拦截：万一外环 PID 算出了负数，直接让它变成 0 (滑行或刹车)，绝不倒车
    if (left_pwm < 0)
        left_pwm = 0;
    if (right_pwm < 0)
        right_pwm = 0;

    // 2. 上限保护：防止 PWM 爆表
    if (left_pwm > PWM_MAX)
        left_pwm = PWM_MAX;
    if (right_pwm > PWM_MAX)
        right_pwm = PWM_MAX;

    // 3. 直接输出 PWM (只管速度，不管方向了)
    left_pwm_ctrl->atim_pwm_set_duty((uint32_t)left_pwm);
    right_pwm_ctrl->atim_pwm_set_duty((uint32_t)right_pwm);
}