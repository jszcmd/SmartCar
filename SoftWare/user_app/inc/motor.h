#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "lq_reg_atim_pwm.hpp"
#include "lq_reg_gpio.hpp"

// ================= 引脚与硬件配置区 =================
// 完全同步你提供的真实引脚
#define LEFT_PWM_PIN ATIM_PWM0_PIN81 // 左轮PWM引脚
#define LEFT_DIR_PIN PIN_21          // 左轮方向控制

#define RIGHT_PWM_PIN ATIM_PWM1_PIN82 // 右轮PWM引脚
#define RIGHT_DIR_PIN PIN_22          // 右轮方向控制

// PWM 基础配置
#define MOTOR_PWM_FREQ 10000 // PWM频率 10kHz
#define PWM_MAX 9900         // PWM最大值

// ================= 对外接口声明 =================

// 电机初始化函数
void Motor_Init(void);

// 电机速度与方向设置函数
// left_pwm / right_pwm 范围： -10000 到 10000
void Set_Motor_PWM(int left_pwm, int right_pwm);

#endif // __MOTOR_HPP