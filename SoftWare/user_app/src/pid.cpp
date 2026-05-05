#include "pid.h"

/** @brief 实例化左右轮的速度 PID 控制器对象 */
PID_TypeDef LSpeed_PID;
PID_TypeDef RSpeed_PID;

/**
 * @brief  PID 参数初始化
 * @param  pid 指向 PID 结构体的指针
 * @param  Kp  比例系数
 * @param  Ki  积分系数
 * @param  Kd  微分系数
 * @note   清空历史状态，防止单片机复位或重新启动控制时产生不可控的跳变。
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->kp = Kp;             // P 参数
    pid->ki = Ki;             // I 参数
    pid->kd = Kd;             // D 参数
    pid->imax = 0;            // 积分限幅 (需在外部赋值配置，0代表不限制)
    pid->integrator = 0;      // 误差积分值清零
    pid->out = 0;             // PID 总输出清零
    pid->out_d = 0;           // KD 输出清零
    pid->out_i = 0;           // KI 输出清零
    pid->out_p = 0;           // KP 输出清零
    pid->last_error = 0;      // 上次误差清零
    pid->last_derivative = 0; // 上次误差差值清零
}

/**
 * @brief  增量式 PID 控制算法
 * @param  pid   指向 PID 结构体的指针
 * @param  error 当前偏差 (目标值 - 实际值)
 * @retval 经过限幅的增量式输出值
 * @note   增量式 PID 输出的是控制量的“变化量”（如 PWM 的增减量），适合电机速度控制。
 * 公式：out += Kp * (e - last_e) + Ki * e + Kd * ((e - last_e) - last_derivative)
 */
float PID_IncCtrl(PID_TypeDef *pid, float error)
{
    // 计算本次偏差与上次偏差之差
    float derivative = error - pid->last_error;

    // 分别计算 P、I、D 三项的增量输出
    pid->out_p = pid->kp * derivative;                          // 比例项增量
    pid->out_i = pid->ki * error;                               // 积分项增量
    pid->out_d = pid->kd * (derivative - pid->last_derivative); // 微分项增量

    // 累加得到本次的控制总输出
    pid->out += pid->out_p + pid->out_i + pid->out_d;

    // 输出限幅 (防止 PWM 占空比等控制量超调)
    if (pid->out > 2000)
        pid->out = 2000;
    else if (pid->out < -2000)
        pid->out = -2000;

    // 保存本次状态，供下一次计算使用
    pid->last_error = error;
    pid->last_derivative = derivative;

    return pid->out;
}

/**
 * @brief  位置式 PID 控制算法
 * @param  pid   指向 PID 结构体的指针
 * @param  error 当前偏差 (目标值 - 实际值)
 * @retval 经过限幅的位置式输出值
 * @note   位置式 PID 直接输出控制量的值（如舵机打角），适合舵机转向控制。
 * 公式：out = Kp * e + Ki * (∑e) + Kd * (e - last_e)
 */
float PID_PosCtrl(PID_TypeDef *pid, float error)
{
    // 积分项累加
    pid->integrator += error;

    // 积分抗饱和 (Anti-windup) 处理：防止误差长时间累积导致积分器爆炸
    if (pid->imax > 0) // 确保已经配置了限幅值
    {
        if (pid->integrator > pid->imax)
            pid->integrator = pid->imax;
        else if (pid->integrator < -pid->imax)
            pid->integrator = -pid->imax;
    }

    // 分别计算 P、I、D 三项的绝对输出
    pid->out_p = pid->kp * error;                     // 比例项
    pid->out_i = pid->ki * pid->integrator;           // 积分项
    pid->out_d = pid->kd * (error - pid->last_error); // 微分项

    // 计算总输出
    pid->out = pid->out_p + pid->out_i + pid->out_d;

    // 输出限幅 (必须在最终结果计算完之后进行限幅才有效)
    if (pid->out > 2000)
        pid->out = 2000;
    else if (pid->out < -2000)
        pid->out = -2000;

    // 保存本次误差，供下一次计算微分项使用
    pid->last_error = error;

    return pid->out;
}