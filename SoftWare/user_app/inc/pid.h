#ifndef __PID_H
#define __PID_H

// #include "headfile.h"

typedef struct
{
    float kp;   // P
    float ki;   // I
    float kd;   // D
    float imax; // 积分限幅

    float out_p; // KP输出
    float out_i; // KI输出
    float out_d; // KD输出
    float out;   // pid输出

    float integrator;      //< 积分值
    float last_error;      //< 上次误差
    float last_derivative; //< 上次误差与上上次误差之差
} PID_TypeDef;

extern PID_TypeDef LSpeed_PID;
extern PID_TypeDef RSpeed_PID;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd); // pid初始化
float PID_IncCtrl(PID_TypeDef *pid, float error);              // 增量式pid
float PID_PosCtrl(PID_TypeDef *pid, float error);              // 位置式pid

#endif