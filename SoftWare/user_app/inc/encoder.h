#ifndef ENCODER_H
#define ENCODER_H

// #include "headfile.h"
#include "lq_reg_pwm_encoder.hpp"
#include <atomic> // <--- 加上这一行！！！
// 全局运行标志，供各模块读取停止/运行状态（在某个 .cpp 中定义）
extern std::atomic<bool> ls_system_running;

extern float encoderL_oringinal; // 编码器原始值
extern float encoderR_oringinal;

extern float encoder_L_final; // 左编码器值
extern float encoder_R_final; // 右编码器值

void encoder_Init(void);   // 编码器初始化
void encoder_update(void); // 编码器循环获取数据，放中断里执行

#endif // ENCODER_H
