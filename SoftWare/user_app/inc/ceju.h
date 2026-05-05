#ifndef CEJU_H
#define CEJU_H

// #include "headfile.h"
#include <lq_i2c_vl53l0x.hpp>
#include <lq_i2c_mpu6050.hpp>

void ceju_loop(void);    // 测距模块任务函数
void mpu6050_loop(void); // 陀螺仪模块任务函数

#endif // CEJU_H
