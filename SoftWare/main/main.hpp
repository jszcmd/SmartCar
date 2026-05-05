#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <stdio.h>

// 包含所有底层驱动头文件
#include "lq_drv_inc.hpp"

// 包含所有工具头文件
#include "lq_common.hpp"

// 包含所有应用层头文件
#include "lq_app_inc.hpp"

// 包含所有测试程序头文件
#include "lq_all_demo.hpp"

// =======================================================================================
// 开源的图像传递，定时器，以及定时器的任务。

// 定时器的函数。
#include "WW_TimerThread.h"
// 向上位器电脑图传的头文件。
#include "WW_transmission.h"
// 图像摄像头传递的头文件。
#include "WW_CAMERA.h"
// 定时器任务的头文件。
#include "WW_TimerTasks.h"

// =======================================================================================

// #######################################################################################
// 图像处理的头文件。
#include "headfile.h" // 图像的所有的头文件
#include "circle.h"   // 圆环
#include "cross.h"    // 10字路口
#include "image.h"    // 图像处理
#include "target.h"
#include "openCV_find_red.h"

// #######################################################################################

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// 图像误差
#include "img_process.h"
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

// 蜂鸣器的头文件
#include "buzzer.h"
// 按键的头文件,拨码按键和独立按键.
#include "key.h"
// pid的头文件，增量式PID和位置式PID
#include "pid.h"
// tft18的屏幕显示的头文件,显示参数。
#include "tft18.h"
// 编码器的头文件。
#include "encoder.h"
// 电机驱动的头文件。
#include "motor.h"
// 无线波形数据传输的头文件。
#include "wireless_wave.h"

#endif