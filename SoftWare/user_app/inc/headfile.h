#ifndef __HEADFILE_H__
#define __HEADFILE_H__
#include <iostream>
#include <opencv2/opencv.hpp>

#include <atomic> //全局运行标志

#include <math.h>
#include <stdarg.h>
#include <stdio.h> // 用于 sprintf

#define uint8 unsigned char
// ====================== 1. 宏定义 ======================
#ifndef UVC_WIDTH
#define UVC_WIDTH 320
#endif
#ifndef UVC_HEIGHT
#define UVC_HEIGHT 240
#endif

#define MT9V03X_W 160  // 宽度
#define MT9V03X_H 120  // 高度
#define MT9V03X_HH 120 // 边线数组最大长度

// 参数宏
#define BEGIN_X 10
#define BEGIN_Y 100
#define RIGHT_ANGLE_THRES_MAX 1.00f
#define RIGHT_ANGLE_THRES_MIN 0.85f

// ====================== 2. 结构体定义 ======================
typedef struct image
{
    const uint8 *data; // 图像数据指针
    uint8 width;       // 图像宽度
    uint8 height;      // 图像高度
    uint8 step;        // 行步长
} image_t;

// 声明参数变量
extern int block_size;
extern int clip_value;
extern float sample_dist;
extern int pixel_per_meter;
extern float angle_dist;
extern int line_blur_kernel;
extern int nms_kernel;
extern float ROAD_WIDTH;

extern int rpts0s_num, rpts1s_num;         // 左右边线点数
extern float rpts0s[MT9V03X_HH][2];        // 左边线坐标
extern float rpts1s[MT9V03X_HH][2];        // 右边线坐标
extern int Lpt0_found, Lpt1_found;         // 左右L角点标志
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id; // 左右L角点索引
extern bool is_straight0, is_straight1;    // 左右长直道标志

extern int target_red_x;
extern int target_red_y;
extern int target_red_w;
extern int target_red_h;

extern bool debug_target_found;
// === 暴露给 image.cpp 的红色标志物真实坐标 ===
extern int target_red_x;
extern int target_red_y;
extern int target_red_w;
extern int target_red_h;

extern int target_override_track_type;
extern float target_shift_offset;

// 全局运行标志，供各模块读取停止/运行状态（在某个 .cpp 中定义）
extern std::atomic<bool> ls_system_running;

#include "image.h"
#include "cross.h"
#include "circle.h"
#include "openCV_find_red.h"
#include "target.h"

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// 我把下面这些都放入到 main文件夹下面的main.hpp里面了。
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

// #include "WW_TimerTasks.h"        //三路周期定时任务
// #include "wireless_wave.h"        //无线波形
// #include "motor.h"                //电机
// #include "lq_reg_pwm_encoder.hpp" //编码器PWM类
// #include "encoder.h"              //编码器
// #include "pid.h"                  //pid控制(包括增量式和位置式)
// #include "key.h"                  //按键
// #include "ceju.h"                 //测距模块+mpu6050(均为初始数据)
// #include "tft18.h"                //TFT18显示函数
// #include "close_loop.h" //闭环控制

extern cv::Mat detect;
extern cv::Mat img_nitoushi; // 逆透视的图像。
extern float Guide;          // errors

#endif // __HEADFILE_H__
