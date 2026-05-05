#ifndef __CROSS_H__
#define __CROSS_H__

#include "headfile.h"

// 定义状态枚举
enum cross_type_e
{
    CROSS_NONE = 0,
    CROSS_BEGIN,
    CROSS_IN,
    CROSS_HALF,
    CROSS_HALF_LEFT,
    CROSS_HALF_RIGHT,
    CROSS_NUM
};

// 外部变量声明
extern enum cross_type_e cross_type;
extern int far_Lpt0_found, far_Lpt1_found;
extern float far_rpts0s[MT9V03X_HH][2];
extern float far_rpts1s[MT9V03X_HH][2];
extern int far_rpts0s_num, far_rpts1s_num;
extern float dn;
extern int start0_x, start0_y, start1_x, start1_y;
extern int far_Lpt0_rpts0s_id;
extern float conf;
// 盲补线激活标志位
extern bool blind_patch_active;
// 暴露远线角度数组供显示使用
extern float far_rpts0an[MT9V03X_HH];
extern float far_rpts1an[MT9V03X_HH];

// 调试变量声明
extern float debug_patch_start_x, debug_patch_start_y; // 补线起点
extern float debug_patch_end_x, debug_patch_end_y;     // 补线终点
extern float debug_patch_dist;                         // 补线长度
extern int debug_far_Lpt_id;                           // 锁定的远角点ID

// 暴露给屏幕显示详细状态的字符串
extern char cross_state_info[64];

extern int CBH_THRESHOLD_FAR;
int calc_contrast_cross(image_t *img, int x, int y); // 暴露对比度计算函数供整定使用
// 核心函数声明
void check_cross(void);
void run_cross(void);

// 辅助函数声明
void cross_farline(void);
void cross_farline_L(void);
void cross_farline_R(void);

#endif