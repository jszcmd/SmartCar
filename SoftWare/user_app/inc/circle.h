#ifndef __circle_h__
#define __circle_h__

#include "headfile.h"

enum circle_type_e
{
    CIRCLE_NONE = 0,
    CIRCLE_LEFT_BEGIN,    // 发现左环特征
    CIRCLE_RIGHT_BEGIN,   // 发现右环特征
    CIRCLE_LEFT_IN,       // 准备入左环
    CIRCLE_RIGHT_IN,      // 准备入右环
    CIRCLE_LEFT_RUNNING,  // 左环内巡线
    CIRCLE_RIGHT_RUNNING, // 右环内巡线
    CIRCLE_LEFT_OUT,      // 准备出左环
    CIRCLE_RIGHT_OUT,     // 准备出右环
    CIRCLE_LEFT_END,      // 出环结束
    CIRCLE_RIGHT_END      // 出环结束
};

extern enum circle_type_e circle_type;
extern int track_type; // 0=混合模式, 1=强制左线, 2=强制右线

void check_circle(void);
void run_circle(void);

#endif // __circle_h__