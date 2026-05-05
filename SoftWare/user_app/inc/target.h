#ifndef TARGET_H
#define TARGET_H

#include "headfile.h"

enum target_state_e
{
    TARGET_NONE = 0,    // 无目标
    TARGET_FOUND,       // 发现红条,正在分类
    TARGET_AVOID_LEFT,  // 分类结果: 武器 -> 左绕
    TARGET_AVOID_RIGHT, // 分类结果: 物资 -> 右绕
    TARGET_STRAIGHT,    // 分类结果: 交通工具 -> 直行
    TARGET_PASSING,     // 正在执行绕行中(盲跑或编码器计数)
    TARGET_DONE         // 完成
};

extern enum target_state_e target_state;
void check_target_logic(void);

#endif // TARGET_H