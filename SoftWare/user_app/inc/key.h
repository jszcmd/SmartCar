#ifndef KEY_H
#define KEY_H

// 引入底层 GPIO 寄存器操作库和标准整型头文件
// #include "headfile.h"
#include "lq_reg_gpio.hpp"
#include <stdint.h>

// =====================================================
// 宏定义后备补充 (防止底层库未定义引发编译报错)
// =====================================================
#ifndef uint8
#define uint8 unsigned char
#endif

// =====================================================
// 硬件引脚映射配置
// =====================================================
// 对应智能车板子上的按键 K0, K1, K2
#define KEY1_PIN PIN_44 ///< 按键 1 (K0) 引脚
#define KEY2_PIN PIN_45 ///< 按键 2 (K1) 引脚
#define KEY3_PIN PIN_80 ///< 按键 3 (K2) 引脚

// 对应智能车板子上的拨码开关
#define KEY_BOMA1_PIN PIN_17 ///< 拨码开关 1 引脚
#define KEY_BOMA2_PIN PIN_20 ///< 拨码开关 2 引脚

// =====================================================
// 外部全局对象声明 (实体定义在 key.cpp 中)
// =====================================================
extern ls_gpio key1; ///< 按键 1 (K0) 硬件操作对象
extern ls_gpio key2; ///< 按键 2 (K1) 硬件操作对象
extern ls_gpio key3; ///< 按键 3 (K2) 硬件操作对象

// =====================================================
// 外部状态与标志位变量声明 (供其他业务代码读取)
// =====================================================
/** @name 当前按键电平状态 (通常 1:未按, 0:按下) */
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;

/** @name 上一次检测的按键电平状态 (用于边缘跳变检测) */
extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;

/** @name 按键触发事件标志位 (1:触发生效, 0:等待触发) */
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;

/** @brief 拨码开关组合模式值 (通常范围为 1 ~ 4，用于切换发车策略) */
extern uint8 key_mode;

// =====================================================
// 接口函数声明
// =====================================================
/**
 * @brief  按键循环检测与消抖函数
 * @note   【重要】必须放入定时器中断或系统滴答定时器中，建议每隔 10ms 调用一次。
 */
void key_loop(void);

/**
 * @brief  拨码开关状态检测函数
 * @note   根据两个拨码开关的组合状态更新全局变量 key_mode。
 */
void key_boma(void);

#endif // KEY_H
