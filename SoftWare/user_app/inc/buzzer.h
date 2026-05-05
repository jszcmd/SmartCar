#ifndef __BUZZER_H_ // 条件编译，防止头文件被重复包含
#define __BUZZER_H_

// 引入底层驱动、应用层以及公共宏定义的头文件
#include "lq_reg_gpio.hpp"
#include <stdio.h>
#include <unistd.h>

// C++ 兼容性处理：如果是 C++ 编译器，则按照 C 语言的标准进行编译和链接
#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  蜂鸣器初始化
     * @note   用于配置蜂鸣器控制引脚的时钟、GPIO 模式（如推挽输出）以及初始电平状态
     */
    void Buzzer_Init(void);

    /**
     * @brief  开启蜂鸣器
     * @note   改变 GPIO 输出电平,使蜂鸣器发声
     */
    void buzzer_on(void);

    /**
     * @brief  关闭蜂鸣器
     * @note   恢复 GPIO 输出电平,使蜂鸣器停止发声
     */
    void buzzer_off(void);

#ifdef __cplusplus
}
#endif

#endif // __BUZZER_H_