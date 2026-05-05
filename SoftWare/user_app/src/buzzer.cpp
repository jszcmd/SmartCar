#include "buzzer.h"

/**
 * @brief  蜂鸣器控制句柄指针
 * @note   使用静态全局指针保存 GPIO 对象地址，配合延迟初始化机制。
 * 初始化为 nullptr，防止野指针操作。
 */
static ls_gpio *pBuzzer = nullptr;

/**
 * @brief  蜂鸣器硬件初始化
 * @note   采用单例模式的思想,确保 GPIO 对象只被实例化一次。
 * 配置引脚号为 PIN_12（需根据实际电路板图纸核对）,工作模式为输出模式（GPIO_MODE_OUT）.
 */
void Buzzer_Init(void)
{
    // 检查是否已经初始化过，防止重复初始化
    if (pBuzzer == nullptr)
    {
        /**
         * @brief 定义局部静态 GPIO 对象
         * @note  静态局部变量的作用域仅在该函数内,但生命周期与整个程序一致,
         * 函数执行完毕后对象不会被销毁.
         */
        static ls_gpio instance(PIN_12, GPIO_MODE_OUT);

        // 将静态对象的地址赋给全局指针,供其他函数调用
        pBuzzer = &instance;
    }
}

/**
 * @brief  开启蜂鸣器 (高电平驱动)
 * @note   在操作硬件前会先调用初始化函数，确保硬件已准备就绪。
 * （注意：此处默认蜂鸣器为高电平触发，若是低电平触发发声，需更改此处电平逻辑）
 */
void buzzer_on(void)
{
    // 确保蜂鸣器已初始化（延迟初始化调用）
    Buzzer_Init();

    // 使用成员函数控制硬件，输出高电平
    pBuzzer->gpio_level_set(GPIO_HIGH);

    // 通过串口/终端输出调试状态信息
    printf("Buzzer status: ON\n");
}

/**
 * @brief  关闭蜂鸣器 (低电平停止)
 * @note   在操作硬件前同样会确保引脚已初始化完毕。
 */
void buzzer_off(void)
{
    // 确保蜂鸣器已初始化
    Buzzer_Init();

    // 使用成员函数控制硬件，输出低电平停止发声
    pBuzzer->gpio_level_set(GPIO_LOW);

    // 通过串口/终端输出调试状态信息
    printf("Buzzer status: OFF\n");
}