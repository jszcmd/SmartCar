#include "key.h"
#include <stdio.h> // 使用 printf 需要包含此头文件

// =====================================================
// 硬件对象实例化 (使用自定义的 ls_gpio 类)
// =====================================================
ls_gpio key1(KEY1_PIN, GPIO_MODE_IN);
ls_gpio key2(KEY2_PIN, GPIO_MODE_IN);
ls_gpio key3(KEY3_PIN, GPIO_MODE_IN);

ls_gpio key_boma1(KEY_BOMA1_PIN, GPIO_MODE_IN);
ls_gpio key_boma2(KEY_BOMA2_PIN, GPIO_MODE_IN);

// =====================================================
// 全局状态变量定义与初始化
// =====================================================
// 当前按键状态 (默认 1 为未按下/高电平)
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;

// 上一次检测时的按键状态 (用于边缘检测，务必初始化为 1，防止上电误触发)
uint8 key1_last_status = 1;
uint8 key2_last_status = 1;
uint8 key3_last_status = 1;

// 按键触发标志位 (1: 已触发, 0: 等待处理)
uint8 key1_flag = 0;
uint8 key2_flag = 0;
uint8 key3_flag = 0;

// 拨码开关组合模式 (范围: 1 ~ 4)
uint8 key_mode = 1;

/**
 * @brief  拨码开关状态检测
 * @note   读取两位拨码开关的状态，组合产生 4 种不同的模式。
 * 常用于智能车在上电前拨动开关，决定发车跑哪一套策略或哪个赛道。
 */
void key_boma(void)
{
    // 假设拨码开关打倒是 0(低电平)，未打倒是 1(高电平)
    if (key_boma1.gpio_level_get() == 0)
    {
        if (key_boma2.gpio_level_get() == 0)
        {
            key_mode = 3; // 00 状态 -> 模式 3
        }
        else
        {
            key_mode = 2; // 01 状态 -> 模式 2
        }
    }
    else
    {
        if (key_boma2.gpio_level_get() == 0)
        {
            key_mode = 4; // 10 状态 -> 模式 4
        }
        else
        {
            key_mode = 1; // 11 状态 -> 模式 1
        }
    }
}

/**
 * @brief  按键扫描主循环函数
 * @note   【重要】此函数需放入定时器中断中，每隔 10ms 调用一次。
 * 利用定时器的延时特性天然实现按键消抖。
 */
void key_loop(void)
{
    // 1. 保存上一次的按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;

    // 2. 读取当前的按键状态
    key1_status = key1.gpio_level_get();
    key2_status = key2.gpio_level_get();
    key3_status = key3.gpio_level_get();

    // 3. 边缘检测 (判断按键释放的瞬间)
    // 逻辑解析: 当前为 1(高电平松开)，上次为 0(低电平按下) -> 代表按键刚刚被松开
    if (key1_status && !key1_last_status)
        key1_flag = 1;
    if (key2_status && !key2_last_status)
        key2_flag = 1;
    if (key3_status && !key3_last_status)
        key3_flag = 1;

    // =====================================================
    // 4. 事件处理 (响应标志位)
    // =====================================================
    // 规范逻辑：只判断 flag 是否被置位，执行完毕后立即清零

    if (key1_flag == 1)
    {
        key1_flag = 0; // 使用按键之后，必须清除标志位防止重复执行

        printf("Key1 (K0) triggered!\n");
        // TODO: 这里放按键 1 被按下并松开后，想要执行的事件
    }

    if (key2_flag == 1)
    {
        key2_flag = 0;

        printf("Key2 (K1) triggered!\n");
        // TODO: 这里放按键 2 被按下并松开后，想要执行的事件
    }

    if (key3_flag == 1)
    {
        key3_flag = 0;

        printf("Key3 (K2) triggered!\n");
        // TODO: 这里放按键 3 被按下并松开后，想要执行的事件
    }
}