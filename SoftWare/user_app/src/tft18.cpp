#include "tft18.h"

/**
 * @brief  TFT18 显示屏初始化
 * @note   配置屏幕显示方向并清屏。
 * 默认设置为：竖屏显示、绿色背景。
 */
void tft18_init(void)
{
    lq_tft18_drv_init(1);       // 初始化屏幕方向：0=横屏, 1=竖屏
    lq_tft18_drv_cls(U16GREEN); // 清屏并使用绿色背景 (U16GREEN 需在头文件中定义)
}

/********************************************************************************
 * @brief   TFT18 显示数据封装函数集
 * @note    支持各种基础数据类型转换为字符串并显示。
 * 底层调用 8x8 像素点阵的字符显示函数。
 * 默认字体颜色为黑色 (U16BLACK)，背景色为绿色 (U16GREEN)。
 ********************************************************************************/

/**
 * @brief  在指定坐标显示无符号 8 位整数 (uint8_t)
 * @param  x    横坐标 (列)
 * @param  y    纵坐标 (行)
 * @param  val  要显示的数值 (范围: 0 ~ 255)
 */
void tft18_uint8(uint8_t x, uint8_t y, uint8_t val)
{
    char buf[4]; // 最大值 "255" (3字符) + 字符串结束符 '\0' = 4字节
    snprintf(buf, sizeof(buf), "%u", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示无符号 16 位整数 (uint16_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值 (范围: 0 ~ 65535)
 */
void tft18_uint16(uint8_t x, uint8_t y, uint16_t val)
{
    char buf[6]; // 最大值 "65535" (5字符) + '\0' = 6字节
    snprintf(buf, sizeof(buf), "%u", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示无符号 32 位整数 (uint32_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值 (范围: 0 ~ 4294967295)
 */
void tft18_uint32(uint8_t x, uint8_t y, uint32_t val)
{
    char buf[11]; // 最大值 "4294967295" (10字符) + '\0' = 11字节
    snprintf(buf, sizeof(buf), "%u", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示无符号 64 位整数 (uint64_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值
 */
void tft18_uint64(uint8_t x, uint8_t y, uint64_t val)
{
    char buf[21]; // 最大值 20字符 + '\0' = 21字节
    // 注意：64位无符号整数标准格式化符为 %llu
    snprintf(buf, sizeof(buf), "%llu", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示有符号 8 位整数 (int8_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值 (范围: -128 ~ 127)
 */
void tft18_int8(uint8_t x, uint8_t y, int8_t val)
{
    char buf[5]; // 负号 "-" + 最大值 "128" (3字符) + '\0' = 5字节
    snprintf(buf, sizeof(buf), "%d", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示有符号 16 位整数 (int16_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值 (范围: -32768 ~ 32767)
 */
void tft18_int16(uint8_t x, uint8_t y, int16_t val)
{
    char buf[7]; // 负号 "-" + "32768" (5字符) + '\0' = 7字节
    snprintf(buf, sizeof(buf), "%d", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示有符号 32 位整数 (int32_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值 (范围: -2147483648 ~ 2147483647)
 */
void tft18_int32(uint8_t x, uint8_t y, int32_t val)
{
    char buf[12]; // 负号 "-" + "2147483648" (10字符) + '\0' = 12字节
    snprintf(buf, sizeof(buf), "%d", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示有符号 64 位整数 (int64_t)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的数值
 */
void tft18_int64(uint8_t x, uint8_t y, int64_t val)
{
    char buf[21]; // 负号 + 19位数字 + '\0' = 21字节
    // 注意：64位有符号整数标准格式化符为 %lld
    snprintf(buf, sizeof(buf), "%lld", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示浮点数 (float)
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  val  要显示的浮点数值
 * @note   默认 %f 保留 6 位小数，若需修改精度可改为 "%.2f" 等格式
 */
void tft18_float(uint8_t x, uint8_t y, float val)
{
    char buf[21]; // 足够容纳常规浮点数转换的字符串
    snprintf(buf, sizeof(buf), "%f", val);
    lq_tft18_drv_p8x8_str(x, y, buf, U16BLACK, U16GREEN);
}

/**
 * @brief  在指定坐标显示字符串
 * @param  x    横坐标
 * @param  y    纵坐标
 * @param  str  要显示的字符串指针
 * @note   调用者需确保传入的字符串以 '\0' 结尾。
 * 注释提示最大显示长度为 128 字符，受限于底层处理逻辑或屏幕尺寸。
 */
void tft18_string(uint8_t x, uint8_t y, const char *str)
{
    // 直接将指针透传给底层驱动
    lq_tft18_drv_p8x8_str(x, y, str, U16BLACK, U16GREEN);
}