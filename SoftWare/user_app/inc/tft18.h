#ifndef TFT18_H // 条件编译，防止头文件被重复包含
#define TFT18_H

// 包含项目依赖的公共头文件和底层 TFT18 驱动头文件
#include "lq_display_types.hpp"
#include "lq_display_tft18.hpp"
#include "lq_display_font.hpp"

/**
 * @brief  TFT18 显示屏初始化
 * @note   在使用任何屏幕显示功能前必须优先调用,用于初始化引脚、配置屏幕方向及清屏。
 */
void tft18_init(void);

/********************************************************************************
 * @name    数据与字符串显示接口
 * @brief   在屏幕指定位置显示不同类型的数据。底层默认使用黑字绿底显示。
 * @param   x   横坐标 (列)
 * @param   y   纵坐标 (行)
 ********************************************************************************/

// ================= 无符号整数显示 =================
/** @brief 显示无符号 8 位整数 (范围: 0 ~ 255) */
void tft18_uint8(uint8_t x, uint8_t y, uint8_t val);

/** @brief 显示无符号 16 位整数 (范围: 0 ~ 65535) */
void tft18_uint16(uint8_t x, uint8_t y, uint16_t val);

/** @brief 显示无符号 32 位整数 */
void tft18_uint32(uint8_t x, uint8_t y, uint32_t val);

/** @brief 显示无符号 64 位整数 */
void tft18_uint64(uint8_t x, uint8_t y, uint64_t val);

// ================= 有符号整数显示 =================
/** @brief 显示有符号 8 位整数 (范围: -128 ~ 127) */
void tft18_int8(uint8_t x, uint8_t y, int8_t val);

/** @brief 显示有符号 16 位整数 (范围: -32768 ~ 32767) */
void tft18_int16(uint8_t x, uint8_t y, int16_t val);

/** @brief 显示有符号 32 位整数 */
void tft18_int32(uint8_t x, uint8_t y, int32_t val);

/** @brief 显示有符号 64 位整数 */
void tft18_int64(uint8_t x, uint8_t y, int64_t val);

// ================= 浮点数显示 =================
/** * @brief 显示单精度浮点数 (float)
 * @note  默认格式化保留 6 位小数
 */
void tft18_float(uint8_t x, uint8_t y, float val);

// ================= 字符串显示 =================
/** * @brief 显示字符串
 * @param str 指向要显示的字符串的指针
 * @note  传入的字符串必须以 '\0' 结尾，最大显示长度受限于内部缓冲区 (128字符)
 */
void tft18_string(uint8_t x, uint8_t y, const char *str);

#endif // TFT18_H