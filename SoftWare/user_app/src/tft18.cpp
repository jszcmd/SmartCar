/*********************************************************************************************************************
 * 文件名称：tft18.cpp
 * 功能描述：TFT18 屏幕显示封装实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 函数命名修复（继承自原版本）：
 *   原代码用 lq_tft18_drv_putstr_p8x8（不存在），实际库里是 lq_tft18_drv_p8x8_str
 *   本文件统一用正确的函数名
 *
 * ─────────────────────────────────────────────────────────────────────
 * 字体使用约定：
 *   通过 tft18_set_font() 切换当前字体
 *   字体内部宏映射到对应的 lq 库函数：
 *     TFT_FONT_6X8  → lq_tft18_drv_p6x8_str
 *     TFT_FONT_8X8  → lq_tft18_drv_p8x8_str  （默认）
 *     TFT_FONT_8X16 → lq_tft18_drv_p8x16_str
 ********************************************************************************************************************/

#include "tft18.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// ══════════════════════════════════════════════════════════════
// 当前字体和颜色
// ══════════════════════════════════════════════════════════════
static TftFont_t s_font = TFT_FONT_8X8;
static lq_display_color_t s_fg = TFT18_DEFAULT_FG;
static lq_display_color_t s_bg = TFT18_DEFAULT_BG;

// 自动换行光标
static uint8_t s_cursor_y = 0;

// ══════════════════════════════════════════════════════════════
// 内部辅助：用当前字体打印字符串
// ══════════════════════════════════════════════════════════════
static void putstr_with_current_font(uint8_t x, uint8_t y, const char *str)
{
    switch (s_font)
    {
    case TFT_FONT_6X8:
        lq_tft18_drv_p6x8_str(x, y, str, s_fg, s_bg);
        break;
    case TFT_FONT_8X8:
        lq_tft18_drv_p8x8_str(x, y, str, s_fg, s_bg);
        break;
    case TFT_FONT_8X16:
        lq_tft18_drv_p8x16_str(x, y, str, s_fg, s_bg);
        break;
    }
}

// ══════════════════════════════════════════════════════════════
// 初始化
// ══════════════════════════════════════════════════════════════
void tft18_init(void)
{
    lq_tft18_drv_init(TFT18_PORTRAIT);
    lq_tft18_drv_cls(s_bg);
    s_cursor_y = 0;
    printf("[TFT18] Init done. Mode=%s %dx%d\n",
           TFT18_PORTRAIT ? "Portrait" : "Landscape",
           TFT18_SCREEN_W, TFT18_SCREEN_H);
}

void tft18_flush(void)
{
    lq_tft18_drv_flush();
}

void tft18_cls(lq_display_color_t color)
{
    lq_tft18_drv_cls(color);
    s_cursor_y = 0;
}

// ══════════════════════════════════════════════════════════════
// 多字号 / 颜色 配置
// ══════════════════════════════════════════════════════════════
void tft18_set_font(TftFont_t font) { s_font = font; }

uint8_t tft18_font_width(void)
{
    switch (s_font)
    {
    case TFT_FONT_6X8:
        return 6;
    case TFT_FONT_8X8:
        return 8;
    case TFT_FONT_8X16:
        return 8;
    default:
        return 8;
    }
}

uint8_t tft18_font_height(void)
{
    switch (s_font)
    {
    case TFT_FONT_6X8:
        return 8;
    case TFT_FONT_8X8:
        return 8;
    case TFT_FONT_8X16:
        return 16;
    default:
        return 8;
    }
}

void tft18_set_color(lq_display_color_t fg, lq_display_color_t bg)
{
    s_fg = fg;
    s_bg = bg;
}

// ══════════════════════════════════════════════════════════════
// 整数显示（向下兼容）
// ══════════════════════════════════════════════════════════════
void tft18_uint8(uint8_t x, uint8_t y, uint8_t val)
{
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_uint16(uint8_t x, uint8_t y, uint16_t val)
{
    char buf[6];
    snprintf(buf, sizeof(buf), "%u", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_uint32(uint8_t x, uint8_t y, uint32_t val)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "%u", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_uint64(uint8_t x, uint8_t y, uint64_t val)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "%lu", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_int8(uint8_t x, uint8_t y, int8_t val)
{
    char buf[5];
    snprintf(buf, sizeof(buf), "%d", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_int16(uint8_t x, uint8_t y, int16_t val)
{
    char buf[7];
    snprintf(buf, sizeof(buf), "%d", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_int32(uint8_t x, uint8_t y, int32_t val)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", val);
    putstr_with_current_font(x, y, buf);
}

void tft18_int64(uint8_t x, uint8_t y, int64_t val)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "%ld", val);
    putstr_with_current_font(x, y, buf);
}

// ══════════════════════════════════════════════════════════════
// 浮点 / 字符串显示
// ══════════════════════════════════════════════════════════════
void tft18_float(uint8_t x, uint8_t y, float val, uint8_t decimals)
{
    char fmt[8];
    char buf[24];
    snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
    snprintf(buf, sizeof(buf), fmt, val);
    putstr_with_current_font(x, y, buf);
}

void tft18_string(uint8_t x, uint8_t y, const char *str)
{
    putstr_with_current_font(x, y, str);
}

// ══════════════════════════════════════════════════════════════
// 带标签显示
// ══════════════════════════════════════════════════════════════
void tft18_label_int16(uint8_t x, uint8_t y, const char *label, int16_t val)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%s: %d", label, val);
    putstr_with_current_font(x, y, buf);
}

void tft18_label_float(uint8_t x, uint8_t y, const char *label, float val, uint8_t decimals)
{
    char fmt[16];
    char buf[32];
    snprintf(fmt, sizeof(fmt), "%%s: %%.%df", decimals);
    snprintf(buf, sizeof(buf), fmt, label, val);
    putstr_with_current_font(x, y, buf);
}

void tft18_label_uint16(uint8_t x, uint8_t y, const char *label, uint16_t val)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%s: %u", label, val);
    putstr_with_current_font(x, y, buf);
}

// ══════════════════════════════════════════════════════════════
// 自动换行
// ══════════════════════════════════════════════════════════════
void tft18_reset_cursor(void)
{
    s_cursor_y = 0;
}

void tft18_println(const char *fmt, ...)
{
    char buf[64];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 当前行打印
    putstr_with_current_font(0, s_cursor_y, buf);

    // 光标移到下一行
    s_cursor_y += tft18_font_height();

    // 超出屏幕底部 → 环绕回顶部
    if (s_cursor_y + tft18_font_height() > TFT18_SCREEN_H)
        s_cursor_y = 0;
}

// ══════════════════════════════════════════════════════════════
// 绘图原语
// 注：lq 库里是否有完整的画点画线接口需要看实现
// 这里假设至少有 fill_rect 类似函数；如果不存在，进度条用整行字符串绕过
// ══════════════════════════════════════════════════════════════

void tft18_pixel(uint8_t x, uint8_t y, lq_display_color_t color)
{
    // 用 1×1 矩形模拟点
    tft18_rect_filled(x, y, 1, 1, color);
}

void tft18_hline(uint8_t x, uint8_t y, uint8_t w, lq_display_color_t color)
{
    tft18_rect_filled(x, y, w, 1, color);
}

void tft18_vline(uint8_t x, uint8_t y, uint8_t h, lq_display_color_t color)
{
    tft18_rect_filled(x, y, 1, h, color);
}

void tft18_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, lq_display_color_t color)
{
    if (w == 0 || h == 0)
        return;
    tft18_hline(x, y, w, color);
    tft18_hline(x, y + h - 1, w, color);
    tft18_vline(x, y, h, color);
    tft18_vline(x + w - 1, y, h, color);
}

/*
 * tft18_rect_filled 是绘图基础函数。
 * lq_display_tft18 库提供了 lq_tft18_drv_full_rect / lq_tft18_drv_fill_rect 之类
 * 由于不同版本命名可能有差异，这里做一个降级方案：
 *   - 优先调用库的填充函数（如果存在请取消下面的注释并改成实际名字）
 *   - 否则用循环画字符串模拟（开销略大但兼容性好）
 */
void tft18_rect_filled(uint8_t x, uint8_t y, uint8_t w, uint8_t h, lq_display_color_t color)
{
    // ─── 方案 A：直接调用库的填充矩形函数 ──────────────────
    // 如果 lq 库有这个函数，取消注释这一行：
    // lq_tft18_drv_full_rect(x, y, x + w, y + h, color);

    // ─── 方案 B：用空格字符串填充（兼容方案）──────────────
    // 把背景色临时设为 color，然后画一行空格字符
    lq_display_color_t old_bg = s_bg;
    s_bg = color;

    // 准备填充字符串（每个字符占 8 像素宽）
    char fill[32];
    int chars = (w + 7) / 8; // 向上取整
    if (chars >= (int)sizeof(fill))
        chars = sizeof(fill) - 1;
    memset(fill, ' ', chars);
    fill[chars] = '\0';

    // 多行绘制（每行 8 像素高）
    TftFont_t old_font = s_font;
    s_font = TFT_FONT_8X8;
    for (uint8_t row = 0; row < h; row += 8)
    {
        putstr_with_current_font(x, y + row, fill);
    }
    s_font = old_font;
    s_bg = old_bg;
}

// ══════════════════════════════════════════════════════════════
// 进度条
// ══════════════════════════════════════════════════════════════
void tft18_progress_bar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    // 1. 绘制外边框
    tft18_rect(x, y, w, h, U16BLACK);

    // 2. 内部空白填充背景
    if (w > 2 && h > 2)
        tft18_rect_filled(x + 1, y + 1, w - 2, h - 2, s_bg);

    // 3. 按百分比填充进度
    uint8_t fill_w = (uint8_t)((w - 2) * percent / 100);
    if (fill_w > 0 && h > 2)
    {
        // 颜色根据进度变化：低=红，中=黄，高=绿
        lq_display_color_t color;
        if (percent < 30)
            color = U16RED;
        else if (percent < 70)
            color = U16YELLOW;
        else
            color = U16BLUE; // 用蓝代替绿（背景已经是绿了）
        tft18_rect_filled(x + 1, y + 1, fill_w, h - 2, color);
    }
}

// ══════════════════════════════════════════════════════════════
// 水平柱状条（用于显示 value/max 的比例）
// ══════════════════════════════════════════════════════════════
void tft18_h_bar(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                 float value, float max_value)
{
    if (max_value <= 0.0f)
        return;
    if (value < 0.0f)
        value = 0.0f;
    if (value > max_value)
        value = max_value;

    uint8_t percent = (uint8_t)(value * 100.0f / max_value);
    tft18_progress_bar(x, y, w, h, percent);
}

// ══════════════════════════════════════════════════════════════
// 状态指示灯（5×5 实心方块当圆点用）
// ══════════════════════════════════════════════════════════════
void tft18_status_led(uint8_t x, uint8_t y, TftLedState_t status)
{
    lq_display_color_t color;
    switch (status)
    {
    case TFT_LED_OK:
        color = U16BLUE;
        break; // 在绿底上"绿色"看不见，用蓝
    case TFT_LED_WARN:
        color = U16YELLOW;
        break;
    case TFT_LED_ERROR:
        color = U16RED;
        break;
    case TFT_LED_INFO:
        color = U16WHITE;
        break;
    case TFT_LED_OFF:
    default:
        color = U16BLACK;
        break;
    }
    tft18_rect_filled(x, y, 5, 5, color);
}

// ══════════════════════════════════════════════════════════════
// 带标签的状态灯
// ══════════════════════════════════════════════════════════════
void tft18_status_led_label(uint8_t x, uint8_t y,
                            const char *label,
                            TftLedState_t status)
{
    tft18_status_led(x, y + 1, status); // 灯稍微下移对齐文字
    tft18_string(x + 8, y, label);      // 文字右移 8 像素
}