/*********************************************************************************************************************
 * 文件名称：buzzer.cpp
 * 功能描述：蜂鸣器控制模块实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 非阻塞状态机原理：
 *
 *   每次 buzzer_tick() 检查当前任务，按预设序列推进 GPIO 高低电平。
 *
 *   每个非阻塞任务由一个"音符序列"驱动：
 *     [{响 N tick}, {停 M tick}, {响 N tick}, ...]
 *
 *   状态机变量：
 *     s_step     ← 当前在序列的第几步
 *     s_step_cnt ← 当前步还剩几个 tick
 *
 *   每 tick：
 *     - s_step_cnt-- ；
 *     - 减到 0 时进入下一步，按下一步的"响/停"设置 GPIO；
 *     - 序列结束 → 任务完成，蜂鸣器关闭。
 *
 * ─────────────────────────────────────────────────────────────────────
 * 心跳独立运行：
 *   - 不和非阻塞鸣叫共用状态变量
 *   - 每 BUZZER_HEARTBEAT_PERIOD_TICKS 周期"借用"一下 GPIO，再恢复
 *   - 如果心跳和非阻塞任务"撞车"，让非阻塞优先（心跳跳过本次）
 ********************************************************************************************************************/

#include "buzzer.h"

// ══════════════════════════════════════════════════════════════
// 硬件单例
// ══════════════════════════════════════════════════════════════
static ls_gpio *pBuzzer = nullptr;
static bool s_is_on = false; // 当前 GPIO 实际电平

// ══════════════════════════════════════════════════════════════
// 非阻塞状态机
// ══════════════════════════════════════════════════════════════

// 音符序列中的一个音符（响/停 + 持续 tick 数）
typedef struct
{
    bool on;        // true=响，false=停
    uint16_t ticks; // 持续多少 tick
} BuzzerNote_t;

// 序列最大长度（足够装下任何预设音效）
#define BUZZER_MAX_NOTES 16

static BuzzerNote_t s_seq[BUZZER_MAX_NOTES]; // 当前任务的音符序列
static uint8_t s_seq_len = 0;                // 序列实际长度
static uint8_t s_step = 0;                   // 当前步索引
static uint16_t s_step_cnt = 0;              // 当前步剩余 tick 数
static bool s_busy = false;                  // 是否有任务正在执行
static bool s_loop = false;                  // 是否循环（紧急音效用）

// 心跳模式独立计数器
static bool s_heartbeat_enable = false;
static uint16_t s_heartbeat_cnt = 0;    // 距离上次心跳多少 tick
static bool s_heartbeat_active = false; // 心跳脉冲正在响中

// ══════════════════════════════════════════════════════════════
// 内部辅助：直接控制 GPIO
// ══════════════════════════════════════════════════════════════
static void buzzer_gpio_set(bool on)
{
    if (pBuzzer == nullptr)
        return;
    pBuzzer->gpio_level_set(on ? GPIO_HIGH : GPIO_LOW);
    s_is_on = on;
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：构造音符序列
// 把"响 count 次"展开成 [响,停,响,停,...]
// ══════════════════════════════════════════════════════════════
static void build_seq_beep(uint8_t count, uint16_t on_ticks, uint16_t gap_ticks)
{
    s_seq_len = 0;
    if (count == 0)
        return;

    for (uint8_t i = 0; i < count && s_seq_len < BUZZER_MAX_NOTES - 1; i++)
    {
        s_seq[s_seq_len++] = {true, on_ticks}; // 响
        // 最后一次不加间隔
        if (i < count - 1)
            s_seq[s_seq_len++] = {false, gap_ticks};
    }
}

// 构造长鸣序列：单个音符
static void build_seq_long(uint16_t on_ticks)
{
    s_seq_len = 1;
    s_seq[0] = {true, on_ticks};
}

// 启动当前序列（从第 0 步开始）
static void start_current_seq(bool loop)
{
    if (s_seq_len == 0)
    {
        s_busy = false;
        buzzer_gpio_set(false);
        return;
    }

    s_step = 0;
    s_step_cnt = s_seq[0].ticks;
    s_busy = true;
    s_loop = loop;
    buzzer_gpio_set(s_seq[0].on);
}

// ══════════════════════════════════════════════════════════════
// 初始化
// ══════════════════════════════════════════════════════════════
void Buzzer_Init(void)
{
    if (pBuzzer != nullptr)
        return;

    static ls_gpio instance(BUZZER_PIN, GPIO_MODE_OUT);
    pBuzzer = &instance;

    buzzer_gpio_set(false);

    // 清空状态机
    s_seq_len = 0;
    s_step = 0;
    s_step_cnt = 0;
    s_busy = false;
    s_loop = false;
    s_heartbeat_enable = false;
    s_heartbeat_cnt = 0;
    s_heartbeat_active = false;

    printf("[Buzzer] Init done. PIN=%d\n", BUZZER_PIN);
}

// ══════════════════════════════════════════════════════════════
// 基础开关
// ══════════════════════════════════════════════════════════════
void buzzer_on(void)
{
    Buzzer_Init();
    buzzer_gpio_set(true);
}

void buzzer_off(void)
{
    Buzzer_Init();
    buzzer_gpio_set(false);
}

bool buzzer_is_on(void) { return s_is_on; }

// ══════════════════════════════════════════════════════════════
// 阻塞模式（原版本，向下兼容）
// ══════════════════════════════════════════════════════════════
void buzzer_beep(uint8_t count)
{
    Buzzer_Init();
    for (uint8_t i = 0; i < count; i++)
    {
        buzzer_gpio_set(true);
        usleep(BUZZER_BEEP_US);
        buzzer_gpio_set(false);
        if (i < count - 1)
            usleep(BUZZER_GAP_US);
    }
}

void buzzer_long(uint32_t duration_us)
{
    Buzzer_Init();
    buzzer_gpio_set(true);
    usleep(duration_us);
    buzzer_gpio_set(false);
}

// ══════════════════════════════════════════════════════════════
// 非阻塞模式
// ══════════════════════════════════════════════════════════════
void buzzer_beep_async(uint8_t count)
{
    Buzzer_Init();
    build_seq_beep(count, BUZZER_SHORT_TICKS, BUZZER_GAP_TICKS);
    start_current_seq(false);
}

void buzzer_long_async(uint32_t duration_ms)
{
    Buzzer_Init();
    uint16_t ticks = (uint16_t)(duration_ms / BUZZER_TICK_PERIOD_MS);
    if (ticks == 0)
        ticks = 1;
    build_seq_long(ticks);
    start_current_seq(false);
}

// ══════════════════════════════════════════════════════════════
// 预设音效
// ══════════════════════════════════════════════════════════════
void buzzer_play_pattern(BuzzerPattern_t pattern)
{
    Buzzer_Init();

    switch (pattern)
    {
    case BUZZER_PATTERN_NONE:
        buzzer_stop_async();
        return;

    case BUZZER_PATTERN_SUCCESS:
        // 短-短：表示确认成功
        build_seq_beep(2, BUZZER_SHORT_TICKS, BUZZER_GAP_TICKS);
        start_current_seq(false);
        break;

    case BUZZER_PATTERN_ERROR:
        // 长-长-长：表示错误
        s_seq_len = 5;
        s_seq[0] = {true, BUZZER_LONG_TICKS};
        s_seq[1] = {false, BUZZER_GAP_TICKS};
        s_seq[2] = {true, BUZZER_LONG_TICKS};
        s_seq[3] = {false, BUZZER_GAP_TICKS};
        s_seq[4] = {true, BUZZER_LONG_TICKS};
        start_current_seq(false);
        break;

    case BUZZER_PATTERN_WARNING:
        // 短-短-短：表示警告
        build_seq_beep(3, BUZZER_SHORT_TICKS, BUZZER_GAP_TICKS);
        start_current_seq(false);
        break;

    case BUZZER_PATTERN_STARTUP:
        // 长-短-短：表示启动完成
        s_seq_len = 5;
        s_seq[0] = {true, BUZZER_LONG_TICKS};
        s_seq[1] = {false, BUZZER_GAP_TICKS};
        s_seq[2] = {true, BUZZER_SHORT_TICKS};
        s_seq[3] = {false, BUZZER_GAP_TICKS};
        s_seq[4] = {true, BUZZER_SHORT_TICKS};
        start_current_seq(false);
        break;

    case BUZZER_PATTERN_EMERGENCY:
        // 连续快响（循环）
        s_seq_len = 2;
        s_seq[0] = {true, BUZZER_SHORT_TICKS};
        s_seq[1] = {false, BUZZER_SHORT_TICKS};
        start_current_seq(true); // 循环模式
        break;
    }
}

// ══════════════════════════════════════════════════════════════
// 停止异步任务
// ══════════════════════════════════════════════════════════════
void buzzer_stop_async(void)
{
    s_busy = false;
    s_loop = false;
    s_seq_len = 0;
    buzzer_gpio_set(false);
}

bool buzzer_is_busy(void) { return s_busy; }

// ══════════════════════════════════════════════════════════════
// 心跳模式
// ══════════════════════════════════════════════════════════════
void buzzer_set_heartbeat(bool enable)
{
    Buzzer_Init();
    s_heartbeat_enable = enable;
    s_heartbeat_cnt = 0;
    s_heartbeat_active = false;
    printf("[Buzzer] Heartbeat %s\n", enable ? "ENABLED" : "DISABLED");
}

// ══════════════════════════════════════════════════════════════
// 状态机驱动（必须放定时器里）
// ══════════════════════════════════════════════════════════════
void buzzer_tick(void)
{
    Buzzer_Init();

    // ── 1. 处理非阻塞任务 ────────────────────────────────────
    if (s_busy)
    {
        if (s_step_cnt > 0)
        {
            s_step_cnt--;
        }

        if (s_step_cnt == 0)
        {
            // 进入下一步
            s_step++;
            if (s_step >= s_seq_len)
            {
                if (s_loop)
                {
                    // 循环模式：从头开始
                    s_step = 0;
                }
                else
                {
                    // 序列结束
                    s_busy = false;
                    buzzer_gpio_set(false);
                    return;
                }
            }
            // 应用新一步的 GPIO 状态
            s_step_cnt = s_seq[s_step].ticks;
            buzzer_gpio_set(s_seq[s_step].on);
        }
        return; // 非阻塞任务运行中，跳过心跳
    }

    // ── 2. 处理心跳（仅在没有非阻塞任务时）───────────────────
    if (s_heartbeat_enable)
    {
        s_heartbeat_cnt++;

        if (!s_heartbeat_active)
        {
            // 等待心跳周期到达
            if (s_heartbeat_cnt >= BUZZER_HEARTBEAT_PERIOD_TICKS)
            {
                buzzer_gpio_set(true);
                s_heartbeat_active = true;
                s_heartbeat_cnt = 0;
            }
        }
        else
        {
            // 心跳脉冲正在响中，等待结束
            if (s_heartbeat_cnt >= BUZZER_HEARTBEAT_PULSE_TICKS)
            {
                buzzer_gpio_set(false);
                s_heartbeat_active = false;
                s_heartbeat_cnt = 0;
            }
        }
    }
}
