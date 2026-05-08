/*********************************************************************************************************************
 * 文件名称：WW_TimerTasks.cpp
 * 功能描述：定时器任务调度模块实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 整体调度流程（每个 tick = 5ms 触发一次 unified_entry）：
 *
 *   ┌──────────────────────────────────────────────────────────┐
 *   │  unified_entry() 入口                                      │
 *   │   ↓                                                        │
 *   │  记录 tick 起始时间 t_start                                  │
 *   │   ↓                                                        │
 *   │  超载检测：与上次 t_start 比较，间隔异常则计数               │
 *   │   ↓                                                        │
 *   │  g_timer_tick++                                             │
 *   │   ↓                                                        │
 *   │  按分频比依次调用各回调                                      │
 *   │    - 5ms   每 tick 都跑                                     │
 *   │    - 10ms  tick % 2 == 0                                    │
 *   │    - 20ms  tick % 4 == 0                                    │
 *   │    - 100ms tick % 20 == 0                                   │
 *   │   ↓                                                        │
 *   │  每个回调都包在 measure 函数里：                              │
 *   │    - 记录开始时间                                            │
 *   │    - 调用回调                                                │
 *   │    - 计算耗时，更新最大值，超时则打印 WARN                    │
 *   └──────────────────────────────────────────────────────────┘
 *
 * ─────────────────────────────────────────────────────────────────────
 * 任务顺序（5ms tick 内）的关键性：
 *
 *   错误顺序（原版本）：
 *     encoder_update() → close_loop_update() → distance_sensor_update()
 *     问题：没有 Motor_UpdateRamp，加速度限制功能用不了
 *
 *   正确顺序（新版本）：
 *     1. encoder_update()         // 先采集速度
 *     2. close_loop_update()      // 用最新速度算 PID，更新目标 PWM
 *     3. Motor_UpdateRamp()       // 把目标 PWM 平滑过渡到硬件
 *     4. distance_sensor_update() // 测距独立，顺序无关
 *     5. buzzer_tick()            // 蜂鸣器状态机推进
 ********************************************************************************************************************/

#include "WW_TimerTasks.h"
#include <stdio.h>
#include <time.h>

// ══════════════════════════════════════════════════════════════
// 全局变量定义
// ══════════════════════════════════════════════════════════════
uint32_t g_timer_tick = 0;

// ══════════════════════════════════════════════════════════════
// 内部状态
// ══════════════════════════════════════════════════════════════
static TimerThread *unified_timer = nullptr;
static bool s_timer_running = false;

// 性能统计
static uint32_t s_max_us_5ms = 0;
static uint32_t s_max_us_10ms = 0;
static uint32_t s_max_us_20ms = 0;
static uint32_t s_max_us_100ms = 0;
static uint32_t s_overtime_count = 0;
static uint32_t s_overload_count = 0;
static uint32_t s_overtime_logged = 0;

// 上一个 tick 的起始时间（用于超载检测）
static uint64_t s_last_tick_us = 0;

// ══════════════════════════════════════════════════════════════
// 内部辅助：获取单调时间（微秒）
// ══════════════════════════════════════════════════════════════
static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：测量并执行任务，更新最大耗时
// ══════════════════════════════════════════════════════════════
typedef void (*TaskFunc_t)(void);

static void measure_and_run(TaskFunc_t func, uint32_t *max_us, const char *task_name)
{
    if (func == nullptr)
        return;

    uint64_t t_start = now_us();
    func();
    uint64_t elapsed = now_us() - t_start;

    // 更新最大耗时
    if ((uint32_t)elapsed > *max_us)
        *max_us = (uint32_t)elapsed;

    // 超时检测（仅 5ms 类任务）
    if (elapsed > TIMER_OVERTIME_THRESH_US)
    {
        s_overtime_count++;
        if (s_overtime_logged < TIMER_OVERTIME_LOG_LIMIT)
        {
            printf("[Timer] WARN: %s took %lu us (>%d us threshold)\n",
                   task_name, (unsigned long)elapsed, TIMER_OVERTIME_THRESH_US);
            s_overtime_logged++;
            if (s_overtime_logged == TIMER_OVERTIME_LOG_LIMIT)
                printf("[Timer] (further overtime warnings suppressed)\n");
        }
    }
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：停止并释放定时器
// 注册为系统退出回调，Ctrl+C 时自动触发
// ══════════════════════════════════════════════════════════════
static void stop_all_timers(void)
{
    if (unified_timer != nullptr)
    {
        unified_timer->stop();
        delete unified_timer;
        unified_timer = nullptr;
    }
    s_timer_running = false;
    printf("[Timer] All tasks stopped.\n");
}

// ══════════════════════════════════════════════════════════════
// 各周期任务回调
// ══════════════════════════════════════════════════════════════

// ── 5ms 任务：高频实时控制 ────────────────────────────────────
void timer_5ms_callback(void)
{
    // 顺序很关键，不要随意调换
    encoder_update();         // 1. 先采集编码器
    close_loop_update();      // 2. 用最新数据算 PID
    Motor_UpdateRamp();       // 3. 加速度限制平滑（若启用）
    distance_sensor_update(); // 4. 测距独立，顺序无关
    buzzer_tick();            // 5. 蜂鸣器状态机推进
}

// ── 10ms 任务：无线波形 ───────────────────────────────────────
void timer_10ms_callback(void)
{
    // wireless_wave_loop 内部还有节流（默认 100ms 一次）
    // 这里 10ms 调用一次只是给它机会决定要不要发
    wireless_wave_loop();
}

// ── 20ms 任务：陀螺仪 ─────────────────────────────────────────
void timer_20ms_callback(void)
{
    mpu6050_loop();
}

// ── 100ms 任务：按键 ──────────────────────────────────────────
void timer_100ms_callback(void)
{
    key_loop();
}

// ══════════════════════════════════════════════════════════════
// 统一定时器入口（运行在定时器线程中）
// ══════════════════════════════════════════════════════════════
static void unified_entry(void *)
{
    uint64_t tick_start = now_us();

    // ── 超载检测 ─────────────────────────────────────────────
    // tick 间隔应该恰好是 5000 µs，如果偏差超过阈值说明系统过载
    if (s_last_tick_us != 0)
    {
        uint64_t actual_interval = tick_start - s_last_tick_us;
        uint64_t expected = (uint64_t)TIMER_BASE_PERIOD_MS * 1000;
        uint64_t jitter = (actual_interval > expected) ? (actual_interval - expected) : (expected - actual_interval);
        if (jitter > TIMER_OVERLOAD_JITTER_US)
        {
            s_overload_count++;
        }
    }
    s_last_tick_us = tick_start;

    // ── 全局 tick 自增 ───────────────────────────────────────
    g_timer_tick++;

    // ── 5ms 任务：每 tick 都跑 ───────────────────────────────
    measure_and_run(timer_5ms_callback, &s_max_us_5ms, "5ms_task");

    // ── 10ms 任务 ────────────────────────────────────────────
    if ((g_timer_tick % TIMER_DIV_10MS) == 0)
        measure_and_run(timer_10ms_callback, &s_max_us_10ms, "10ms_task");

    // ── 20ms 任务 ────────────────────────────────────────────
    if ((g_timer_tick % TIMER_DIV_20MS) == 0)
        measure_and_run(timer_20ms_callback, &s_max_us_20ms, "20ms_task");

    // ── 100ms 任务 ───────────────────────────────────────────
    if ((g_timer_tick % TIMER_DIV_100MS) == 0)
        measure_and_run(timer_100ms_callback, &s_max_us_100ms, "100ms_task");
}

// ══════════════════════════════════════════════════════════════
// 启动调度
// ══════════════════════════════════════════════════════════════
void timer_start_tasks(void)
{
    if (s_timer_running)
    {
        printf("[Timer] Already running, skip.\n");
        return;
    }

    // 注册系统退出回调（Ctrl+C / SIGINT）
    lq_signal_set_exit_cb(stop_all_timers);

    // 重置性能统计
    timer_reset_stats();

    // 创建统一定时器
    unified_timer = new TimerThread(
        static_cast<TimerThread::CallbackFunc>(unified_entry),
        nullptr,
        TIMER_BASE_PERIOD_MS // 基础周期
    );

    unified_timer->start();
    s_timer_running = true;

    printf("[Timer] Tasks started. Base = %d ms\n", TIMER_BASE_PERIOD_MS);
    printf("[Timer]   5ms   → encoder + close_loop + ramp + distance + buzzer\n");
    printf("[Timer]   10ms  → wireless_wave\n");
    printf("[Timer]   20ms  → mpu6050\n");
    printf("[Timer]   100ms → key\n");
}

// ══════════════════════════════════════════════════════════════
// 停止调度
// ══════════════════════════════════════════════════════════════
void timer_stop_tasks(void)
{
    stop_all_timers();
}

bool timer_is_running(void)
{
    return s_timer_running;
}

// ══════════════════════════════════════════════════════════════
// 性能监控
// ══════════════════════════════════════════════════════════════
uint32_t timer_get_max_us_5ms(void) { return s_max_us_5ms; }
uint32_t timer_get_max_us_10ms(void) { return s_max_us_10ms; }
uint32_t timer_get_max_us_20ms(void) { return s_max_us_20ms; }
uint32_t timer_get_max_us_100ms(void) { return s_max_us_100ms; }

uint32_t timer_get_overtime_count(void) { return s_overtime_count; }
uint32_t timer_get_overload_count(void) { return s_overload_count; }

void timer_reset_stats(void)
{
    s_max_us_5ms = s_max_us_10ms = s_max_us_20ms = s_max_us_100ms = 0;
    s_overtime_count = 0;
    s_overload_count = 0;
    s_overtime_logged = 0;
    s_last_tick_us = 0;
}

void timer_print_stats(void)
{
    printf("┌───────────────── Timer Stats ─────────────────┐\n");
    printf("│ tick=%-10u  running=%s                        │\n", g_timer_tick, s_timer_running ? "yes" : "no ");
    printf("│ max_us_5ms   = %5u                            │\n", s_max_us_5ms);
    printf("│ max_us_10ms  = %5u                            │\n", s_max_us_10ms);
    printf("│ max_us_20ms  = %5u                            │\n", s_max_us_20ms);
    printf("│ max_us_100ms = %5u                            │\n", s_max_us_100ms);
    printf("│ overtime_count = %u                           │\n", s_overtime_count);
    printf("│ overload_count = %u                           │\n", s_overload_count);
    printf("└───────────────────────────────────────────────┘\n");
}