#include "WW_TimerTasks.h"

static void timer_5ms_entry(void *) { timer_5ms_callback(); }
static void timer_20ms_entry(void *) { timer_20ms_callback(); }
static void timer_100ms_entry(void *) { timer_100ms_callback(); }

static TimerThread timer_5ms(static_cast<TimerThread::CallbackFunc>(timer_5ms_entry), nullptr, 5);
static TimerThread timer_20ms(static_cast<TimerThread::CallbackFunc>(timer_20ms_entry), nullptr, 20);
static TimerThread timer_100ms(static_cast<TimerThread::CallbackFunc>(timer_100ms_entry), nullptr, 100);
static bool timer_tasks_started = false;

static void stop_all_timers()
{
    timer_5ms.stop();
    timer_20ms.stop();
    timer_100ms.stop();
    timer_tasks_started = false;
}

// 5ms 周期任务
void timer_5ms_callback(void)
{
    encoder_update(); // 编码器数据更新函数，放在 5ms 周期执行
}

// 20ms 周期任务
void timer_20ms_callback(void)
{
    // 这里放 20ms 周期任务，比如速度环、姿态环、传感器轮询
}

// 100ms 周期任务
void timer_100ms_callback(void)
{
    key_loop();

    // 这里放 100ms 周期任务，比如状态上报、心跳灯、慢速滤波
}

void timer_start_tasks(void)
{
    if (timer_tasks_started)
    {
        return;
    }

    timer_tasks_started = true;

    lq_signal_set_exit_cb(stop_all_timers);

    timer_5ms.start();
    timer_20ms.start();
    timer_100ms.start();
}

void timer_stop_tasks(void)
{
    stop_all_timers();
}
