/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 高精度定时线程源文件
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 功能描述：实现了带有绝对时间补偿机制的定时循环，有效消除任务执行耗时带来的周期偏移。
 ********************************************************************************************************************/

#include "WW_TimerThread.h"

TimerThread::TimerThread(CallbackFunc func, void *arg, unsigned int interval_ms)
{
    this->func = func;
    this->arg = arg;
    this->interval_ms = interval_ms;
    this->thread_id = 0;
    this->m_running = false;
}

TimerThread::~TimerThread(void)
{
    // 如果对象被销毁，确保线程安全退出
    stop();
}

bool TimerThread::start()
{
    if (m_running)
    {
        return true; // 已经运行则直接返回
    }

    m_running = true;

    if (pthread_create(&thread_id, NULL, thread_entry, this) != 0)
    {
        m_running = false;
        return false;
    }

    // 设置线程为分离状态，主线程无需 join
    pthread_detach(thread_id);
    return true;
}

void TimerThread::stop()
{
    if (m_running)
    {
        m_running = false;
        // 注意：因为是 detach 状态，我们只修改标志位，让线程自己走到终点退出。
        // 如果需要严格同步等待它退出，需要改用 joinable 模式。
        // 但对于控制系统，当前这种优雅退出（修改标志位）通常足够了。
    }
}

// 辅助函数：时间累加计算
void TimerThread::add_ms_to_timespec(struct timespec *ts, unsigned int ms)
{
    ts->tv_nsec += (ms % 1000) * 1000000L;
    ts->tv_sec += ms / 1000;

    // 处理纳秒进位
    if (ts->tv_nsec >= 1000000000L)
    {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

void *TimerThread::thread_entry(void *param)
{
    TimerThread *self = (TimerThread *)param;

    struct timespec next_time;

    // 获取当前单调时间（不受系统时间被修改的影响，适合测距/定时）
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    // 核心循环：受 m_running 标志位控制
    while (self->m_running)
    {

        // 1. 执行业务回调函数 (比如 PID 计算)
        if (self->func)
        {
            self->func(self->arg);
        }

        // 2. 计算下一次的【绝对触发时间】
        // 关键点：我们是基于上一次的目标时间加上 interval，而不是基于当前时间！
        // 这样即使 self->func 执行消耗了 3ms，下一次休眠也会自动缩短 3ms。
        add_ms_to_timespec(&next_time, self->interval_ms);

        // 3. 休眠直到目标时间到达
        // TIMER_ABSTIME 表示我们要睡到这个绝对的时间点
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    return NULL;
}