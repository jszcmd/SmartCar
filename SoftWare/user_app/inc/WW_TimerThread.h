/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 高精度定时线程头文件
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 功能描述：基于 POSIX 线程的高精度定时器封装（支持时间补偿，防累积误差）
 * 适用场景：机器人控制、PID 周期运算、传感器高频采样
 ********************************************************************************************************************/

#ifndef __WUWU_TIMERTHREAD_H
#define __WUWU_TIMERTHREAD_H

#include <pthread.h>
#include <time.h>
// 移除了 #include "main.hpp" 以降低耦合度，如果有必要的数据类型请在此单独包含或前置声明

class TimerThread
{
public:
    /******************************************************************
     * @brief   线程回调函数原型
     ******************************************************************/
    typedef void (*CallbackFunc)(void *);

    /******************************************************************
     * @brief       构造函数
     *
     * @param       func                    传入周期控制函数
     * @param       arg                     传入回调函数的参数, 没有设置为NULL
     * @param       interval_ms             执行间隔（毫秒），默认10ms
     ******************************************************************/
    TimerThread(CallbackFunc func, void *arg = NULL, unsigned int interval_ms = 10);

    /******************************************************************
     * @brief       析构函数（自动安全停止线程）
     ******************************************************************/
    ~TimerThread(void);

    /******************************************************************
     * @brief       启动定时线程
     * @return      成功返回true，失败返回false
     ******************************************************************/
    bool start();

    /******************************************************************
     * @brief       停止定时线程
     * @note        将安全地通知线程退出循环
     ******************************************************************/
    void stop();

    /******************************************************************
     * @brief       检查线程是否正在运行
     ******************************************************************/
    bool isRunning() const { return m_running; }

private:
    static void *thread_entry(void *param);

    // 辅助函数：将时间步进指定的毫秒数
    static void add_ms_to_timespec(struct timespec *ts, unsigned int ms);

    pthread_t thread_id;
    CallbackFunc func;
    void *arg;
    unsigned int interval_ms;

    volatile bool m_running; // 线程运行标志位，使用 volatile 防止编译器优化
};

#endif // __WUWU_TIMERTHREAD_H