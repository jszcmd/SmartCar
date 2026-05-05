#ifndef __WW_TIMER_TASKS_H
#define __WW_TIMER_TASKS_H

#include <unistd.h>
#include "encoder.h"
#include "key.h"
#include "lq_signal_handle.hpp"
#include "WW_TimerThread.h"

void timer_5ms_callback(void);
void timer_20ms_callback(void);
void timer_100ms_callback(void);

void timer_start_tasks(void);
void timer_stop_tasks(void);

#endif // __WW_TIMER_TASKS_H