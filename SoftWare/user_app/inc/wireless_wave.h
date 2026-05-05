#ifndef WIRELESS_WAVE_H
#define WIRELESS_WAVE_H

// #include "headfile.h"
#include "encoder.h"
#include "lq_udp_client.hpp"
#include <sys/socket.h>

void wireless_wave_Init(void); // 无线波形初始化
void wireless_wave_Loop(void); // 无线波形主循环,放while循环里

#endif // 无线波形数据传输