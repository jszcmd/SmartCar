/*********************************************************************************************************************
 * 文件名称：wireless_wave.cpp
 * 功能描述：UDP 无线波形数据传输实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 非阻塞节流原理：
 *
 *   原版本：
 *     wireless_wave_Loop() {
 *         发送数据;
 *         usleep(100ms);  // ❌ 阻塞 100ms
 *     }
 *     调用者每次进来都要被卡 100ms
 *
 *   新版本：
 *     wireless_wave_loop() {
 *         now = 当前毫秒;
 *         if (now - last_send < interval) return;  // ⭐ 立即返回
 *         发送数据;
 *         last_send = now;
 *     }
 *     调用者立即返回，不阻塞
 *
 * ─────────────────────────────────────────────────────────────────────
 * 时间获取：
 *   使用 clock_gettime(CLOCK_MONOTONIC, ...) 获取单调递增时间
 *   不会受系统时间调整影响（gettimeofday 会受 NTP 影响）
 ********************************************************************************************************************/

#include "wireless_wave.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// ══════════════════════════════════════════════════════════════
// 通道描述
// ══════════════════════════════════════════════════════════════
typedef struct
{
    const float *pVar;              // 数据源指针，NULL = 未注册
    char name[WW_CHANNEL_NAME_MAX]; // 通道名称
} WW_Channel_t;

// ══════════════════════════════════════════════════════════════
// 内部状态
// ══════════════════════════════════════════════════════════════
static lq_udp_client udp_client;
static bool s_ready = false;
static bool s_enable = true;

static WW_Channel_t s_channels[WW_MAX_CHANNELS];

static uint32_t s_interval_ms = WW_DEFAULT_INTERVAL_MS;
static uint64_t s_last_send_ms = 0;

static uint32_t s_success_cnt = 0;
static uint32_t s_fail_cnt = 0;
static uint32_t s_fail_logged = 0; // 已经打印过错误的次数

// 引用编码器（用于默认通道注册）
extern float encoder_L_final;
extern float encoder_R_final;

// ══════════════════════════════════════════════════════════════
// 内部辅助：获取当前单调时间（毫秒）
// ══════════════════════════════════════════════════════════════
static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：构造发送字符串
//   "ch0:  1.23,ch1:  4.56,ch2:  7.89\n"
// 只输出已注册（pVar != NULL）的通道
// ══════════════════════════════════════════════════════════════
static int build_send_string(char *buf, int buf_size)
{
    int written = 0;
    bool first = true;

    for (int i = 0; i < WW_MAX_CHANNELS; i++)
    {
        if (s_channels[i].pVar == nullptr)
            continue;

        // 通道名（如果空则用默认 "chN"）
        const char *name = s_channels[i].name[0] ? s_channels[i].name : nullptr;
        char default_name[8];
        if (name == nullptr)
        {
            snprintf(default_name, sizeof(default_name), "ch%d", i);
            name = default_name;
        }

        // 取值
        float val = *(s_channels[i].pVar);

        // 拼接（首字段不加逗号）
        int n = snprintf(buf + written, buf_size - written,
                         "%s%s:%7.2f", first ? "" : ",", name, val);
        if (n < 0 || n >= buf_size - written)
            break;
        written += n;
        first = false;
    }

    // 末尾换行
    if (written < buf_size - 1)
    {
        buf[written++] = '\n';
        buf[written] = '\0';
    }

    return written;
}

// ══════════════════════════════════════════════════════════════
// 初始化
// ══════════════════════════════════════════════════════════════
void wireless_wave_Init(void)
{
    printf("=========================================\n");
    printf("  UDP -> LoongHost (non-blocking version)\n");
    printf("=========================================\n");
    printf("Target IP:   %s\n", WW_DEFAULT_IP);
    printf("Target Port: %d\n", WW_DEFAULT_PORT);
    printf("Interval:    %u ms (%.1f Hz)\n",
           s_interval_ms, 1000.0f / (float)s_interval_ms);
    printf("=========================================\n");

    // 清空通道数组
    memset(s_channels, 0, sizeof(s_channels));

    // 创建 UDP 客户端
    udp_client.udp_client_init(WW_DEFAULT_IP, WW_DEFAULT_PORT);
    int fd = udp_client.get_udp_socket_fd();
    if (fd < 0)
    {
        printf("FATAL: UDP socket failed! Check network (ifconfig).\n");
        s_ready = false;
        return;
    }

    // 启用广播权限（方便后续切换到广播 IP）
    int broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        printf("WARN: SO_BROADCAST set failed (non-fatal)\n");
    }

    // 默认注册：ch0 = encoder_L, ch1 = encoder_R
    ww_set_channel(0, &encoder_L_final, "ch1");
    ww_set_channel(1, &encoder_R_final, "ch2");

    s_ready = true;
    s_enable = true;
    s_last_send_ms = 0;
    s_success_cnt = 0;
    s_fail_cnt = 0;
    s_fail_logged = 0;

    printf("[WW] Init OK. socket fd=%d\n", fd);
}

// ══════════════════════════════════════════════════════════════
// 主循环（非阻塞）
// ══════════════════════════════════════════════════════════════
void wireless_wave_loop(void)
{
    if (!s_ready)
        return;
    if (!s_enable)
        return;

    // ── 节流：未到发送时刻就立即返回（非阻塞核心）─────────────
    uint64_t now = now_ms();
    if (s_last_send_ms != 0 &&
        (now - s_last_send_ms) < s_interval_ms)
    {
        return;
    }
    s_last_send_ms = now;

    // ── 构造发送字符串 ────────────────────────────────────────
    char buf[128];
    int len = build_send_string(buf, sizeof(buf));
    if (len <= 1)
        return; // 没有任何通道注册，啥也不发

    // ── 发送 ──────────────────────────────────────────────────
    ssize_t ret = udp_client.udp_send_string(buf);
    if (ret < 0)
    {
        s_fail_cnt++;
        // 失败日志限频，避免网络断开时刷屏
        if (s_fail_logged < WW_FAIL_LOG_LIMIT)
        {
            printf("[WW] SEND FAIL ret=%zd (%u/%u total)\n",
                   ret, s_fail_cnt, s_fail_cnt + s_success_cnt);
            s_fail_logged++;
            if (s_fail_logged == WW_FAIL_LOG_LIMIT)
                printf("[WW] (further send-fail logs suppressed)\n");
        }
    }
    else
    {
        s_success_cnt++;
        // 网络恢复后重置日志限频，下次再断开还能打印
        s_fail_logged = 0;
    }
}

// ══════════════════════════════════════════════════════════════
// 兼容包装：原 Loop 大写版本
// ══════════════════════════════════════════════════════════════
void wireless_wave_Loop(void)
{
    wireless_wave_loop();
}

// ══════════════════════════════════════════════════════════════
// 通道注册
// ══════════════════════════════════════════════════════════════
void ww_set_channel(uint8_t idx, const float *pVar, const char *name)
{
    if (idx >= WW_MAX_CHANNELS)
        return;

    s_channels[idx].pVar = pVar;

    if (name != nullptr)
    {
        // 安全拷贝（限制长度）
        strncpy(s_channels[idx].name, name, WW_CHANNEL_NAME_MAX - 1);
        s_channels[idx].name[WW_CHANNEL_NAME_MAX - 1] = '\0';
    }
    else
    {
        s_channels[idx].name[0] = '\0';
    }

    if (pVar)
        printf("[WW] Channel %d registered: %s\n", idx,
               s_channels[idx].name[0] ? s_channels[idx].name : "(default)");
    else
        printf("[WW] Channel %d unregistered\n", idx);
}

// ══════════════════════════════════════════════════════════════
// 修改目标 IP/端口
// ══════════════════════════════════════════════════════════════
bool ww_set_target(const char *ip, uint16_t port)
{
    if (ip == nullptr)
        return false;

    udp_client.udp_client_init(ip, port);
    int fd = udp_client.get_udp_socket_fd();
    if (fd < 0)
    {
        printf("[WW] ww_set_target FAILED: socket error\n");
        s_ready = false;
        return false;
    }

    int broadcast = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    s_ready = true;
    s_fail_cnt = s_success_cnt = s_fail_logged = 0;
    printf("[WW] Target updated: %s:%u (fd=%d)\n", ip, port, fd);
    return true;
}

// ══════════════════════════════════════════════════════════════
// 间隔配置
// ══════════════════════════════════════════════════════════════
void ww_set_interval(uint32_t ms)
{
    if (ms == 0)
        ms = 1; // 防 0
    s_interval_ms = ms;
    printf("[WW] Send interval set to %u ms (%.1f Hz)\n",
           ms, 1000.0f / (float)ms);
}

// ══════════════════════════════════════════════════════════════
// 启用/暂停
// ══════════════════════════════════════════════════════════════
void ww_set_enabled(bool enable)
{
    s_enable = enable;
    printf("[WW] %s\n", enable ? "ENABLED" : "PAUSED");
}

bool ww_is_enabled(void) { return s_enable; }
bool ww_is_ready(void) { return s_ready; }

uint32_t ww_get_success_count(void) { return s_success_cnt; }
uint32_t ww_get_fail_count(void) { return s_fail_cnt; }