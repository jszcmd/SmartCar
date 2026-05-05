#include "wireless_wave.h"

/** * @brief 外部引用的左右轮编码器最终计算值
 * @note  由外部 encoder 模块计算得出，此处仅做读取发送
 */
extern float encoder_L_final, encoder_R_final;

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
/** @brief 目标上位机 (PC端) 的 IP 地址 */
const std::string TARGET_IP = "10.186.189.225";
/** @brief 目标上位机接收数据的 UDP 端口号 */
const uint16_t TARGET_PORT = 8080;

/** @brief 静态 UDP 客户端实例，负责底层网络通信 */
static lq_udp_client udp_client;
/** @brief UDP 初始化完成标志位，用于防止在未准备好时意外调用发送函数 */
static bool udp_ready = false;

/**
 * @brief  无线数据传输/虚拟示波器 初始化
 * @note   初始化 UDP 客户端，建立套接字，并配置 Socket 选项。
 * 如果网络未配置或 Socket 创建失败，会在此处拦截并打印错误提示。
 */
void wireless_wave_Init(void)
{
    printf("=========================================\n");
    printf("  UDP -> LoongHost 编码器数据传输\n");
    printf("=========================================\n");
    printf("Target IP:   %s\n", TARGET_IP.c_str());
    printf("Target Port: %d\n", TARGET_PORT);
    printf("Format:      encoder:val1,val2\\n\n");
    printf("=========================================\n");

    // 初始化 UDP 客户端，绑定目标 IP 和端口
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);

    // 获取底层 Socket 文件描述符，进行合法性检查
    int fd = udp_client.get_udp_socket_fd();
    if (fd < 0)
    {
        printf("FATAL: UDP socket 创建失败!\n");
        printf("       请检查网络是否已配置 (ifconfig)\n");
        return;
    }

    // 开启套接字广播权限 (SO_BROADCAST)
    // 注意：即使当前指定了单播 IP，开启此权限也方便后续修改为广播地址 (如 255.255.255.255) 进行全网段发送
    int broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        printf("WARN: SO_BROADCAST 设置失败\n");
    }

    // 标记 UDP 已准备就绪，允许 Loop 函数执行发送逻辑
    udp_ready = true;
    printf("UDP 初始化成功 (socket fd=%d)\n", fd);
    printf("开始发送数据...\n\n");
}

/**
 * @brief  无线数据传输主循环任务
 * @note   周期性读取编码器数据，格式化为特定字符串协议后，通过 UDP 发送至上位机。
 * 建议将此函数放在定时器中断或独立线程中循环调用。
 */
void wireless_wave_Loop(void)
{
    // 如果 UDP 未初始化成功或出现致命错误，直接返回，避免崩溃
    if (!udp_ready)
        return;

    // 采样当前左右编码器的值
    float ch1 = encoder_L_final;
    float ch2 = encoder_R_final;

    // 准备字符串缓冲区，按照上位机 (LoongHost) 要求的格式进行打包
    char encoder_str[64];
    snprintf(encoder_str, sizeof(encoder_str), "ch1:%7.2f,ch2:%7.2f", ch1, ch2);

    // 通过 UDP 发送格式化好的字符串
    ssize_t ret = udp_client.udp_send_string(encoder_str);
    if (ret < 0)
    {
        // 错误处理：使用静态变量记录连续失败次数
        // 最多只打印 5 次错误信息，防止网络断开时终端被报错信息疯狂刷屏导致系统卡顿
        static int fail_cnt = 0;
        if (++fail_cnt <= 5)
            printf("SEND FAIL(ret=%zd)\n", ret);
    }

    // 在本地终端同步打印当前发送的数据，方便本地调试确认
    printf("Encoder: %s\n", encoder_str);

    // 延时 100 毫秒 (即 10Hz 的发送频率)，避免网络发包过快导致拥堵或丢包
    usleep(100 * 1000);
}