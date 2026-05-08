#include "main.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

extern std::atomic<bool> ls_system_running;

int main(void)
{
    // ══════════════════════════════════════════════
    // 1. 硬件外设初始化
    // ══════════════════════════════════════════════
    // Motor_Init();
    // encoder_Init();
    // wireless_wave_Init();
    tft18_init();

    // ══════════════════════════════════════════════
    // 2. 摄像头 + 图传初始化
    // ══════════════════════════════════════════════
    if (!camera_server_init(160, 120, 120))
    {
        printf("摄像头初始化失败，程序退出\n");
        return -1;
    }

    printf("系统启动成功，开始测试摄像头...\n");

    // ══════════════════════════════════════════════
    // 3. 主循环（测试用，暂不启动定时器）
    // ══════════════════════════════════════════════
    while (ls_system_running)
    {
        // 取一帧，处理，返回中线偏差
        float mid_error = get_errors();

        // 打印偏差值，观察是否正常
        printf("mid_error = %.2f\n", mid_error);
    }

    // ══════════════════════════════════════════════
    // 4. 退出清理
    // ══════════════════════════════════════════════
    camera_server_deinit();
    printf("系统安全退出\n");

    return 0;
}
