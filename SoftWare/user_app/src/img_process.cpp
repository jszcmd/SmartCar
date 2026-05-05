/*********************************************************************************************************************
 * 文件名称：img_process.cpp
 * 功能描述：摄像头初始化、图传管理、图像处理与中线误差计算
 ********************************************************************************************************************/

#include "img_process.h"

using namespace cv;
using namespace std;

// ══════════════════════════════════════════════════════════════
// 全局图像变量定义
// ══════════════════════════════════════════════════════════════
cv::Mat img;        // 摄像头原始彩色图像
cv::Mat img_gray;   // 灰度图像
cv::Mat black_img;  // 纯黑背景图（用于绘制辅助线）
cv::Mat detect;     // resize后的检测图像
cv::Mat img_binary; // 二值化图像

// ══════════════════════════════════════════════════════════════
// 外部变量声明
// ══════════════════════════════════════════════════════════════
extern float Guide;          // 中线偏差值，由 image_handle 更新
extern cv::Mat img_nitoushi; // 逆透视图像，由 image_handle 更新

// ══════════════════════════════════════════════════════════════
// 全局指针定义
// ══════════════════════════════════════════════════════════════
Camera *g_camera = nullptr;
TransmissionStreamServer *g_server = nullptr;

// ══════════════════════════════════════════════════════════════
// 摄像头 + 图传初始化
// 只在程序启动时调用一次
// ══════════════════════════════════════════════════════════════
bool camera_server_init(int width, int height, int fps)
{
    // ── 初始化摄像头 ──────────────────────────────────────────
    g_camera = new Camera();
    if (g_camera->init(width, height, fps) != 0)
    {
        printf("摄像头初始化失败\n");
        delete g_camera;
        g_camera = nullptr;
        return false;
    }
    printf("摄像头初始化成功: %dx%d @ %dfps\n", width, height, fps);

    // ── 启动图传服务器 ────────────────────────────────────────
    g_server = new TransmissionStreamServer();
    if (g_server->start_server(8080) != 0)
    {
        // 图传启动失败不影响摄像头正常工作，只打印警告继续运行
        printf("图传服务器启动失败，警告：图传不可用，继续运行\n");
        delete g_server;
        g_server = nullptr;
    }
    else
    {
        printf("图传服务器启动成功，访问 http://<板子IP>:8080\n");
    }

    return true;
}

// ══════════════════════════════════════════════════════════════
// 释放摄像头和图传资源
// 程序退出时调用
// ══════════════════════════════════════════════════════════════
void camera_server_deinit()
{
    if (g_server != nullptr)
    {
        g_server->stop_server();
        delete g_server;
        g_server = nullptr;
    }

    if (g_camera != nullptr)
    {
        delete g_camera;
        g_camera = nullptr;
    }

    printf("摄像头和图传已释放\n");
}

// ══════════════════════════════════════════════════════════════
// 零拷贝图传（推荐）
// 直接使用摄像头的JPEG原始数据推流，无需解码再编码
// 适合只看原图的场景，性能比 camera_transmit 高50~100倍
// ══════════════════════════════════════════════════════════════
void camera_transmit_raw()
{
    if (g_server == nullptr || !g_server->is_running())
        return;
    g_server->update_frame_jpeg(g_camera->jpeg_nowdata);
}

// ══════════════════════════════════════════════════════════════
// 普通图传
// 传入已经处理过的 Mat 图像（如画了辅助线、逆透视图等）
// ══════════════════════════════════════════════════════════════
void camera_transmit(const cv::Mat &frame)
{
    if (g_server == nullptr || !g_server->is_running())
        return;
    g_server->update_frame_mat(frame);
}

// ══════════════════════════════════════════════════════════════
// 获取中线误差
// 每个控制周期调用一次，内部只处理一帧，处理完立即返回
// 不能有死循环，循环的事情由调用方（主循环/定时器）负责
// ══════════════════════════════════════════════════════════════
float get_errors()
{
    // 摄像头未初始化时直接返回0，不崩溃
    if (g_camera == nullptr)
        return 0;

    // ── 取一帧 ────────────────────────────────────────────────
    if (!g_camera->capture_frame(img))
    {
        printf("摄像头捕获失败\n");
        return 0;
    }
    if (img.empty())
        return 0;

    // ── 图像翻转（摄像头安装方向是反的，上下左右都翻转）────────
    // flip 参数 -1 = 同时翻转X轴和Y轴，等效于旋转180度
    cv::flip(img, img, -1);

    // ── 图像缩放（统一到处理分辨率）─────────────────────────────
    cv::resize(img, detect, cv::Size(UVC_WIDTH, UVC_HEIGHT));

    // ── 灰度化 ────────────────────────────────────────────────
    cvtColor(img, img_gray, COLOR_BGR2GRAY);

    // ── 自适应二值化（OTSU自动计算阈值）──────────────────────────
    threshold(img_gray, img_binary, 0, 255, THRESH_BINARY | THRESH_OTSU);

    // ── 图像处理核心（中线提取，结果写入 Guide 和 img_nitoushi）──
    image_handle(img_gray);

    // ── 绘制辅助线 ────────────────────────────────────────────
    black_img = Mat::zeros(cv::Size(160, 120), CV_8UC1);
    image_display_opencv(black_img);

    // 在第80行（100-20）画一条白色水平参考线
    for (int i = 0; i < 160; i++)
    {
        img_gray.at<uchar>(100 - 20, i) = 255;
    }

    // ── 图传 ──────────────────────────────────────────────────
    // 优先传逆透视图（更直观），为空时降级传灰度图
    if (!img_nitoushi.empty())
        camera_transmit(img_nitoushi);
    else
        camera_transmit(img_gray);

    // 返回中线偏差值（由 image_handle 写入 Guide）
    return Guide;
}