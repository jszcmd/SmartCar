#include "openCV_find_red.h"

using namespace cv;
using namespace std;

Red_Target_t Find_Target_Board(const Mat &input_frame)
{
    Red_Target_t result;
    result.found = false;

    Mat lab, a_channel, b_channel, mask_a, mask_b, mask;
    vector<Mat> channels;

    cvtColor(input_frame, lab, COLOR_BGR2Lab);
    split(lab, channels);
    a_channel = channels[1];
    b_channel = channels[2];

    threshold(a_channel, mask_a, 141, 255, THRESH_BINARY);

    threshold(b_channel, mask_b, 118, 255, THRESH_BINARY);

    bitwise_and(mask_a, mask_b, mask);
    // ==========================================

    static const Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    Rect max_rect;
    bool found_contours = false;

    // --- DEBUG 专用变量 ---
    double debug_max_area = 0;
    float debug_ratio = 0.0f;
    Rect debug_rect;

    for (const auto &cnt : contours)
    {
        double area = contourArea(cnt);
        Rect r = boundingRect(cnt);
        float ratio = (float)r.width / (float)r.height;
        // --- DEBUG 专用输出 ---
        if (area > debug_max_area)
        {
            debug_max_area = area;
            debug_ratio = ratio;
            debug_rect = r;
        }

        if (area > 35 && area > max_area && ratio > 0.8f)
        {
            max_area = area;
            max_rect = r;
            found_contours = true;
        }
    }

    static int debug_counter = 0;
    debug_counter++;
    Mat debug_draw = input_frame.clone();
    // if (debug_counter % 15 == 0)
    // {
    // printf("[CV Debug] MaxArea: %5.0f | Ratio: %4.2f | X:%3d Y:%3d | Strict Found: %d\n",
    //        debug_max_area, debug_ratio, debug_rect.x, debug_rect.y, found_contours);
    // fflush(stdout);

    // if (debug_max_area > 50)
    // {
    rectangle(debug_draw, debug_rect, Scalar(0, 0, 255), 1);

    if (found_contours)
    {
        rectangle(debug_draw, max_rect, Scalar(0, 255, 0), 1);
    }
    // }
    // }
    cv::imshow("red", debug_draw);
    // ==========================================

    // --- 目标裁剪与打包输出 ---
    if (found_contours)
    {
        result.found = true;
        result.red_rect = max_rect;
    }

    else
    {
        result.found = true;
    }

    //     int crop_size = max_rect.width;

    //     // 防越界保护：兜底顶边
    //     int roi_y = max_rect.y - crop_size;
    //     if (roi_y < 0)
    //         roi_y = 0;

    //     int roi_x = max_rect.x;
    //     // 防越界保护：兜底右边
    //     if (roi_x + crop_size > input_frame.cols)
    //     {
    //         crop_size = input_frame.cols - roi_x;
    //     }

    //     // 只要能裁出来一块图，就送进去
    //     if (crop_size > 10 && max_rect.y > roi_y)
    //     {
    //         // Rect target_rect(roi_x, roi_y, crop_size, max_rect.y - roi_y);
    //         // result.target_roi = input_frame(target_rect).clone();

    //         // // 把最终送给模型的图也存一份看看裁得对不对
    //         // if (debug_counter % 15 == 0)
    //         // {
    //         //     imwrite("/home/lsy/LS2K0300_Library/debug_roi.jpg", result.target_roi);
    //         // }
    //     }
    //     else
    //     {
    //         result.found = false;
    //     }
    //}

    result.max_area = debug_max_area;
    result.max_ratio = debug_ratio;
    result.max_width = debug_rect.width;

    return result;
}