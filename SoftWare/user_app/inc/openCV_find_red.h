#ifndef _OPENCV_FIND_RED_HPP_
#define _OPENCV_FIND_RED_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

typedef struct
{
    bool found;         // 是否找到
    cv::Rect red_rect;  // 红色块的矩形框
    cv::Mat target_roi; // 红色块上方扣出来的目标图片

    // 用于向上位机透传的实时底层数据
    double max_area;
    float max_ratio;
    int max_width;
} Red_Target_t;

Red_Target_t Find_Target_Board(const cv::Mat &input_frame);

#endif