#include "target.h"

int64_t total_encoder;

// 用于给 TCP 界面显示的数据接口 ===
bool debug_target_found = false;
int debug_roi_x = 0;
int debug_roi_y = 0;
int debug_roi_w = 0;

int target_red_x = 0;
int target_red_y = 0;
int target_red_w = 0;
int target_red_h = 0;

// 状态定义
enum target_state_e target_state = TARGET_NONE;
int target_override_track_type = -1;
float target_shift_offset = 0.0f;

// 定义用于显示的全局变量
double target_debug_area = 0;
float target_debug_ratio = 0;
int target_debug_width = 0;

void check_target_logic(void)
{
    // 1. 无条件运行视觉检测
    Red_Target_t res = Find_Target_Board(detect);

    // 一抓到数据就立马存到全局变量里，供 image.cpp 打印
    target_debug_area = res.max_area;
    target_debug_ratio = res.max_ratio;
    target_debug_width = res.max_width;

    static int64_t memory_start_encoder = 0;
    static int last_valid_x = 0;
    static int last_valid_y = 0;
    static int last_valid_w = 0;
    static int last_valid_h = 0;

    const int HOLD_ENCODER_DIST = 5000;

    // 如果处于十字或圆环状态，强行不执行避障
    if (circle_type != CIRCLE_NONE || cross_type != CROSS_NONE)
    {
        target_state = TARGET_NONE;
        target_override_track_type = -1;
        target_shift_offset = 0.0f;
        debug_target_found = false;
        return;
    }

    // ==============================================================
    // 状态 1：视野里清晰看到红框 (刷新记忆)
    // ==============================================================
    if (res.found && res.red_rect.width > 5)
    {
        target_state = TARGET_FOUND;
        memory_start_encoder = total_encoder;

        // 记住最后的完整物理坐标
        last_valid_x = res.red_rect.x;
        last_valid_y = res.red_rect.y;
        last_valid_w = res.red_rect.width;
        last_valid_h = res.red_rect.height;

        // 测试写死: 1代表右绕
        int class_id = 1;

        if (class_id == 0)
        {
            target_override_track_type = 1; // 左绕
            target_shift_offset = 25.0f;
        }
        else if (class_id == 1)
        {
            target_override_track_type = 2; // 右绕
            target_shift_offset = 25.0f;
        }

        // 正常更新接口数据
        debug_target_found = true;
        debug_roi_w = res.red_rect.width;
        debug_roi_x = res.red_rect.x;
        debug_roi_y = res.red_rect.y - debug_roi_w;
        if (debug_roi_y < 0)
            debug_roi_y = 0;

        target_red_x = res.red_rect.x;
        target_red_y = res.red_rect.y;
        target_red_w = res.red_rect.width;
        target_red_h = res.red_rect.height;
    }
    // ==============================================================
    // 状态 2：视野里丢失红框
    // ==============================================================
    else
    {

        if ((target_state == TARGET_FOUND || target_state == TARGET_PASSING) &&
            std::abs(total_encoder - memory_start_encoder) < HOLD_ENCODER_DIST)
        {
            target_state = TARGET_PASSING;

            debug_target_found = true;
            target_red_x = last_valid_x;
            target_red_y = last_valid_y;
            target_red_w = last_valid_w;
            target_red_h = last_valid_h;

            debug_roi_x = last_valid_x;
            debug_roi_w = last_valid_w;
            debug_roi_y = last_valid_y - last_valid_w;
            if (debug_roi_y < 0)
                debug_roi_y = 0;
        }
        // 已经跑够了安全距离，彻底恢复直行
        else
        {
            target_state = TARGET_NONE;
            target_override_track_type = -1;
            target_shift_offset = 0.0f;

            debug_target_found = false;
            debug_roi_w = 0;
            target_red_x = 0;
            target_red_y = 0;
            target_red_w = 0;
            target_red_h = 0;
        }
    }
}