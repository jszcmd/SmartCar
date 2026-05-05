#include "circle.h"

// ====================== 全局变量 ======================
enum circle_type_e circle_type = CIRCLE_NONE;
int track_type = 0; // 0: 混合/左优先, 1: 强制左线, 2: 强制右线

// 计数器与标志位
int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

// ====================== 1. 圆环检测 (Check) ======================
void check_circle(void)
{
    // 只有在非十字、非圆环状态下检测
    if (circle_type != CIRCLE_NONE || cross_type != CROSS_NONE)
        return;

    // [左圆环检测]
    // 条件：右侧长直道 && 左侧出现L角点 (且角点比较近 < 25)
    if (Lpt0_found && !Lpt1_found && is_straight1 && Lpt0_rpts0s_id < 25)
    {
        circle_type = CIRCLE_LEFT_BEGIN;
        none_left_line = 0;
        have_left_line = 0;
        printf(">>> CIRCLE LEFT BEGIN <<<\n");
    }

    // [右圆环检测]
    // 条件：左侧长直道 && 右侧出现L角点
    else if (!Lpt0_found && Lpt1_found && is_straight0 && Lpt1_rpts1s_id < 25)
    {
        circle_type = CIRCLE_RIGHT_BEGIN;
        none_right_line = 0;
        have_right_line = 0;
        printf(">>> CIRCLE RIGHT BEGIN <<<\n");
    }
}

// ====================== 2. 圆环执行 (Run) ======================
void run_circle(void)
{
    // ================== 左圆环状态机 ==================
    if (circle_type == CIRCLE_LEFT_BEGIN)
    {
        track_type = 2; // 强制巡右线 (外直道)

        // 逻辑：先丢左线 -> 后有左线 -> 且距离满足 -> 入环
        if (rpts0s_num < 2 && !Lpt0_found)
        {
            none_left_line++;
            have_left_line = 0;
        }
        if (rpts0s_num > 30 && none_left_line)
        {
            have_left_line++;
        }

        if (have_left_line > 2)
        {
            circle_type = CIRCLE_LEFT_IN;
            none_left_line = 0;
            have_left_line = 0;
            printf(">>> CIRCLE LEFT IN <<<\n");
        }
    }
    else if (circle_type == CIRCLE_LEFT_IN)
    {
        track_type = 1; // 切换巡左线 (内圆内侧线)
        // cross_farline_L(); // 开启左远线搜索辅助

        if (rpts1s_num < 5)
            none_right_line++;
        if (rpts1s_num > 25 && none_right_line > 1)
        {
            circle_type = CIRCLE_LEFT_RUNNING;
            none_right_line = 0;
            printf(">>> CIRCLE LEFT RUNNING <<<\n");
        }
    }
    else if (circle_type == CIRCLE_LEFT_RUNNING)
    {
        track_type = 2; // 环内巡右线 (外圆内侧线)

        if (Lpt1_found && Lpt1_rpts1s_id < (int)(0.7f / sample_dist))
        {
            circle_type = CIRCLE_LEFT_OUT;
            none_right_line = 0;
            printf(">>> CIRCLE LEFT OUT <<<\n");
        }
    }
    else if (circle_type == CIRCLE_LEFT_OUT)
    {
        track_type = 1; // 切换巡左线
        // cross_farline_R(); // 补线逻辑

        if (rpts1s_num < 5)
            none_right_line++;
        if ((rpts1s_num > 30 && !Lpt1_found && none_right_line > 1))
        {
            circle_type = CIRCLE_LEFT_END;
            printf(">>> CIRCLE LEFT END <<<\n");
        }
    }
    else if (circle_type == CIRCLE_LEFT_END)
    {
        track_type = 2; // 强制右线
        static int end_count = 0;
        end_count++;
        if (end_count > 50)
        {
            circle_type = CIRCLE_NONE;
            track_type = 0; // 恢复默认
            end_count = 0;
            printf(">>> CIRCLE FINISHED <<<\n");
        }
    }

    // ================== 右圆环状态机 ==================
    else if (circle_type == CIRCLE_RIGHT_BEGIN)
    {
        track_type = 1; // 强制巡左线
        if (rpts1s_num < 2 && !Lpt1_found)
        {
            none_right_line++;
            have_right_line = 0;
        }
        if (rpts1s_num > 30 && none_right_line)
        {
            have_right_line++;
        }

        if (have_right_line > 2)
        {
            circle_type = CIRCLE_RIGHT_IN;
            none_right_line = 0;
            printf(">>> CIRCLE RIGHT IN <<<\n");
        }
    }
    else if (circle_type == CIRCLE_RIGHT_IN)
    {
        track_type = 2; // 切换巡右线
        cross_farline_R();
        if (rpts0s_num < 5)
            none_left_line++;
        if (rpts0s_num > 25 && none_left_line > 1)
        {
            circle_type = CIRCLE_RIGHT_RUNNING;
            none_left_line = 0;
            printf(">>> CIRCLE RIGHT RUNNING <<<\n");
        }
    }
    else if (circle_type == CIRCLE_RIGHT_RUNNING)
    {
        track_type = 1; // 环内巡左线
        if (Lpt0_found && Lpt0_rpts0s_id < (int)(0.7f / sample_dist))
        {
            circle_type = CIRCLE_RIGHT_OUT;
            printf(">>> CIRCLE RIGHT OUT <<<\n");
        }
    }
    else if (circle_type == CIRCLE_RIGHT_OUT)
    {
        track_type = 2; // 切换巡右线
        cross_farline_L();
        if (rpts0s_num < 5)
            none_left_line++;
        if ((rpts0s_num > 30 && !Lpt0_found && none_left_line > 1))
        {
            circle_type = CIRCLE_RIGHT_END;
            printf(">>> CIRCLE RIGHT END <<<\n");
        }
    }
    else if (circle_type == CIRCLE_RIGHT_END)
    {
        track_type = 1; // 强制左线
        static int end_count = 0;
        end_count++;
        if (end_count > 50)
        {
            circle_type = CIRCLE_NONE;
            track_type = 0;
            end_count = 0;
            printf(">>> CIRCLE FINISHED <<<\n");
        }
    }
    else
    {
        track_type = 0; // 默认
    }
}