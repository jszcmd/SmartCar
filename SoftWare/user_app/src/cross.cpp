#include "cross.h"

// ====================== 0. 外部引用与宏定义 ======================
char cross_state_info[64] = "CROSS_NONE";
extern image_t img_raw;

extern int calc_contrast(image_t *img, int x, int y, int direction);

int CBH_THRESHOLD_FAR = 2000; // 默认值，稍后会被自动覆盖
#define PI 3.1415926535f
#define CLIP(x, low, up) ((x) > (up) ? (up) : ((x) < (low) ? (low) : (x)))

// ====================== 1. 全局变量定义 ======================
enum cross_type_e cross_type = CROSS_NONE;
int cross_enc_count = 0;
bool blind_patch_active = false; // 盲补线激活标志位
// 远线相关变量
int far_Lpt0_found = 0, far_Lpt1_found = 0;
int far_Lpt0_rpts0s_id = 0, far_Lpt1_rpts1s_id = 0;

// 远线过程数组
int far_ipts0[MT9V03X_HH][2], far_ipts1[MT9V03X_HH][2];
int far_ipts0_num = 0, far_ipts1_num = 0;

float far_rpts0[MT9V03X_HH][2], far_rpts1[MT9V03X_HH][2];
int far_rpts0_num = 0, far_rpts1_num = 0;

float far_rpts0b[MT9V03X_HH][2], far_rpts1b[MT9V03X_HH][2];
int far_rpts0b_num = 0, far_rpts1b_num = 0;

float far_rpts0s[MT9V03X_HH][2], far_rpts1s[MT9V03X_HH][2];
int far_rpts0s_num = 0, far_rpts1s_num = 0;

float far_rpts0a[MT9V03X_HH], far_rpts1a[MT9V03X_HH];
float far_rpts0an[MT9V03X_HH], far_rpts1an[MT9V03X_HH];

// 逻辑控制标志
uint8 if_lost_left_line = 0, if_lost_right_line = 0;

// 调试变量定义
float debug_patch_start_x = 0, debug_patch_start_y = 0;
float debug_patch_end_x = 0, debug_patch_end_y = 0;
float debug_patch_dist = 0;
int debug_far_Lpt_id = 0;

// ====================== 2. 核心：非破坏性融合函数 ======================

/**
 * @brief 三段式融合：近线(保留) + 补线(生成) + 远线(拼接)
 * @param target_arr    目标数组 (也是近线源数组)
 * @param target_num    目标数组点数指针
 * @param near_cut_idx  近线保留截止索引 (例如保留 0~near_cut_idx)
 * @param far_arr       远线源数组
 * @param far_num       远线点数
 * @param far_start_idx 远线拼接起始索引
 * @param s_x, s_y      补线起点
 * @param e_x, e_y      补线终点
 */
void fuse_lines(float target_arr[][2], int *target_num, int near_cut_idx,
                float far_arr[][2], int far_num, int far_start_idx,
                float s_x, float s_y, float e_x, float e_y)
{
    // 1. 定义内部临时数组 (防止污染源数据)
    float temp_line[MT9V03X_HH][2];
    int temp_idx = 0;

    // --- 第一阶段：复制近线有效部分 ---
    // 如果 near_cut_idx >= 0，说明有近线需要保留
    if (near_cut_idx >= 0 && near_cut_idx < *target_num)
    {
        for (int i = 0; i <= near_cut_idx; i++)
        {
            if (temp_idx >= MT9V03X_HH)
                break;
            temp_line[temp_idx][0] = target_arr[i][0];
            temp_line[temp_idx][1] = target_arr[i][1];
            temp_idx++;
        }
    }

    // --- 第二阶段：生成中间补线 (Linear Patch) ---
    // 动态计算补线需要的点数，避免点太稀疏或太密集
    // 假设每 2cm (约3-4个像素) 插一个点
    float dist = sqrtf((e_x - s_x) * (e_x - s_x) + (e_y - s_y) * (e_y - s_y));
    int patch_steps = (int)(dist / 3.0f);
    if (patch_steps < 5)
        patch_steps = 5; // 最少插5个点
    if (patch_steps > 30)
        patch_steps = 30; // 最多插30个点

    float dx = (e_x - s_x) / (float)patch_steps;
    float dy = (e_y - s_y) / (float)patch_steps;

    for (int i = 1; i <= patch_steps; i++)
    { // i从1开始，避免与起点重复
        if (temp_idx >= MT9V03X_HH)
            break;
        temp_line[temp_idx][0] = s_x + dx * i;
        temp_line[temp_idx][1] = s_y + dy * i;
        temp_idx++;
    }

    // --- 第三阶段：拼接远线有效部分 ---
    if (far_arr != NULL && far_num > 0)
    {
        for (int i = far_start_idx; i < far_num; i++)
        {
            if (temp_idx >= MT9V03X_HH)
                break;
            temp_line[temp_idx][0] = far_arr[i][0];
            temp_line[temp_idx][1] = far_arr[i][1];
            temp_idx++;
        }
    }

    // --- 第四阶段：原子提交 (写回目标数组) ---
    // 此时 temp_line 是一条完整的、完美的线
    *target_num = temp_idx;
    for (int i = 0; i < temp_idx; i++)
    {
        target_arr[i][0] = temp_line[i][0];
        target_arr[i][1] = temp_line[i][1];
    }
}

int calc_contrast_cross(image_t *img, int x, int y)
{
    if (x < 2 || x > img->width - 3)
        return 0;
    int sum_forward = 0, sum_behind = 0;
    sum_forward = AT_CLIP(img, x, y - 1);
    sum_behind = AT_CLIP(img, x, y);
    int sum = sum_forward + sum_behind;
    if (sum == 0)
        return 0;
    int diff = 0;
    diff = abs(sum_behind - sum_forward);
    return (diff * 10000) / sum;
}

// ====================== 3. 远线通用处理链 ======================
float conf = 0;
void process_far_chain(int side)
{
    int *ipts_num_ptr = (side == 0) ? &far_ipts0_num : &far_ipts1_num;
    int (*ipts)[2] = (side == 0) ? far_ipts0 : far_ipts1;
    float (*rpts)[2] = (side == 0) ? far_rpts0 : far_rpts1;
    int *rpts_num_ptr = (side == 0) ? &far_rpts0_num : &far_rpts1_num;
    float (*rpts_b)[2] = (side == 0) ? far_rpts0b : far_rpts1b;
    float (*rpts_s)[2] = (side == 0) ? far_rpts0s : far_rpts1s;
    int *rpts_s_num = (side == 0) ? &far_rpts0s_num : &far_rpts1s_num;
    float *angle_raw = (side == 0) ? far_rpts0a : far_rpts1a;
    float *angle_nms = (side == 0) ? far_rpts0an : far_rpts1an;
    int *far_found = (side == 0) ? &far_Lpt0_found : &far_Lpt1_found;
    int *far_id = (side == 0) ? &far_Lpt0_rpts0s_id : &far_Lpt1_rpts1s_id;

    // 1. 去畸变 + IPM
    *rpts_num_ptr = *ipts_num_ptr;
    for (int i = 0; i < *ipts_num_ptr; i++)
    {
        rpts[i][0] = Cal_rot_x(ipts[i][0], ipts[i][1]);
        rpts[i][1] = Cal_rot_y(ipts[i][0], ipts[i][1]);
    }

    // 2. 滤波 & 重采样
    int temp_num = *rpts_num_ptr;
    blur_points(rpts, temp_num, rpts_b, line_blur_kernel);
    *rpts_s_num = MT9V03X_HH;
    resample_points(rpts_b, temp_num, rpts_s, rpts_s_num, sample_dist * pixel_per_meter);

    // 3. 角度计算
    int dist_pts = (int)roundf(angle_dist / sample_dist);
    local_angle_points(rpts_s, *rpts_s_num, angle_raw, dist_pts);
    nms_angle(angle_raw, *rpts_s_num, angle_nms, nms_kernel);

    // 4. 找远端 L 角点 (严格筛选逻辑)
    *far_found = 0;

    for (int i = 0; i < MIN(*rpts_s_num, 70); i++)
    {
        if (angle_nms[i] == 0)
            continue;

        int im1 = CLIP(i - dist_pts, 0, *rpts_s_num - 1);
        int ip1 = CLIP(i + dist_pts, 0, *rpts_s_num - 1);

        conf = fabs(angle_raw[i]) - (fabs(angle_raw[im1]) + fabs(angle_raw[ip1])) / 2;

        float angle_min = 20.0f / 180.0f * PI;
        float angle_max = 140.0f / 180.0f * PI;

        if (conf > angle_min && conf < angle_max && i < 40 && i > 1)
        {
            *far_id = i;
            *far_found = 1;
            break;
        }
    }
}

// ====================== 4.搜左远线 ======================
int start0_x = 0, start0_y = 0;
int have_lost_Lpt0_found = 0;
void cross_farline_L()
{
    bool valid_start = false;

    // [情况 A]: 还能看到近端的 L 角点 -> 精准定位
    if (Lpt0_found)
    {
        float rx = rpts0s[CLIP(Lpt0_rpts0s_id, 0, rpts0s_num - 1)][0];
        float ry = rpts0s[CLIP(Lpt0_rpts0s_id, 0, rpts0s_num - 1)][1];

        start0_x = (int)Cal_inv_rot_x(rx, ry);
        start0_y = (int)Cal_inv_rot_y(rx, ry) - 3;

        start0_x = CLIP(start0_x, 5, MT9V03X_W - 6);
        start0_y = CLIP(start0_y, 5, MT9V03X_H - 6);

        valid_start = true;

        // 注意：这里不要修改 rpts0s_num，否则会影响后续融合
        have_lost_Lpt0_found = 1;
        if_lost_left_line = 0;
    }
    // [情况 B]: 盲搜模式
    else if (have_lost_Lpt0_found == 1 || if_lost_left_line == 1)
    {
        if_lost_left_line = 1;

        // === 3点间隔扫描法 ===
        int flat_y_sum = 0;
        int flat_cnt = 0;
        int best_y = BEGIN_Y;

        if (ipts0_num > 10)
        {
            for (int i = ipts0_num - 1; i > 5; i -= 3)
            {
                int y_diff = abs(ipts0[i][1] - ipts0[i - 3][1]);
                if (y_diff <= 3)
                {
                    flat_y_sum += ipts0[i][1];
                    flat_cnt++;
                }
                else
                {
                    if (flat_cnt >= 2)
                        break;
                    flat_cnt = 0;
                    flat_y_sum = 0;
                }
            }
        }

        if (flat_cnt >= 2)
        {
            best_y = (flat_y_sum / flat_cnt) - 8;
        }
        else
        {
            if (ipts0_num > 9)
            {
                best_y = ipts0[ipts0_num - 1][1] - 10;
            }
            else
            {

                best_y = 100;
            }
        }

        start0_x = 25;
        start0_y = CLIP(best_y, 5, MT9V03X_H - 10);
        valid_start = true;
    }

    if (valid_start)
    {
        int found_y = -1;
        int search_limit_y = block_size / 2;
        int current_y = start0_y;

        for (; current_y > start0_y - 60 && current_y > search_limit_y; current_y--)
        {
            int contrast = calc_contrast_cross(&img_raw, start0_x, current_y);
            if (contrast > CBH_THRESHOLD_FAR)
            {
                found_y = current_y;
                break;
            }
        }

        if (found_y != -1)
        {
            int far_block = 7;
            int far_clip = 0;
            far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);
            findline_lefthand_adaptive(&img_raw, far_block, far_clip, start0_x, found_y, far_ipts0, &far_ipts0_num);
        }
        else
        {
            far_ipts0_num = 0;
        }
    }
    else
    {
        far_ipts0_num = 0;
    }
    process_far_chain(0);
}

// ====================== 5.搜右远线 ======================
int start1_x = 0, start1_y = 0;
int have_lost_Lpt1_found = 0;
void cross_farline_R()
{
    bool valid_start = false;

    // [情况 A]: 还能看到近端的右L角点 -> 精准定位
    if (Lpt1_found)
    {
        float rx = rpts1s[CLIP(Lpt1_rpts1s_id, 0, rpts1s_num - 1)][0];
        float ry = rpts1s[CLIP(Lpt1_rpts1s_id, 0, rpts1s_num - 1)][1];

        start1_x = (int)Cal_inv_rot_x(rx, ry);
        start1_y = (int)Cal_inv_rot_y(rx, ry) - 3;

        start1_x = CLIP(start1_x, 5, MT9V03X_W - 6);
        start1_y = CLIP(start1_y, 5, MT9V03X_H - 6);

        valid_start = true;

        have_lost_Lpt1_found = 1;
        if_lost_right_line = 0;
    }
    // [情况 B]: 盲搜模式
    else if (have_lost_Lpt1_found == 1 || if_lost_right_line == 1)
    {
        if_lost_right_line = 1;

        // === 3点间隔扫描法 (与左线对称) ===
        int flat_y_sum = 0;
        int flat_cnt = 0;
        int best_y = BEGIN_Y;

        if (ipts1_num > 10)
        {
            for (int i = ipts1_num - 1; i > 5; i -= 3)
            {
                int y_diff = abs(ipts1[i][1] - ipts1[i - 3][1]);
                if (y_diff <= 3)
                {
                    flat_y_sum += ipts1[i][1];
                    flat_cnt++;
                }
                else
                {
                    if (flat_cnt >= 2)
                        break;
                    flat_cnt = 0;
                    flat_y_sum = 0;
                }
            }
        }

        if (flat_cnt >= 2)
        {
            best_y = (flat_y_sum / flat_cnt) - 8;
        }
        else
        {
            if (ipts1_num > 9)
            {
                best_y = ipts1[ipts1_num - 1][1] - 10;
            }
            else
            {
                best_y = 100;
            }
        }

        start1_x = 135; // 右侧对称位置
        start1_y = CLIP(best_y, 5, MT9V03X_H - 10);
        valid_start = true;
    }

    if (valid_start)
    {
        int found_y = -1;
        int search_limit_y = block_size / 2;
        int current_y = start1_y;

        for (; current_y > start1_y - 60 && current_y > search_limit_y; current_y--)
        {
            int contrast = calc_contrast_cross(&img_raw, start1_x, current_y);
            if (contrast > CBH_THRESHOLD_FAR)
            {
                found_y = current_y;
                break;
            }
        }

        if (found_y != -1)
        {
            int far_block = 7;
            int far_clip = 0;
            far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);
            findline_righthand_adaptive(&img_raw, far_block, far_clip, start1_x, found_y, far_ipts1, &far_ipts1_num);
        }
        else
        {
            far_ipts1_num = 0;
        }
    }
    else
    {
        far_ipts1_num = 0;
    }
    process_far_chain(1);
}

// ====================== 6. 状态机检查 ======================
float dn = 0;
void check_cross(void)
{
    int out_cross = 0;

    switch (cross_type)
    {
    case CROSS_NONE:
        blind_patch_active = false;

        // --- 左角点检测 -> 进入 CROSS_HALF_LEFT ---
        if (Lpt0_found && circle_type == CIRCLE_NONE)
        {
            cross_farline_L();

            if (far_Lpt0_found)
            {
                float dx = far_rpts0s[far_Lpt0_rpts0s_id][0] - rpts0s[Lpt0_rpts0s_id][0];
                float dy = far_rpts0s[far_Lpt0_rpts0s_id][1] - rpts0s[Lpt0_rpts0s_id][1];
                dn = sqrtf(dx * dx + dy * dy);
                if (fabs(dn - 0.35f * pixel_per_meter) < 0.20f * pixel_per_meter)
                {
                    cross_type = CROSS_HALF_LEFT;
                    cross_enc_count = 0;
                    strcpy(cross_state_info, "HALF_LEFT: Confirm In");
                    printf(">>> HALF LEFT CONFIRMED <<<\n");
                    return;
                }
                else
                {
                    strcpy(cross_state_info, "NONE: Lpt0 found but dn mismatch");
                }
            }
            else
            {
                strcpy(cross_state_info, "NONE: Lpt0 found but far_Lpt0 lost");
            }
        }
        // --- 右角点检测 -> 进入 CROSS_HALF_RIGHT (对称逻辑) ---
        else if (Lpt1_found && circle_type == CIRCLE_NONE)
        {
            cross_farline_R();

            if (far_Lpt1_found)
            {
                float dx = far_rpts1s[far_Lpt1_rpts1s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
                float dy = far_rpts1s[far_Lpt1_rpts1s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
                dn = sqrtf(dx * dx + dy * dy);
                if (fabs(dn - 0.35f * pixel_per_meter) < 0.20f * pixel_per_meter)
                {
                    cross_type = CROSS_HALF_RIGHT;
                    cross_enc_count = 0;
                    strcpy(cross_state_info, "HALF_RIGHT: Confirm In");
                    printf(">>> HALF RIGHT CONFIRMED <<<\n");
                    return;
                }
                else
                {
                    strcpy(cross_state_info, "NONE: Lpt1 found but dn mismatch");
                }
            }
            else
            {
                strcpy(cross_state_info, "NONE: Lpt1 found but far_Lpt1 lost");
            }
        }
        else
        {
            strcpy(cross_state_info, "CROSS_NONE: Normal Track");
        }
        break;

    case CROSS_HALF_LEFT:
        // 盲补激活条件：角点丢失或角点逼近
        if (!Lpt0_found || (Lpt0_found && Lpt0_rpts0s_id < 15))
        {
            blind_patch_active = true;
        }

        // 退出条件：左线恢复足够长
        if (ipts0_num > 70)
        {
            out_cross = 1;
        }

        if (out_cross == 1)
        {
            cross_type = CROSS_NONE;

            for (int i = 0; i < far_ipts0_num; i++)
            {
                far_ipts0[i][0] = 0;
                far_ipts0[i][1] = 0;
            }
            for (int i = 0; i < far_ipts1_num; i++)
            {
                far_ipts1[i][0] = 0;
                far_ipts1[i][1] = 0;
            }
            if_lost_left_line = 0;
            have_lost_Lpt0_found = 0;
            blind_patch_active = false;
            strcpy(cross_state_info, "CROSS_NONE: Exit HALF_LEFT");
            printf(">>> HALF LEFT EXIT <<<\n");
        }
        else
        {
            if (blind_patch_active)
                strcpy(cross_state_info, "HALF_LEFT: BLIND PATCHING");
            else
                strcpy(cross_state_info, "HALF_LEFT: Running");
        }
        break;

    case CROSS_HALF_RIGHT:
        // 盲补激活条件：角点丢失或角点逼近 (与左十字对称)
        if (!Lpt1_found || (Lpt1_found && Lpt1_rpts1s_id < 15))
        {
            blind_patch_active = true;
        }

        // 退出条件：右线恢复足够长 (与左十字对称)
        if (ipts1_num > 70)
        {
            out_cross = 1;
        }

        if (out_cross == 1)
        {
            cross_type = CROSS_NONE;

            for (int i = 0; i < far_ipts0_num; i++)
            {
                far_ipts0[i][0] = 0;
                far_ipts0[i][1] = 0;
            }
            for (int i = 0; i < far_ipts1_num; i++)
            {
                far_ipts1[i][0] = 0;
                far_ipts1[i][1] = 0;
            }
            if_lost_right_line = 0;
            have_lost_Lpt1_found = 0;
            blind_patch_active = false;
            strcpy(cross_state_info, "CROSS_NONE: Exit HALF_RIGHT");
            printf(">>> HALF RIGHT EXIT <<<\n");
        }
        else
        {
            if (blind_patch_active)
                strcpy(cross_state_info, "HALF_RIGHT: BLIND PATCHING");
            else
                strcpy(cross_state_info, "HALF_RIGHT: Running");
        }
        break;

    default:
        cross_type = CROSS_NONE;
        break;
    }
}

// ====================== 7. 状态机执行 (使用 fuse_lines) ======================
void run_cross(void)
{
    // 正常状态清空调试信息
    if (cross_type == CROSS_NONE)
    {
        debug_patch_dist = 0;
        debug_patch_start_x = 0;
        debug_patch_start_y = 0;
        debug_patch_end_x = 0;
        debug_patch_end_y = 0;
    }

    // ==================== 左十字补线 ====================
    if (cross_type == CROSS_HALF_LEFT)
    {
        // 既然是左十字，必须强制只跟左线跑
        track_type = 1;
        cross_farline_L();

        // 屏蔽无关的右线数据，防干扰
        far_ipts1_num = 0;
        far_rpts1_num = 0;
        far_rpts1s_num = 0;
        far_Lpt1_found = 0;

        if (far_rpts0s_num > 5)
        {
            float start_x = 0, start_y = 0;
            float end_x = 0, end_y = 0;
            int near_cut_idx = -1; // 近线保留截止索引
            int far_start_idx = 0; // 远线开始拼接索引

            // --- 1. 确定起点 & 近线截断点 ---

            // [补线核心 A]：盲补线已激活，且近端原图左线完全丢失 -> 启用红线法线推导
            if (blind_patch_active && ipts0_num < 5)
            {
                if (far_Lpt0_found && far_rpts0s_num > 3)
                {
                    // 1. 获取图二中“红线”（横向边界）的向量
                    // 红线的起点是 far_rpts0s[0]，终点是角点 far_Lpt0_rpts0s_id
                    int p_base = 0;
                    int p_corner = far_Lpt0_rpts0s_id;

                    // 容错：如果角点找得太靠前(比如在第1、2个点)，强行拉开距离保证方向计算稳定
                    if (p_corner < 3)
                        p_corner = (far_rpts0s_num > 3) ? 3 : (far_rpts0s_num - 1);

                    // 计算红线向量
                    float dx = far_rpts0s[p_corner][0] - far_rpts0s[p_base][0];
                    float dy = far_rpts0s[p_corner][1] - far_rpts0s[p_base][1];

                    // 2. 求红线的法向量 (左边线，逆时针旋转90度)
                    float nx = -dy;
                    float ny = dx;
                    // 强制保证法线朝向屏幕下方 (因为我们要往下投射，Y轴向下为正)
                    if (ny < 0)
                    {
                        nx = -nx;
                        ny = -ny;
                    }

                    // 3. 归一化法向量
                    float len = sqrtf(nx * nx + ny * ny);
                    if (len > 0.001f)
                    {
                        nx /= len;
                        ny /= len;
                    }
                    else
                    {
                        nx = 0;
                        ny = 1.0f;
                    }

                    // 4. 让这条法线经过真实的远角点，向下投射到 Y = 110
                    if (ny < 0.01f)
                        ny = 0.01f; // 防止除零异常

                    int actual_corner_id = far_Lpt0_rpts0s_id; // 法线必须穿过真正的角点
                    float t = (110.0f - far_rpts0s[actual_corner_id][1]) / ny;

                    start_x = far_rpts0s[actual_corner_id][0] + nx * t;
                    start_y = 110.0f;
                }
                else
                {
                    // 兜底：远线也没角点，直接从远线根部拉到底
                    start_x = far_rpts0s[0][0];
                    start_y = 110;
                }
                near_cut_idx = -1; // 近线已丢失，不截断，全部靠补
            }
            // [补线核心 B]：盲补线已激活，但近线还有点 -> 强制将起点压在数组第0个点
            else if (blind_patch_active)
            {
                start_x = rpts0s[0][0];
                start_y = rpts0s[0][1];
                near_cut_idx = 0;
            }
            // [补线核心 C]：正常状态 -> 从近端角点或近线末端起补
            else if (ipts0_num > 5)
            {
                if (Lpt0_found && Lpt0_rpts0s_id < rpts0s_num)
                {
                    start_x = rpts0s[Lpt0_rpts0s_id][0];
                    start_y = rpts0s[Lpt0_rpts0s_id][1];
                    near_cut_idx = Lpt0_rpts0s_id; // 保留至角点
                }
                else
                {
                    start_x = rpts0s[rpts0s_num - 1][0];
                    start_y = rpts0s[rpts0s_num - 1][1];
                    near_cut_idx = rpts0s_num - 1; // 保留至末端
                }
            }
            // 兜底
            else
            {
                start_x = far_rpts0s[0][0];
                start_y = 110;
                near_cut_idx = -1;
            }

            // --- 2. 确定终点 & 远线起始点 ---
            if (far_Lpt0_found && far_Lpt0_rpts0s_id < far_rpts0s_num)
            {
                end_x = far_rpts0s[far_Lpt0_rpts0s_id][0];
                end_y = far_rpts0s[far_Lpt0_rpts0s_id][1];
                far_start_idx = far_Lpt0_rpts0s_id; // 远线从角点开始接
                debug_far_Lpt_id = far_Lpt0_rpts0s_id;
            }
            else
            {
                end_x = far_rpts0s[0][0];
                end_y = far_rpts0s[0][1];
                far_start_idx = 0; // 远线从头开始接
            }

            // --- 3. 执行融合 ---
            debug_patch_start_x = start_x;
            debug_patch_start_y = start_y;
            debug_patch_end_x = end_x;
            debug_patch_end_y = end_y;
            debug_patch_dist = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));

            // 调用三段融合函数
            fuse_lines(rpts0s, &rpts0s_num, near_cut_idx,
                       far_rpts0s, far_rpts0s_num, far_start_idx,
                       start_x, start_y, end_x, end_y);
        }
    }

    // ==================== 右十字补线 (与左十字完全对称) ====================
    else if (cross_type == CROSS_HALF_RIGHT)
    {
        track_type = 2; // 右十字强制跟右线
        cross_farline_R();

        // 屏蔽无关的左线数据，防干扰
        far_ipts0_num = 0;
        far_rpts0_num = 0;
        far_rpts0s_num = 0;
        far_Lpt0_found = 0;

        if (far_rpts1s_num > 5)
        {
            float start_x = 0, start_y = 0;
            float end_x = 0, end_y = 0;
            int near_cut_idx = -1;
            int far_start_idx = 0;

            // --- 1. 确定起点 & 近线截断点 ---

            // [补线核心 A]：盲补线已激活，且近端原图右线完全丢失 -> 启用法线推导
            if (blind_patch_active && ipts1_num < 5)
            {
                if (far_Lpt1_found && far_rpts1s_num > 3)
                {
                    // 与左侧对称：从远线根部到角点的向量
                    int p_base = 0;
                    int p_corner = far_Lpt1_rpts1s_id;

                    // 容错：角点太靠前时强行拉开距离
                    if (p_corner < 3)
                        p_corner = (far_rpts1s_num > 3) ? 3 : (far_rpts1s_num - 1);

                    // 计算远线向量
                    float dx = far_rpts1s[p_corner][0] - far_rpts1s[p_base][0];
                    float dy = far_rpts1s[p_corner][1] - far_rpts1s[p_base][1];

                    // 求法向量 (右边线，顺时针旋转90度)
                    float nx = dy;
                    float ny = -dx;
                    // 强制保证法线朝向屏幕下方
                    if (ny < 0)
                    {
                        nx = -nx;
                        ny = -ny;
                    }

                    // 归一化法向量
                    float len = sqrtf(nx * nx + ny * ny);
                    if (len > 0.001f)
                    {
                        nx /= len;
                        ny /= len;
                    }
                    else
                    {
                        nx = 0;
                        ny = 1.0f;
                    }

                    // 让法线经过真实的远角点，向下投射到 Y = 110
                    if (ny < 0.01f)
                        ny = 0.01f;

                    int actual_corner_id = far_Lpt1_rpts1s_id;
                    float t = (110.0f - far_rpts1s[actual_corner_id][1]) / ny;

                    start_x = far_rpts1s[actual_corner_id][0] + nx * t;
                    start_y = 110.0f;
                }
                else
                {
                    // 兜底：远线也没角点，直接从远线根部拉到底
                    start_x = far_rpts1s[0][0];
                    start_y = 110;
                }
                near_cut_idx = -1;
            }
            // [补线核心 B]：盲补线已激活，但近线还有点
            else if (blind_patch_active)
            {
                start_x = rpts1s[0][0];
                start_y = rpts1s[0][1];
                near_cut_idx = 0;
            }
            // [补线核心 C]：正常状态 -> 从近端角点或近线末端起补
            else if (ipts1_num > 5)
            {
                if (Lpt1_found && Lpt1_rpts1s_id < rpts1s_num)
                {
                    start_x = rpts1s[Lpt1_rpts1s_id][0];
                    start_y = rpts1s[Lpt1_rpts1s_id][1];
                    near_cut_idx = Lpt1_rpts1s_id;
                }
                else
                {
                    start_x = rpts1s[rpts1s_num - 1][0];
                    start_y = rpts1s[rpts1s_num - 1][1];
                    near_cut_idx = rpts1s_num - 1;
                }
            }
            // 兜底
            else
            {
                start_x = far_rpts1s[0][0];
                start_y = 110;
                near_cut_idx = -1;
            }

            // --- 2. 确定终点 & 远线起始点 ---
            if (far_Lpt1_found && far_Lpt1_rpts1s_id < far_rpts1s_num)
            {
                end_x = far_rpts1s[far_Lpt1_rpts1s_id][0];
                end_y = far_rpts1s[far_Lpt1_rpts1s_id][1];
                far_start_idx = far_Lpt1_rpts1s_id;
                debug_far_Lpt_id = far_Lpt1_rpts1s_id;
            }
            else
            {
                end_x = far_rpts1s[0][0];
                end_y = far_rpts1s[0][1];
                far_start_idx = 0;
            }

            // --- 3. 执行融合 ---
            debug_patch_start_x = start_x;
            debug_patch_start_y = start_y;
            debug_patch_end_x = end_x;
            debug_patch_end_y = end_y;
            debug_patch_dist = sqrtf((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));

            fuse_lines(rpts1s, &rpts1s_num, near_cut_idx,
                       far_rpts1s, far_rpts1s_num, far_start_idx,
                       start_x, start_y, end_x, end_y);
        }
    }
}