#include "image.h"

#ifdef MIN
#undef MIN
#endif
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#ifndef PI
#define PI 3.1415926535f
#endif

// 颜色定义 (RGB565)
#ifndef RGB565_RED
#define RGB565_RED 0xF800
#define RGB565_GREEN 0x07E0
#define RGB565_BLUE 0x001F
#define RGB565_YELLOW 0xFFE0
#define RGB565_CYAN 0x07FF
#define RGB565_MAGENTA 0xF81F
#define RGB565_WHITE 0xFFFF
#define RGB565_BLACK 0x0000
#define RGB565_GRAY 0x8410
#endif

// 显示坐标配置 (IPS200用的)
#define RAW_Y 0
#define IPM_Y 140
#define TXT_Y 280

// ====================== 全局变量 ======================

cv::Mat img_nitoushi; // 逆透视的图像，用来显示。。。

float rptsn[MT9V03X_HH][2];
int rptsn_num = 0;
float pure_angle = 0.0f;
float aim_distance = 0.55f;

#define CBH_MULTIPLIER 10000
int CBH_THRESHOLD = 2000; // 默认值,稍后会被自动整定覆盖
#define CBH_STEP 2

int Lpt0_found = 0;
int Lpt1_found = 0;
int Lpt0_rpts0s_id = 0;
int Lpt1_rpts1s_id = 0;
bool is_straight0 = false, is_straight1 = false;
bool Ypt0_found = false, Ypt1_found = false;
int Ypt0_rpts0s_id = 0, Ypt1_rpts1s_id = 0;

// 用于缓存真实的角点物理坐标，防止被补线函数篡改
float true_Lpt0_x = 0, true_Lpt0_y = 0;
float true_Lpt1_x = 0, true_Lpt1_y = 0;

extern int cross_enc_count;
extern int far_ipts0[MT9V03X_HH][2];
extern int far_ipts1[MT9V03X_HH][2];
extern int far_ipts0_num;
extern int far_ipts1_num;
extern int far_Lpt0_rpts0s_id;
extern int far_Lpt1_rpts1s_id;

int block_size = 15;
int clip_value = 3;
float sample_dist = 0.02f;
int pixel_per_meter = 55;
float angle_dist = 0.1f;
int line_blur_kernel = 7;
int nms_kernel = 5;
float ROAD_WIDTH = 0.45f;

float rot[3][3] = {{-9.3068f, -0.0000f, 279.2050f}, {-5.3127f, -1.2942f, 183.5385f}, {-0.0664f, -0.0000f, 1.0000f}};
float inv_rot[3][3] = {{-0.1074f, -0.0000f, 30.0000f}, {-0.5708f, 0.7667f, 18.6667f}, {-0.0071f, -0.0000f, 1.0000f}};

int ipts0[MT9V03X_HH][2];
int ipts1[MT9V03X_HH][2];
int ipts0_num = 0, ipts1_num = 0;
float rpts0[MT9V03X_HH][2];
float rpts1[MT9V03X_HH][2];
int rpts0_num = 0, rpts1_num = 0;
float rpts0b[MT9V03X_HH][2];
float rpts1b[MT9V03X_HH][2];
int rpts0b_num = 0, rpts1b_num = 0;
float rpts0s[MT9V03X_HH][2];
float rpts1s[MT9V03X_HH][2];
int rpts0s_num = 0, rpts1s_num = 0;

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
image_t img_raw = {.data = NULL, .width = MT9V03X_W, .height = MT9V03X_H, .step = MT9V03X_W};

// ====================== 图像控制相关全局变量 ======================
float cx = 0, cy = 0;
float Guide = 0;
float Guide_up = 0;
float Guide_up_up = 0; // 第三级
float pure_angle_up = 0;
float pure_angle_up_up = 0;      // 第三级角度
float AIM_DISTANCE_UP = 0.8f;    // 中端预瞄距离
float AIM_DISTANCE_UP_UP = 1.1f; // 远端预瞄距离 (长直道预警)

int clip(int x, int low, int up) { return x > up ? up : (x < low ? low : x); }
float clipf(float x, float low, float up) { return x > up ? up : (x < low ? low : x); }
#undef AT
#undef AT_CLIP
#define AT(img, x, y) ((img)->data[(y) * (img)->step + (x)])
#define AT_CLIP(img, x, y) AT(img, clip(x, 0, (img)->width - 1), clip(y, 0, (img)->height - 1))
float Cal_rot_x(float x, float y) { return (rot[1][0] * y + rot[1][1] * x + rot[1][2]) / (rot[2][0] * y + rot[2][1] * x + rot[2][2]); }
float Cal_rot_y(float x, float y) { return (rot[0][0] * y + rot[0][1] * x + rot[0][2]) / (rot[2][0] * y + rot[2][1] * x + rot[2][2]); }
float Cal_inv_rot_x(float x, float y) { return (inv_rot[1][0] * y + inv_rot[1][1] * x + inv_rot[1][2]) / (inv_rot[2][0] * y + inv_rot[2][1] * x + 1.0f); }
float Cal_inv_rot_y(float x, float y) { return (inv_rot[0][0] * y + inv_rot[0][1] * x + inv_rot[0][2]) / (inv_rot[2][0] * y + inv_rot[2][1] * x + 1.0f); }

// 用于可视化 Debug 的中间变量
float debug_dx = 0;  // 横向边长
float debug_dy = 0;  // 纵向边长
float debug_dn = 0;  // 斜边长
int debug_aim_x = 0; // 近端预瞄点X坐标
int debug_aim_y = 0; // 近端预瞄点Y坐标

int calc_contrast(image_t *img, int x, int y, int direction)
{
    if (x < 2 || x > img->width - 3)
        return 0;
    int sum_left = 0, sum_right = 0;
    sum_left = AT_CLIP(img, x - 1, y);
    sum_right = AT_CLIP(img, x, y);
    int sum = sum_left + sum_right;
    if (sum == 0)
        return 0;
    int diff = 0;
    if (direction == 0)
    {
        if (sum_right > sum_left)
            diff = sum_right - sum_left;
        else
            return 0;
    }
    else
    {
        if (sum_left > sum_right)
            diff = sum_left - sum_right;
        else
            return 0;
    }
    return (diff * CBH_MULTIPLIER) / sum;
}

void rot_img_process(void)
{
    for (int i = 0; i < ipts0_num; i++)
    {
        float d = rot[2][0] * ipts0[i][1] + rot[2][1] * ipts0[i][0] + rot[2][2];
        rpts0[i][0] = (rot[1][0] * ipts0[i][1] + rot[1][1] * ipts0[i][0] + rot[1][2]) / d;
        rpts0[i][1] = (rot[0][0] * ipts0[i][1] + rot[0][1] * ipts0[i][0] + rot[0][2]) / d;
    }
    rpts0_num = ipts0_num;
    for (int i = 0; i < ipts1_num; i++)
    {
        float d = rot[2][0] * ipts1[i][1] + rot[2][1] * ipts1[i][0] + rot[2][2];
        rpts1[i][0] = (rot[1][0] * ipts1[i][1] + rot[1][1] * ipts1[i][0] + rot[1][2]) / d;
        rpts1[i][1] = (rot[0][0] * ipts1[i][1] + rot[0][1] * ipts1[i][0] + rot[0][2]) / d;
    }
    rpts1_num = ipts1_num;
}
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++)
    {
        pts_out[i][0] = 0;
        pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++)
        {
            float weight = half + 1 - fabs(j);
            int idx = clip(i + j, 0, num - 1);
            pts_out[i][0] += pts_in[idx][0] * weight;
            pts_out[i][1] += pts_in[idx][1] * weight;
        }
        float s = (half + 1) * (half + 1);
        pts_out[i][0] /= s;
        pts_out[i][1] /= s;
    }
}
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    if (num1 <= 0)
    {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    float dist_acc = 0.0f;
    float dist_next = dist;
    for (int i = 0; i < num1 - 1 && len < *num2; i++)
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];
        float seg_len = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        while (dist_acc + seg_len >= dist_next)
        {
            float t = (dist_next - dist_acc) / seg_len;
            pts_out[len][0] = x0 + (x1 - x0) * t;
            pts_out[len][1] = y0 + (y1 - y0) * t;
            len++;
            dist_next += dist;
            if (len >= *num2)
                break;
        }
        dist_acc += seg_len;
    }
    *num2 = len;
}
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < MT9V03X_HH; i++)
    {
        angle_out[i] = 0.0f;
    }

    for (int i = 0; i < num; i++)
    {
        if (num < 2 * dist + 1)
            continue;
        if (i < dist || i >= num - dist)
        {
            angle_out[i] = 0.0f;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[i - dist][0];
        float dy1 = pts_in[i][1] - pts_in[i - dist][1];
        float dx2 = pts_in[i + dist][0] - pts_in[i][0];
        float dy2 = pts_in[i + dist][1] - pts_in[i][1];
        float a1 = atan2f(dy1, dx1);
        float a2 = atan2f(dy2, dx2);
        angle_out[i] = a2 - a1;
        while (angle_out[i] > 3.14159f)
            angle_out[i] -= 6.28318f;
        while (angle_out[i] < -3.14159f)
            angle_out[i] += 6.28318f;
    }
}
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++)
    {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++)
        {
            if (i + j < 0 || i + j >= num)
                continue;
            if (fabsf(angle_in[i + j]) > fabsf(angle_out[i]))
            {
                angle_out[i] = 0.0f;
                break;
            }
        }
    }
}
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;
    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    while (step < *num && half < x && x <= img->width - half - 1 && y > half + 1 && y <= img->height - half - 1 && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += AT(img, x + dx, y + dy);

        local_thres = local_thres / (block_size * block_size) - clip_value;

        // 底线保护！防止在纯黑阴影中将噪点判定为赛道
        // 如果局部平均阈值低于 45（可微调），强制设定为 45
        if (local_thres < 45)
            local_thres = 45;

        int f = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int fl = AT(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);

        if (f < local_thres)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (fl < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else
        {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }

        // 防原地转圈检测 (Anti-Looping)
        // 每收集满 20 个点，检查一次物理位移
        if (step > 0 && step % 20 == 0)
        {
            int dx = pts[step - 1][0] - pts[step - 20][0];
            int dy = pts[step - 1][1] - pts[step - 20][1];
            // 如果 20 个点的直线物理位移不到 5 个像素，说明 100% 掉进了死循环
            if (dx * dx + dy * dy < 25)
            {
                break; // 果断打断循环,结束搜线
            }
        }
    }
    *num = step;
}
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;
    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    while (step < *num && half < x && x < img->width - half - 1 && y > half + 1 && y < img->height - half - 1 && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += AT(img, x + dx, y + dy);

        local_thres = local_thres / (block_size * block_size) - clip_value;

        // 右线底线保护
        if (local_thres < 45)
            local_thres = 45;

        int f = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int fr = AT(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);

        if (f < local_thres)
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (fr < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else
        {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }

        // 右线防原地转圈检测
        if (step > 0 && step % 20 == 0)
        {
            int dx = pts[step - 1][0] - pts[step - 20][0];
            int dy = pts[step - 1][1] - pts[step - 20][1];
            if (dx * dx + dy * dy < 25)
            {
                break;
            }
        }
    }
    *num = step;
}

void find_corners_strict(float *rpts0a, float *rpts0an, float *rpts1a, float *rpts1an)
{

    if (debug_target_found)
        return;
    // 1. 重置所有标志位
    Lpt0_found = false;
    Lpt1_found = false;
    Lpt0_rpts0s_id = 0;
    Lpt1_rpts1s_id = 0;

    // 直道判断逻辑
    int straight_thres_num = (int)(1.5f / sample_dist);
    is_straight0 = rpts0s_num > straight_thres_num;
    is_straight1 = rpts1s_num > straight_thres_num;

    int k_step = (int)roundf(angle_dist / sample_dist);

    // =================================================================
    // Part 1: 左线搜索 (Lpt0) - 带 DEBUG
    // =================================================================
    // 搜索范围放宽到 70，以捕获远端角点
    for (int i = 0; i < MIN(rpts0s_num, 70); i++) // 可调
    {
        if (rpts0an[i] == 0)
            continue;

        int im1 = clip(i - k_step, 0, rpts0s_num - 1);
        int ip1 = clip(i + k_step, 0, rpts0s_num - 1);

        // 计算角点置信度
        float conf1 = fabs(rpts0an[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2.0f;
        float conf1_deg = conf1 * 180.0f / PI; // 转换为角度用于显示

        // --- 2. L角点检查 (动态阈值) ---

        // [动态阈值计算]
        float base_deg = 70.0f;
        float decay = 0.55f;
        float thres_deg = base_deg - (decay * (float)i);
        if (thres_deg < 50.0f)
            thres_deg = 50.0f; // 最低不低于50度

        float dynamic_thres_rad = thres_deg / 180.0f * PI;
        float max_angle_rad = 135.0f / 180.0f * PI; // 放宽最大角度

        // 宽松要求：只要是往远处延伸即可 (Y轴检查)，去掉 X 轴限制
        // 既然角度已经高达 50~70 度，它不可能是直道，方向即使抖动也无所谓
        bool geometry_check = (rpts0s[im1][1] > rpts0s[ip1][1]);

        // [DEBUG] 只打印看起来像角点(>40度)的点，避免刷屏
        // if (conf1_deg > 40.0f)
        // {
        //     printf("[Lpt0] ID:%d | Ang:%.1f | Thres:%.1f | ", i, conf1_deg, thres_deg);

        //     if (Lpt0_found)
        //     {
        //         printf("SKIP (Already Found)\n");
        //     }
        //     else if (i >= 65)
        //     {
        //         printf("FAIL (Too Far > 65)\n");
        //     }
        //     else if (!geometry_check)
        //     {
        //         printf("FAIL (Geo Error)\n");
        //     }
        //     else if (conf1 > max_angle_rad)
        //     {
        //         printf("FAIL (Too Large > 135)\n");
        //     }
        //     else if (conf1 < dynamic_thres_rad)
        //     {
        //         printf("FAIL (Too Small < Thres)\n");
        //     }
        //     else
        //     {
        //         printf("SUCCESS -> FOUND!\n");
        //     }
        // }

        // [判定逻辑]
        if (!Lpt0_found &&
            conf1 > dynamic_thres_rad && // 大于动态阈值
            conf1 < max_angle_rad &&     // 小于最大角度
            i < 65 &&                    // 距离限制
            geometry_check)              // 几何方向正确
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;

            // 立即锁死真实的物理坐标
            true_Lpt0_x = rpts0s[i][0];
            true_Lpt0_y = rpts0s[i][1];
        }

        // 更新直道标志
        if (conf1 > (15.0f / 180.0f * PI) && i < straight_thres_num)
            is_straight0 = false;

        if (Lpt0_found)
            break;
    }

    // =================================================================
    // Part 2: 右线搜索 (Lpt1) - 对称逻辑
    // =================================================================
    for (int i = 0; i < MIN(rpts1s_num, 70); i++)
    {
        if (rpts1an[i] == 0)
            continue;
        int im1 = clip(i - k_step, 0, rpts1s_num - 1);
        int ip1 = clip(i + k_step, 0, rpts1s_num - 1);

        float conf2 = fabs(rpts1an[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2.0f;
        // float conf2_deg = conf2 * 180.0f / PI;

        // 动态阈值 (右线同理)
        float base_deg = 70.0f;
        float decay = 0.55f;
        float thres_deg = base_deg - (decay * (float)i);
        if (thres_deg < 50.0f)
            thres_deg = 50.0f;

        float dynamic_thres_rad = thres_deg / 180.0f * PI;
        float max_angle_rad = 135.0f / 180.0f * PI;

        if (!Lpt1_found &&
            conf2 > dynamic_thres_rad &&
            conf2 < max_angle_rad &&
            i < 65)
        {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
            // printf("[Lpt1] FOUND ID:%d Ang:%.1f\n", i, conf2_deg); // 右线简单打印
            // 立即锁死真实的物理坐标
            true_Lpt1_x = rpts1s[i][0];
            true_Lpt1_y = rpts1s[i][1];
        }

        if (conf2 > (15.0f / 180.0f * PI) && i < straight_thres_num)
            is_straight1 = false;

        if (Lpt1_found)
            break;
    }
}

// ====================== 中线生成函数 (法线精确局部鼓包 - 防打结版) ======================
void generate_center_line(void)
{
    float temp_line[MT9V03X_HH][2];
    int temp_num = 0;

    // 基础平移距离
    float default_shift = 10;
    int current_track_type = track_type;
    int step = 5;

    // --- 1. 先生成基础的正常中线 temp_line ---
    if (current_track_type == 1)
    {
        if (rpts0s_num > step * 2)
        {
            temp_num = rpts0s_num;
            for (int i = 0; i < rpts0s_num; i++)
            {
                float dx, dy;
                int idx_prev = (i - step < 0) ? 0 : (i - step);
                int idx_next = (i + step >= rpts0s_num) ? (rpts0s_num - 1) : (i + step);
                dx = rpts0s[idx_next][0] - rpts0s[idx_prev][0];
                dy = rpts0s[idx_next][1] - rpts0s[idx_prev][1];
                float nx = -dy;
                float ny = dx;
                float len = sqrtf(nx * nx + ny * ny);
                if (len > 1e-3)
                {
                    nx /= len;
                    ny /= len;
                }
                else
                {
                    nx = 1.0f;
                    ny = 0.0f;
                }
                temp_line[i][0] = rpts0s[i][0] + nx * default_shift;
                temp_line[i][1] = rpts0s[i][1] + ny * default_shift;
            }
        }
        else
        {
            temp_num = 0;
        }
    }
    else if (current_track_type == 2)
    {
        if (rpts1s_num > step * 2)
        {
            temp_num = rpts1s_num;
            for (int i = 0; i < rpts1s_num; i++)
            {
                float dx, dy;
                int idx_prev = (i - step < 0) ? 0 : (i - step);
                int idx_next = (i + step >= rpts1s_num) ? (rpts1s_num - 1) : (i + step);
                dx = rpts1s[idx_next][0] - rpts1s[idx_prev][0];
                dy = rpts1s[idx_next][1] - rpts1s[idx_prev][1];
                float nx = dy;
                float ny = -dx;
                float len = sqrtf(nx * nx + ny * ny);
                if (len > 1e-3)
                {
                    nx /= len;
                    ny /= len;
                }
                else
                {
                    nx = -1.0f;
                    ny = 0.0f;
                }
                temp_line[i][0] = rpts1s[i][0] + nx * default_shift;
                temp_line[i][1] = rpts1s[i][1] + ny * default_shift;
            }
        }
        else
        {
            temp_num = 0;
        }
    }
    else
    {
        // 混合模式 (正常双边巡线)
        if (rpts0s_num > 10 && rpts1s_num > 10)
        {
            temp_num = MIN(rpts0s_num, rpts1s_num);
            for (int i = 0; i < temp_num; i++)
            {
                temp_line[i][0] = (rpts0s[i][0] + rpts1s[i][0]) / 2.0f;
                temp_line[i][1] = (rpts0s[i][1] + rpts1s[i][1]) / 2.0f;
            }
        }
        else if (rpts0s_num > 10)
        {
            if (rpts0s_num > step * 2)
            {
                temp_num = rpts0s_num;
                for (int i = 0; i < rpts0s_num; i++)
                {
                    int p = (i - step < 0) ? 0 : (i - step);
                    int n = (i + step >= rpts0s_num) ? (rpts0s_num - 1) : (i + step);
                    float nx = -(rpts0s[n][1] - rpts0s[p][1]);
                    float ny = (rpts0s[n][0] - rpts0s[p][0]);
                    float len = sqrtf(nx * nx + ny * ny);
                    if (len > 1e-3)
                    {
                        nx /= len;
                        ny /= len;
                    }
                    else
                    {
                        nx = 1.0f;
                        ny = 0.0f;
                    }
                    temp_line[i][0] = rpts0s[i][0] + nx * default_shift;
                    temp_line[i][1] = rpts0s[i][1] + ny * default_shift;
                }
            }
            else
            {
                temp_num = rpts0s_num;
                for (int i = 0; i < rpts0s_num; i++)
                {
                    temp_line[i][0] = rpts0s[i][0] + default_shift;
                    temp_line[i][1] = rpts0s[i][1];
                }
            }
        }
        else if (rpts1s_num > 10)
        {
            if (rpts1s_num > step * 2)
            {
                temp_num = rpts1s_num;
                for (int i = 0; i < rpts1s_num; i++)
                {
                    int p = (i - step < 0) ? 0 : (i - step);
                    int n = (i + step >= rpts1s_num) ? (rpts1s_num - 1) : (i + step);
                    float nx = (rpts1s[n][1] - rpts1s[p][1]);
                    float ny = -(rpts1s[n][0] - rpts1s[p][0]);
                    float len = sqrtf(nx * nx + ny * ny);
                    if (len > 1e-3)
                    {
                        nx /= len;
                        ny /= len;
                    }
                    else
                    {
                        nx = -1.0f;
                        ny = 0.0f;
                    }
                    temp_line[i][0] = rpts1s[i][0] + nx * default_shift;
                    temp_line[i][1] = rpts1s[i][1] + ny * default_shift;
                }
            }
            else
            {
                temp_num = rpts1s_num;
                for (int i = 0; i < rpts1s_num; i++)
                {
                    temp_line[i][0] = rpts1s[i][0] - default_shift;
                    temp_line[i][1] = rpts1s[i][1];
                }
            }
        }
        else
        {
            temp_num = 0;
        }
    }
    // =========================================================
    // 简单粗暴：目标避障时，中线整体平移
    // =========================================================
    // 只要有避障指令 (无论目标在不在视野内，靠 target_override_track_type 记忆维持)
    if (temp_num > 0 && target_override_track_type != -1)
    {

        float dir = (target_override_track_type == 1) ? -1.0f : 1.0f; // 1 左绕(-), 2 右绕(+)
        float shift_amount = target_shift_offset;
        int step_n = 4; // 计算切线法向量的步长

        // 1. 拷贝一份干净的原始中线用于算法线，防止覆盖后自扭曲
        float orig_line[MT9V03X_HH][2];
        for (int i = 0; i < temp_num; i++)
        {
            orig_line[i][0] = temp_line[i][0];
            orig_line[i][1] = temp_line[i][1];
        }

        // 2. 遍历整条中线，不再判断 Y 轴区域，全部加上平移量
        for (int i = 0; i < temp_num; i++)
        {
            int idx_prev = (i - step_n < 0) ? 0 : (i - step_n);
            int idx_next = (i + step_n >= temp_num) ? (temp_num - 1) : (i + step_n);

            float dx = orig_line[idx_next][0] - orig_line[idx_prev][0];
            float dy = orig_line[idx_next][1] - orig_line[idx_prev][1];

            // 计算法线向量 (nx, ny) 以实现垂直于赛道方向的平移
            float nx = -dy;
            float ny = dx;
            float len = sqrtf(nx * nx + ny * ny);
            if (len > 1e-3)
            {
                nx /= len;
                ny /= len;
            }
            else
            {
                nx = 1.0f;
                ny = 0.0f;
            }

            // 将偏移量叠加回原数组
            temp_line[i][0] = orig_line[i][0] + nx * dir * shift_amount;
            temp_line[i][1] = orig_line[i][1] + ny * dir * shift_amount;
        }
    }

    // 最后平滑和重采样
    if (temp_num > 5)
    {
        float smooth_line[MT9V03X_HH][2];
        blur_points(temp_line, temp_num, smooth_line, 7); // 这里能进一步将刚才的梯形圆滑化

        rptsn_num = MT9V03X_HH;
        resample_points(smooth_line, temp_num, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
    }
    else
    {
        rptsn_num = MT9V03X_HH;
        resample_points(temp_line, temp_num, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
    }
}

void calculate_control_error(void)
{
    if (rptsn_num < 3)
    {
        pure_angle = 0;
        pure_angle_up = 0;
        pure_angle_up_up = 0;
        Guide = 0;
        Guide_up = 0;
        Guide_up_up = 0;
        debug_dn = 0; // 重置显示状态
        return;
    }

    // 1. 动态物理原点映射
    float H_zoom = 0.98f;
    float Half_width = MT9V03X_W / 2.0f;
    cx = Cal_rot_x(Half_width, MT9V03X_H * H_zoom);
    cy = Cal_rot_y(Half_width, MT9V03X_H * H_zoom);

    // 2. 选取三级预瞄点
    int aim_idx = (int)clip(roundf(aim_distance / sample_dist), 0, rptsn_num - 1);
    int aim_idx_up = (int)clip(roundf(AIM_DISTANCE_UP / sample_dist), 0, rptsn_num - 1);
    int aim_idx_up_up = (int)clip(roundf(AIM_DISTANCE_UP_UP / sample_dist), 0, rptsn_num - 1);

    //  保存近端预瞄点坐标用于绘图
    debug_aim_x = (int)rptsn[aim_idx][0];
    debug_aim_y = (int)rptsn[aim_idx][1];

    // 3. 计算近端转向偏差 (Guide)
    float dx = rptsn[aim_idx][0] - cx;
    float dy = cy - rptsn[aim_idx][1];
    if (dy < 0.01f)
        dy = 0.01f;
    float dn2 = dx * dx + dy * dy;

    debug_dx = dx;
    debug_dy = dy;
    debug_dn = sqrtf(dn2); // 计算出真实的斜边长

    if (dn2 > 0.001f)
        pure_angle = atanf((2.0f * dy * dx) / dn2) * 180.0f / PI;
    else
        pure_angle = 0;

    Guide = dx * fabs(pure_angle);
    Guide = clipf(Guide, -150.0f, 150.0f);

    // 4. 计算中端速度决策偏差 (Guide_up)
    float dx_up = rptsn[aim_idx_up][0] - cx;
    float dy_up = cy - rptsn[aim_idx_up][1];
    if (dy_up < 0.01f)
        dy_up = 0.01f;
    float dn2_up = dx_up * dx_up + dy_up * dy_up;
    if (dn2_up > 0.001f)
        pure_angle_up = atanf((2.0f * dy_up * dx_up) / dn2_up) * 180.0f / PI;
    else
        pure_angle_up = 0;

    Guide_up = dx_up * fabs(pure_angle_up);
    Guide_up = clipf(Guide_up, -150.0f, 150.0f);

    // 5. 计算远端速度决策偏差 (Guide_up_up)
    float dx_up_up = rptsn[aim_idx_up_up][0] - cx;
    float dy_up_up = cy - rptsn[aim_idx_up_up][1];
    if (dy_up_up < 0.01f)
        dy_up_up = 0.01f;
    float dn2_up_up = dx_up_up * dx_up_up + dy_up_up * dy_up_up;
    if (dn2_up_up > 0.001f)
        pure_angle_up_up = atanf((2.0f * dy_up_up * dx_up_up) / dn2_up_up) * 180.0f / PI;
    else
        pure_angle_up_up = 0;

    Guide_up_up = dx_up_up * fabs(pure_angle_up_up);
    Guide_up_up = clipf(Guide_up_up, -150.0f, 150.0f);
}

void image_handle(const cv::Mat &frame)
{
    img_raw.data = frame.ptr<uint8_t>(0);
    // 2. 初始化扫描起始点
    int start_x = img_raw.width / 2;
    int center_y = BEGIN_Y - 20;

    int scan_x1 = start_x;
    int scan_y1 = BEGIN_Y - 20;
    int scan_x2 = start_x;
    int scan_y2 = BEGIN_Y - 20;

    // --- 【边界放宽参数定义】 ---

    int half_block = block_size / 2;
    int min_scan_x = half_block + 2;
    int max_scan_x = img_raw.width - half_block - 2;

    // 3. 确定起扫中心点 (根据是否检测到目标物，或寻找最亮区域)
    if (debug_target_found && (target_red_y <= BEGIN_Y + 10) && (target_red_y + target_red_h >= BEGIN_Y - 10))
    {
        scan_x1 = target_red_x - 5;
        if (scan_x1 < min_scan_x)
            scan_x1 = min_scan_x; // 同步使用新的最小边界

        scan_x2 = target_red_x + target_red_w + 5;
        if (scan_x2 > max_scan_x)
            scan_x2 = max_scan_x; // 同步使用新的最大边界

        start_x = target_red_x + target_red_w / 2;
    }
    else
    {
        int max_brightness = 0;
        for (int tx = img_raw.width / 2 - 40; tx < img_raw.width / 2 + 40; tx += 5)
        {
            int bri = AT_CLIP(&img_raw, tx, center_y); // AT_CLIP获取灰度值
            if (bri > max_brightness)
            {
                max_brightness = bri;
                start_x = tx;
            }
        }
        if (max_brightness < 50)
            start_x = img_raw.width / 2;
        scan_x1 = start_x;
        scan_x2 = start_x;
    }

    // =========================================================
    // ------------------- 左边线搜索 (L型扫描 + 防噪点) -------------------
    // =========================================================
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    int left_found = 0;

    // A. 底边水平搜索 (往左扫)
    for (; scan_x1 > min_scan_x; scan_x1--)
    {
        int contrast = calc_contrast(&img_raw, scan_x1, scan_y1, 0);
        if (contrast > CBH_THRESHOLD)
        {
            int temp_num = sizeof(ipts0) / sizeof(ipts0[0]);
            // 起点往赛道内侧多缩 2 个像素，避开模糊边缘
            findline_lefthand_adaptive(&img_raw, block_size, clip_value, scan_x1, scan_y1, ipts0, &temp_num);
            // 传进去的图像 正方形的边长 经验值    爬线起始点 爬线起始点 把爬线点存到 ipts0 数组里 爬线点数量存到 temp_num 里

            // 只有当真正爬出至少 5 个点，才承认这是赛道边线，否则视为噪点继续扫描
            if (temp_num >= 5)
            {
                ipts0_num = temp_num;
                left_found = 1;
                break;
            }
        }
    }

    // B. 左侧垂直边框搜索 (L型补偿)

    if (!left_found && cross_type == CROSS_NONE)
    {
        for (int scan_y_edge = scan_y1 - 2; scan_y_edge > BEGIN_Y - 10; scan_y_edge -= 2)
        {
            int contrast = calc_contrast(&img_raw, min_scan_x, scan_y_edge, 0);
            if (contrast > CBH_THRESHOLD)
            {
                int temp_num = sizeof(ipts0) / sizeof(ipts0[0]);
                findline_lefthand_adaptive(&img_raw, block_size, clip_value, min_scan_x + 2, scan_y_edge, ipts0, &temp_num);

                if (temp_num >= 5)
                { // 防噪点过滤
                    ipts0_num = temp_num;
                    left_found = 1;
                    break;
                }
            }
        }
    }
    if (!left_found)
        ipts0_num = 0;

    // =========================================================
    // ------------------- 右边线搜索 (L型扫描 + 防噪点) -------------------
    // =========================================================
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    int right_found = 0;

    // 追踪变量，如果你不需要上位机显示 Debug 文本，可以注释掉这些 dbg_ 变量
    int dbg_scan_mode = 0;
    int dbg_start_x = 0;
    int dbg_start_y = 0;
    int dbg_contrast = 0;

    // A. 底边水平搜索 (往右扫)
    for (; scan_x2 < max_scan_x; scan_x2++)
    {
        int contrast = calc_contrast(&img_raw, scan_x2, scan_y2, 1);
        if (contrast > CBH_THRESHOLD)
        {
            int temp_num = sizeof(ipts1) / sizeof(ipts1[0]);
            // 起点往赛道内侧多缩 2 个像素 (scan_x2 - 2)，避开反光噪点
            findline_righthand_adaptive(&img_raw, block_size, clip_value, scan_x2 - 2, scan_y2, ipts1, &temp_num);

            if (temp_num >= 5)
            {
                ipts1_num = temp_num;
                right_found = 1;
                dbg_scan_mode = 1;
                dbg_start_x = scan_x2;
                dbg_start_y = scan_y2;
                dbg_contrast = contrast;
                break;
            }
        }
    }

    if (!right_found && cross_type == CROSS_NONE)
    {
        for (int scan_y_edge = scan_y2 - 2; scan_y_edge > BEGIN_Y - 30; scan_y_edge -= 2)
        {
            int contrast = calc_contrast(&img_raw, max_scan_x, scan_y_edge, 1);
            if (contrast > CBH_THRESHOLD)
            {
                int temp_num = sizeof(ipts1) / sizeof(ipts1[0]);
                findline_righthand_adaptive(&img_raw, block_size, clip_value, max_scan_x - 2, scan_y_edge, ipts1, &temp_num);

                if (temp_num >= 5)
                {
                    ipts1_num = temp_num;
                    right_found = 1;
                    dbg_scan_mode = 2;
                    dbg_start_x = max_scan_x;
                    dbg_start_y = scan_y_edge;
                    dbg_contrast = contrast;
                    break;
                }
            }
        }
    }
    if (!right_found)
        ipts1_num = 0;

    // ====== 以下原代码不变 ======
    rot_img_process();
    blur_points(rpts0, rpts0_num, rpts0b, line_blur_kernel);
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, line_blur_kernel);
    rpts1b_num = rpts1_num;

    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    float sample_pixel_dist = sample_dist * pixel_per_meter;
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_pixel_dist);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_pixel_dist);

    float rpts0a[MT9V03X_HH], rpts1a[MT9V03X_HH];
    float rpts0an[MT9V03X_HH], rpts1an[MT9V03X_HH];
    int angle_dist_points = (int)roundf(angle_dist / sample_dist);
    local_angle_points(rpts0s, rpts0s_num, rpts0a, angle_dist_points);
    local_angle_points(rpts1s, rpts1s_num, rpts1a, angle_dist_points);
    nms_angle(rpts0a, rpts0s_num, rpts0an, nms_kernel);
    nms_angle(rpts1a, rpts1s_num, rpts1an, nms_kernel);

    find_corners_strict(rpts0a, rpts0an, rpts1a, rpts1an);
    check_circle();
    check_cross();
    run_circle();
    run_cross();
    generate_center_line();
    calculate_control_error();
}

void image_display_opencv(cv::Mat &img_gray)
{
    cv::Mat img_show = img_gray.clone();
    // cv::Mat img_nitoushi = img_gray.clone();
    img_nitoushi = img_gray.clone(); // 显示到全局变量里面。

    for (int i = 0; i < ipts0_num; i++)
    {
        img_show.at<uchar>(ipts0[i][1], ipts0[i][0]) = 255;
    }

    for (int i = 0; i < ipts1_num; i++)
    {
        img_show.at<uchar>(ipts1[i][1], ipts1[i][0]) = 255;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////
    // ================== 逆透视左线保护 ==================
    for (int i = 0; i < rpts0s_num; i++)
    {
        int px = (int)rpts0s[i][0];
        int py = (int)rpts0s[i][1];
        // 严格限制在 160x120 图像范围内，防止 .at() 越界崩溃
        if (px >= 0 && px < 160 && py >= 0 && py < 120)
        {
            img_nitoushi.at<uchar>(py, px) = 255;
        }
    }

    // ================== 逆透视右线保护 ==================
    for (int i = 0; i < rpts1s_num; i++)
    {
        int px = (int)rpts1s[i][0];
        int py = (int)rpts1s[i][1];
        if (px >= 0 && px < 160 && py >= 0 && py < 120)
        {
            img_nitoushi.at<uchar>(py, px) = 255;
        }
    }

    // ================== 逆透视中线保护 ==================
    for (int i = 0; i < rptsn_num; i++)
    {
        int px = (int)rptsn[i][0];
        int py = (int)rptsn[i][1];
        if (px >= 0 && px < 160 && py >= 0 && py < 120)
        {
            img_nitoushi.at<uchar>(py, px) = 255;
        }
    }

    // ================== 左角点标记保护 ==================
    if (Lpt0_found)
    {
        int px = (int)true_Lpt0_x;
        int py = (int)true_Lpt0_y;
        // 只在角点处于画面可见范围内时才进行绘制
        if (px >= 0 && px < 160 && py >= 0 && py < 120)
        {
            cv::drawMarker(img_nitoushi, cv::Point(px, py), cv::Scalar(255), cv::MARKER_CROSS, 4, 1);
            // 提示：你原来的 size 设为 1 可能太小了看不清，这里我帮你改成了 10，你可以按需调回来
        }
    }

    // ================== 右角点标记保护 ==================
    if (Lpt1_found)
    {
        int px = (int)true_Lpt1_x;
        int py = (int)true_Lpt1_y;
        if (px >= 0 && px < 160 && py >= 0 && py < 120)
        {
            cv::drawMarker(img_nitoushi, cv::Point(px, py), cv::Scalar(255), cv::MARKER_CROSS, 4, 1);
        }
    }

    // 预瞄点绘制
    cv::drawMarker(img_nitoushi, cv::Point(debug_aim_x, debug_aim_y), cv::Scalar(255), cv::MARKER_TILTED_CROSS, 4, 1);

    // cv::imshow("yuantu", img_show);
    // cv::imshow("nitoushi", img_nitoushi);
    // 在Linux中使用不了imshow.
}
