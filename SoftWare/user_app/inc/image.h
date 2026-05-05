#ifndef IMAGE_H
#define IMAGE_H

#include "headfile.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern float pure_angle;
extern int have_lost_Lpt0_found;
extern int have_lost_Lpt1_found;
extern int ipts0_num, ipts1_num;
extern int ipts0[MT9V03X_HH][2];
extern int ipts1[MT9V03X_HH][2];
extern char cross_state_info[64];

void calibrate_cbh_thresholds(void); // 声明整定函数
void image_handle(const cv::Mat &frame);
void image_display_opencv(cv::Mat &img_gray);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
float Cal_rot_x(float x, float y);
float Cal_rot_y(float x, float y);
float Cal_inv_rot_x(float x, float y);
float Cal_inv_rot_y(float x, float y);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num);
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num);

#define AT(img, x, y) ((img)->data[(y) * (img)->step + (x)])
#define AT_CLIP(img, x, y) AT(img, clip(x, 0, (img)->width - 1), clip(y, 0, (img)->height - 1))
int clip(int x, int low, int up); 
#endif // IMAGE_H