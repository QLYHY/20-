#ifndef _IMAGE_H
#define _IMAGE_H

#include "zf_common_headfile.h"

#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白

#define white_pixel 255
#define black_pixel 0

#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色

#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值

#define image_h 90//八邻域图像高度
#define image_w 180//八邻域图像宽度

extern volatile int Island_State,Left_Island_Flag,Right_Island_Flag,Search_Stop_Line,Left_Line[MT9V03X_H],Right_Line[MT9V03X_H];
extern volatile uint8 Img_Disappear_Flag; //图像消失标志位，1就说明丢图了
extern uint8 image_two_value[MT9V03X_H][MT9V03X_W],original_image[image_h][image_w],bin_image[image_h][image_w],image_copy[image_h][image_w];
extern volatile int Left_Lost_Time,Right_Lost_Time;
extern int buxian_flag;
extern volatile int Zebra_State,Boundry_Start_Left,Left_Up_Guai[2];
extern volatile int unilateral_bridge_state,jump_state,Cross_Flag,barrier_state;

extern uint8 l_border[image_h];//左线数组
extern uint8 r_border[image_h];//右线数组
extern uint8 center_line[image_h];//中线数组

extern int piont_x[10],piont_y[10],flag_count;
extern int left_barrier,right_barrier;
extern int streeing_switch;
extern int jump_begin_distance;
extern int zaber_start_flag,order_flag;
extern int8 Element_Order[];
extern int island_road_state;
int my_abs(int value);
void Show_Boundry(void);
void image_disposal(void);
void image_process(void);
void histogram_equalization(uint8_t* image, int width, int height);
void optimizeBinaryImage(uint8(*bin_image)[image_w]);
void showimage(void);
void find_first_lose(void);
void showimage(void);
int left_point_big(int start,int end);
extern int point_of_contact[2],symmetry_point[2];
#endif /*_IMAGE_H*/

