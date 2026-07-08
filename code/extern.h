/*
 * extern.h
 *
 *  Created on: 2025年4月25日
 *      Author: zhangjie
 */

#ifndef CODE_EXTERN_H_
#define CODE_EXTERN_H_
#include "zf_common_headfile.h"

// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

#define KEY1                    (P20_6)
#define KEY2                    (P20_7)
#define KEY3                    (P11_2)
#define KEY4                    (P11_3)

// 只有X边界
extern uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
extern uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];
// X Y边界都是单独指定的
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
extern uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];
extern uint8 line_state;

//extern uint8 left_draw,right_draw,middle_draw;
extern uint8 state;
extern uint32 image_time;
extern uint16 first_state_1;
extern int ips[20];
extern int first_lose[2],first_lastylose[2];
extern int aclture_distance,detour_integer,detour_state,detour_state_for,deter_time;
extern int right_barrier_up,right_barrier_down,left_barrier_up,left_barrier_down,barrier_point;
extern int right_bridge_up,right_bridge_down,left_bridge_up,left_bridge_down;
extern int right_jump_up,right_jump_down,left_jump_up,left_jump_down;
extern int del;
extern int last_distance;
extern int wrong_distance_state;
extern int wish;
extern int protect_flag;
extern int bridge_high,bridge_cancle,bridge_delay,bridge_delay_2,bridge_delay_cancle,bridge_err,barreir_delay,bridge_last_delay;
extern int pwm_servo;
extern float high_leg;
extern int podao_state,podao_reday,podao_delay;
void open_rgb565_image(void);
void Key_init(void);
void key_scan(void);
void effect(void);
void wrong_distance(int star_data, int *data, int *data_last, float data_p);
#endif /* CODE_EXTERN_H_ */
