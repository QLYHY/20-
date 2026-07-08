/*
 * engine.h
 *
 *  Created on: 2025年1月7日
 *      Author: lenovo
 */

#ifndef CODE_ENGINE_H_
#define CODE_ENGINE_H_

#include "zf_common_headfile.h"

#define engine_R2               ATOM0_CH2_P21_4
#define engine_L2               ATOM0_CH3_P21_5
#define engine_L1               ATOM0_CH0_P21_2
#define engine_R1               ATOM0_CH1_P21_3

#define leg1                    60
#define leg2                    90
#define leg3                    90
#define leg4                    60
#define L5                      35

#define wide                       90

typedef enum    // 枚举舵机ID
{
    L1,L2,R1,R2,ALL,

}engine_ID_enum;

typedef struct
{
    float XLeft,YLeft;
    float XRight,YRight;
    float Left_alpha,Left_beta;
    float Right_alpha,Right_beta;

}engine;

extern engine engine_data;

extern float E_H,Kp_H,Kp_X;

extern float X,Y,Y_demand,Y_L,Y_R;
extern int leg_state_left,leg_state_right,unilateral_bridge_num,leg_state,leg_same;
extern float yaw_err,yaw_ture;

extern int jump_count,jump_timer_begine,jump_timer;
extern float engine_limit_max,engine_limit_min;
//====================================================舵机 基础函数====================================================
void engine_init(void);
void engine_control(engine_ID_enum ID,float angle);
void engine_jump(void);
float PWM_limit(float pwm);
void engine_console(void);
void engine_output_fllter(void);

void engine_slow(int high);     //单边桥缓慢抬腿
void engine_define(void);       //计算通过单边桥数量
void yaw_amend(int high,int low); //单边桥闭环航向角补偿误差
//====================================================舵机 基础函数====================================================

#endif /* CODE_ENGINE_H_ */
