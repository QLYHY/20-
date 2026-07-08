/*
 * pid.h
 *
 *  Created on: 2025年3月10日
 *      Author: lenovo
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern float angle_KP,angle_KI,angle_KD;
extern float angle_speed_KP,angle_speed_KI,angle_speed_KD;
extern float Balance_Velocity_KP,Balance_Velocity_KI,Balance_Velocity_KD;
extern float steering_KP_small,steering_KP_large,steering_KD_small,steering_KD_large,steering_GKD;
extern float steering_KP,steering_KP2,steering_KD;
extern float Velocity_KP,Velocity_KI;
extern float roll_angle_KP,roll_angle_KI,roll_angle_KD;
extern float Steering_compensation,Steering_compensation_L,Steering_compensation_R;
extern float check,check1,check2,check3,speed_error,check4;
extern float bend_KP,bend_KI,bend_KD;
extern float Balance_Velocity_KP_TEMP,Balance_Velocity_KI_TEMP,Balance_Velocity_KD_TEMP;

extern float bend_gain;

extern float Err_streeing;

int Distance_Measure(void);

float angle_pid(float Angle,float Angle_Zero);              //角度环
float angle_speed_pid(float PWM_expectation,float Gyro);    //角速度环
float Balance_Velocity_pid(int expect_speed,int encoder);   //速度环
float steering_pid(float ExpectAngle);                      //转向环
void streeing_optimize(void);                               //转向优化（调用这个）
float roll_angle_pid(float to_roll, float Angle);                         //横滚环
float roll_angle_speed_pid(float expect_gyro,float gyro);
float bend_pid(float ERR);                                  //压弯环
float yaw_angle_pid(float to_yaw,float act_yaw);
#endif /* CODE_PID_H_ */
