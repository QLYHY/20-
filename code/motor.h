/*
 * motor.h
 *
 *  Created on: 2025Äê1ÔÂ23ÈÕ
 *      Author: lenovo
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#define V_D_Increase            2.67f

#include "zf_common_headfile.h"

typedef struct
{
        int16 *gyro_raw_data;
        int16 *acc_raw_data;
        float gyro_ration;
        float acc_ration;
        float call_cycle;
        int16 mechanical_zero;

        int16 filtering_angle;
        int16 angle_temp;

}imu_posture;

extern imu_posture imp_posture_value;
extern int velocity;
extern int Speed_Integral,Speed_Integral_flag,Speed_Integral_count;
extern int angle_Integral_start_flag,angle_Integral_finish_flag;
extern float PWM_accel_l,PWM_accel_r,PWM_accel;
extern int speed;
extern int E_H_flag;

extern float roll_pwm;

extern int unilateral_bridge_timer;

extern int Normal_speed,Special_speed;

extern int leg_high;

void suduhuan(void);
void jiaoduhuan(void);
void jiaosuduhuan(void);
float Err_Sum(void);
int angle_Integral(float aim);
void unilateral_bridge(void);
void roll_balance(void);

#endif /* CODE_MOTOR_H_ */
