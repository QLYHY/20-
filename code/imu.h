/*
 * imu.h
 *
 *  Created on: 2025年7月11日
 *      Author: zhangjie
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

extern float yaw,bridge_yaw,yaw_last,yaw_yuan;
extern int bridge_yaw_state;;
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;


typedef struct {
    float pitch;
    float roll;
    float pitch_last;
    float roll_last;
    float yaw;
    float yaw_last;
} euler_param_t;


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

extern icm_param_t icm_data;
extern euler_param_t eulerAngle;
extern float now_angle,target_angle;
extern float fx,fy,fz;
extern float gyro_x,gyro_y,gyro_z;
void gyroOffset_init(void);

float fast_sqrt(float x);

void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void ICM_getValues();

void ICM_getEulerianAngles(void);

//int angle_Integral(float aim);          //int 鏄繑鍥炲€肩殑绫诲瀷锛寁oid璇存槑璇ュ嚱鏁颁笉闇€瑕佽繑鍥炲€�

void imu_data_get(void);


#endif /* CODE_IMU_H_ */

