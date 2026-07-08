/*
 * motor.c
 *
 *  Created on: 2025年1月23日
 *      Author: lenovo
 */

#include "motor.h"
#include "math.h"


int Normal_speed=650;//450
int Special_speed=225;//225
int speed=0;
int SPEED=0;
float PWM_X=0,PWM_accel=0,PWM_accel_l=0,PWM_accel_r=0,PWM_End=0;
float PWM_Left=0,PWM_Right=0;
int Speed_Integral=0,Speed_Integral_flag=0,Speed_Integral_count=0;
int angle_Integral_start_flag=0,angle_Integral_finish_flag=0;

int E_H_flag=0;

int velocity;

int unilateral_bridge_timer=0;

const uint8 Weight[MT9V03X_H]=      //图像权重
{

        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 5, 9, 11,15,19,20,20,
        19,17,15,13,11,11, 9, 5, 3, 3,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,


};

/*-------------------------------------------------------------------------------------------------------------------
  @brief     摄像头误差获取
  @param     null
  @return    获取到的误差
  Sample     err=Err_Sum();
  @note      加权取平均
-------------------------------------------------------------------------------------------------------------------*/
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差

    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//常规误差计算
    {

        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//右移1位，等效除2
        weight_count+=Weight[i];
    }
    err=err/weight_count;

    return err;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环计算
// 参数说明     无
// 返回参数     void
// 使用示例     suduhuan();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void suduhuan(void)
{

    PWM_accel= -Balance_Velocity_pid(speed,(int)velocity);

    if(unilateral_bridge_state==0&&podao_state==0&&jump_state==0)
    {
        speed = Normal_speed - 0*fabs(err);
    }

    if(island_road_state==1||Zebra_State==1||(zaber_start_flag==0&&begine_flag==1))
    {
        aclture_distance+=(velocity/20);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度环计算
// 参数说明     无
// 返回参数     void
// 使用示例     jiaoduhuan();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void jiaoduhuan(void)
{
    PWM_X=angle_pid(eulerAngle.roll,-1.7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角速度环计算
// 参数说明     无
// 返回参数     void
// 使用示例     jiaosuduhuan();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void jiaosuduhuan(void)
{
    PWM_End=angle_speed_pid(PWM_X,-imu660ra_gyro_x);

    PWM_Left = PWM_End+Steering_compensation;
    PWM_Right = PWM_End-Steering_compensation;
    ips[16]=PWM_End;
    if(!protect_flag)
        small_driver_set_duty((PWM_Right),(-PWM_Left));//small_driver_set_duty((int)(-PWM_Left),(int)(PWM_Right));
//    else if(protect_flag)small_driver_set_duty(0,0);  //保护
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度积分计算
// 参数说明     无
// 返回参数     积分标志位   积分未完成：0 积分完成：1
// 使用示例     angle_Integral();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int angle_Integral(float aim)
{
    int angle_Integral_finish_flag=0;

    yaw -= gyro_z *57.325*0.018f*6*5.6*0.0333;  //*57.325*0.0018f

    if(fabs(yaw-yaw_last)<0.001)
    {
        yaw=yaw_last;
    }
    yaw_last=yaw;

    if(fabs(yaw)>=aim)
    {
        angle_Integral_finish_flag=1;
        yaw=0;
        yaw_last=0;
    }

    return angle_Integral_finish_flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单边桥控制函数
// 参数说明     无
// 返回参数     无
// 使用示例     unilateral_bridge();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int leg_high=80;
void unilateral_bridge(void)
{
      if(unilateral_bridge_state==1&&unilateral_bridge_timer<200)          //5cm
      {
          Kp_X=1;

          speed=150;
//          engine_slow(75);
    }
      else if(unilateral_bridge_state==1&&unilateral_bridge_timer>=200&&unilateral_bridge_timer<300)          //5cm
      {
          speed=100;
          engine_slow(leg_high);

     }
      else if(unilateral_bridge_state==1&&unilateral_bridge_timer>=300&&unilateral_bridge_timer<600)          //5cm
        {
            speed=50;
            engine_slow(leg_high);
       }
      else if(unilateral_bridge_state==1&&unilateral_bridge_timer>=600)
      {
          Kp_X=0.6;
        speed=Normal_speed-170;
        engine_slow(leg_high);

//        steering_KP_small=19;
//        steering_KP_large=41;

        E_H_flag=1;

        if((unilateral_bridge_num==3&&bridge_delay_cancle>=500)||(bridge_err>=1000)
                ||bridge_last_delay>=2500)
        {
            if(order_flag<3)
            {
                order_flag++;
            }
              bridge_last_delay=0;
              unilateral_bridge_state=0;
              unilateral_bridge_num=0;
              bridge_delay_cancle=0;
              unilateral_bridge_timer=0;
              Kp_X=1;
//              steering_KP_small=19;
//              steering_KP_large=41;
              speed=Normal_speed;
        }
      }

    else if(Zebra_State==1)
    {
        bridge_high=0;
        E_H_flag=0;
        Y_demand=40;
        speed=0;
    }
    else if(unilateral_bridge_state==0&&podao_state!=1)
     {
         bridge_high=0;
         E_H_flag=0;
        engine_slow(40);

     }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     横滚平衡计算函数
// 参数说明     无
// 返回参数     无
// 使用示例     roll_balance();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float roll_pwm=0;
void roll_balance(void)
{
    if(unilateral_bridge_state)roll_pwm=1.15*roll_angle_pid(0,eulerAngle.pitch);
    else roll_pwm=0;
}


