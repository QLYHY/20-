/*
 * source.c
 *
 *  Created on: 2025年3月18日
 *      Author: lenovo
 */
#include "source.h"

int system_count=0;
float err,dl1b_distance_cm=0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     cc60_ch1中断回调函数（1ms）
// 参数说明     无
// 返回参数     void
// 使用示例     无需手动调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void cc60_pit_ch1_isr_callback(void)
{//dl1a_get_distance
    if(begine_flag==1)
    {

        if(system_count>=50000)
        {
            system_count=0;
        }

        unilateral_bridge();
        if(unilateral_bridge_state)
        {
            bridge_last_delay++;
            bridge_delay++;
            if(abs(roll_pwm)<50)bridge_err++;
            else if(abs(roll_pwm)>200)bridge_err=0;
        }
        else if(!unilateral_bridge_state)bridge_delay_2++;
        if(unilateral_bridge_num==3){bridge_delay_cancle++;}
        if(barrier_state&&!barrier_point)barreir_delay++;
        if(podao_state)podao_delay++;

        if(system_count%1 == 0)
        {
            if(del>=200)jiaosuduhuan();

            engine_console();
        }
        if(system_count%5 == 0)
        {
            streeing_optimize();
            if(del>=200)jiaoduhuan();
        }
        if(system_count%10 == 0)
        {
            if(fabs(err<=10))
            {
                dl1b_get_distance();
            }
        }

        if(system_count%15 == 0)
        {
            roll_balance();
        }
        if(system_count%20 == 0)
        {
            velocity=Distance_Measure();
                ips[4]=velocity;
             if(del>=200)suduhuan();
        }

        if(unilateral_bridge_state==1)
        {
            unilateral_bridge_timer++;
        }

        if(jump_state==1&&jump_count==0)
        {
            engine_jump();
        }
        system_count++;
    }
}
float max_angle=0;
int max_angle_start_flag=0;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     cc61_ch1中断回调函数（1ms）
// 参数说明     无
// 返回参数     void
// 使用示例     无需手动调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void cc61_pit_ch1_isr_callback(void)
{
    if(begine_flag==1)
    {
        if(zaber_start_flag==0&&aclture_distance>=1000)
        {
            zaber_start_flag=1;
            max_angle_start_flag=1;
            aclture_distance=0;
        }
        if(abs(max_angle)<abs(eulerAngle.roll+1.5)&&max_angle_start_flag==1)
        {
            max_angle=eulerAngle.roll+1.5;
        }
        if(dl1b_finsh_flag==1)
        {
            if(dl1b_distance_mm<=545)
            {
                dl1b_distance_cm=(float)(dl1b_distance_mm/10-4);
            }
            else
            {
        //            dl1b_distance_mm=0;
                dl1b_distance_cm=0;
            }
            dl1a_finsh_flag=0;
        }

        if(jump_timer_begine==1)
        {
            jump_timer++;
            if(jump_timer<650)
            {
//                Balance_Velocity_KP=Balance_Velocity_KP_TEMP/2;
//                Balance_Velocity_KI=Balance_Velocity_KI_TEMP/2;
//                Balance_Velocity_KD=Balance_Velocity_KD_TEMP/2;
                speed=600;
            }
            else
            {
//                Balance_Velocity_KP=Balance_Velocity_KP_TEMP;
//                Balance_Velocity_KI=Balance_Velocity_KI_TEMP;
//                Balance_Velocity_KD=Balance_Velocity_KD_TEMP;
                speed=Normal_speed;
                jump_timer_begine=0;
                jump_timer=0;
            }
        }
    }
}

