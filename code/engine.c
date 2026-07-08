/*
 * engine.c
 *
 *  Created on: 2025年1月7日
 *      Author: lenovo
 */

#include "engine.h"
#include "math.h"

float PWM=0,PWM1=0,PWM2=0;
float X=0,Y=0,Y_demand=40;
float Y_L=0,Y_R=0;
//float Kp_Y=0.20,Kp_X=0.4,Kp_H=0.6;
float Kp_Y=1,Kp_X=1,Kp_H=0.6;
float E_H=0;

float engine_limit_max=L5*1.5-7,engine_limit_min=-L5;

engine engine_data = {0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度限幅
// 参数说明     angle           限幅角度
// 返回参数     float
// 使用示例     angle_limit(angle);
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float angle_limit(float angle)
{
    if(angle>=90)
    {
        angle=90;
    }
    if(angle<=-90)
    {
        angle=-90;
    }


    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM限幅
// 参数说明     pwm           限幅角度
// 返回参数     float
// 使用示例     PWM_limit(pwm);
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float PWM_limit(float pwm)
{
    if(pwm>=1240*6.6)
    {
        pwm=1240*6.6;
    }
    if(pwm<=260*6.6)
    {
        pwm=260*6.6;
    }
    return pwm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     无
// 返回参数     void
// 使用示例     engine_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void engine_init(void)
{
     pwm_init(engine_L1,300,4950);
     pwm_init(engine_L2,300,4950);
     pwm_init(engine_R1,300,4950);
     pwm_init(engine_R2,300,4950);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机角度控制
// 参数说明     ID              选择 舵机ID
// 参数说明     angle           设置舵机角度
// 返回参数     void
// 使用示例     engine_control(L1,45);    设置左1舵机角度为45°
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float last_angle=0;
void engine_control(engine_ID_enum ID,float angle)
{
    switch(ID)
    {
        case L1:
            PWM=1/90.0*-angle*3300+750*6;
            PWM=PWM_limit(PWM+20*6);
            pwm_set_duty(engine_L1,((int)(PWM+roll_pwm)/10)*10);
            break;
        case L2:
            PWM=1/90.0*angle*3300+750*6;
            PWM=PWM_limit(PWM);
            pwm_set_duty(engine_L2,((int)(PWM-roll_pwm)/10)*10);
            break;
        case R1:
            PWM=1/90.0*angle*3300+750*6;
            PWM=PWM_limit(PWM-0*6);
            pwm_set_duty(engine_R1,((int)(PWM+roll_pwm)/10)*10);
            break;
        case R2:
            PWM=1/90.0*-angle*3300+750*6;
            PWM=PWM_limit(PWM-20*6);
            pwm_set_duty(engine_R2,((int)(PWM-roll_pwm)/10)*10);
            break;
        case ALL:
            PWM1=1/90.0*angle*3300+750*6;
            PWM2=-1/90.0*angle*3300+750*6;
            PWM1=PWM_limit(PWM1);
            PWM2=PWM_limit(PWM2);
            pwm_set_duty(engine_L2,((int)PWM1/5*5));
            pwm_set_duty(engine_R2,((int)(PWM2+40*6)/5*5));
            pwm_set_duty(engine_R1,((int)(PWM1-0*6)/5*5));
            pwm_set_duty(engine_L1,((int)(PWM2+20*6)/5*5));


    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机跳跃
// 参数说明     无
// 返回参数     void
// 使用示例     engine_jump();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int jump_count=0,jump_timer_begine=0,jump_timer=0;
void engine_jump(void)
{
    if(jump_state==1&&jump_count==0&&unilateral_bridge_state==0)//
    {
//      engine_control(ALL,0);
        bend_gain=0;
//        engine_control(ALL,-90);
        engine_control(L1,-90);
        engine_control(R1,-90);
        engine_control(R2,-60);
        engine_control(L2,-60);
        system_delay_ms(100);
//        engine_control(ALL,30);
        engine_control(R2,30);
        engine_control(L2,30);
        engine_control(R1,10);
        engine_control(L1,10);

        system_delay_ms(120);
        engine_control(L1,-30);
        engine_control(R1,-30);
        engine_control(R2,-30);
        engine_control(L2,-30);
        system_delay_ms(80);
        engine_control(R2,10);
        engine_control(L2,10);
        engine_control(L1,0);
        engine_control(R1,0);


        jump_timer_begine=1;

        jump_state=0;
        jump_count=1;
        streeing_switch=0;

    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机角度控制
// 参数说明     ID              选择 舵机ID
// 参数说明     angle           设置舵机角度
// 返回参数     void
// 使用示例     engine_control(L1,45);    设置左1舵机角度为45°
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void engine_inverse_resolve(void)
{
    float a,b,c,d,e,f;

    float alpha1,alpha2,beta1,beta2;

    a = 2.0*engine_data.XLeft*leg1;
    b = 2.0*engine_data.YLeft*leg1;
    c = engine_data.XLeft*engine_data.XLeft+engine_data.YLeft*engine_data.YLeft+leg1*leg1-leg2*leg2*1.0;

    alpha1 = 2*atan((b+sqrt(a*a+b*b-c*c))/(a+c));
    alpha2 = 2*atan((b-sqrt(a*a+b*b-c*c))/(a+c));



    d = 2.0*(engine_data.XLeft - L5)*leg4;
    e = 2.0*engine_data.YLeft*leg4;
    f = (engine_data.XLeft - L5)*(engine_data.XLeft - L5) + leg4*leg4 +engine_data.YLeft*engine_data.YLeft -leg3*leg3*1.0;

    beta1 = 2*atan((e+sqrt(d*d + e*e - f*f))/(d+f));
    beta2 = 2*atan((e-sqrt(d*d + e*e - f*f))/(d+f));

    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

    if(alpha1>=PI/4)
        {engine_data.Left_alpha = alpha1;}
    else
        {engine_data.Left_alpha = alpha2;}

    if(beta1>=0&&beta1<=PI/4)         engine_data.Left_beta = beta1;
    else                              engine_data.Left_beta = beta2;



    engine_data.Left_alpha = (engine_data.Left_alpha*180/PI)-180;
    engine_data.Left_beta = -(engine_data.Left_beta*180/PI);



    a = 2.0*engine_data.XRight*leg1;
    b = 2.0*engine_data.YRight*leg1;
    c = engine_data.XRight*engine_data.XRight+engine_data.YRight*engine_data.YRight+leg1*leg1-leg2*leg2*1.0;

    alpha1 = 2*atan((b+sqrt(a*a+b*b-c*c))/(a+c));
    alpha2 = 2*atan((b-sqrt(a*a+b*b-c*c))/(a+c));

    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

    d = 2.0*(engine_data.XRight - L5)*leg4;
    e = 2.0*engine_data.YRight*leg4;
    f = (engine_data.XRight - L5)*(engine_data.XRight - L5) + leg4*leg4 +engine_data.YRight*engine_data.YRight -leg3*leg3*1.0;

    beta1 = 2*atan((e+sqrt(d*d + e*e - f*f))/(d+f));
    beta2 = 2*atan((e-sqrt(d*d + e*e - f*f))/(d+f));

    if(alpha1>=PI/4)         engine_data.Right_alpha = alpha1;
    else                                         engine_data.Right_alpha = alpha2;

    if(beta1>=0&&beta1<=PI/4)         engine_data.Right_beta = beta1;
    else                              engine_data.Right_beta = beta2;

    engine_data.Right_alpha = (engine_data.Right_alpha*180/PI)-180;
    engine_data.Right_beta = -(engine_data.Right_beta*180/PI);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     腿高限幅
// 参数说明     无
// 返回参数     void
// 使用示例     Leg_Height_Limit(Y_L);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float Leg_Height_Limit(float y)
{

    if(y>120)
    {
        y=120;
    }
    if(y<30)
    {
        y=30;
    }

    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     腿高解算并控制
// 参数说明     无
// 返回参数     void
// 使用示例     engine_console();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void engine_console(void)
{

    X = Kp_X*PWM_accel+L5/2-7;//

    Y_L = Y_L + Kp_Y * ((Y_demand+bend_gain) - Y_L);
    Y_R = Y_R + Kp_Y * ((Y_demand-bend_gain) - Y_R);

    if(X>engine_limit_max)X=engine_limit_max;
    if(X<engine_limit_min)X=engine_limit_min;

    engine_data.XLeft = X;
    engine_data.YLeft = Y_L;//+E_H;
    engine_data.XRight = X;
    engine_data.YRight = Y_R;//-E_H;

    engine_data.YLeft=Leg_Height_Limit(engine_data.YLeft);
    engine_data.YRight=Leg_Height_Limit(engine_data.YRight);

    engine_inverse_resolve();

    engine_data.Left_alpha=angle_limit(engine_data.Left_alpha);
    engine_data.Left_beta=angle_limit(engine_data.Left_beta);
    engine_data.Right_alpha=angle_limit(engine_data.Right_alpha);
    engine_data.Right_beta=angle_limit(engine_data.Right_beta);

    engine_control(L1,engine_data.Left_alpha);
    engine_control(L2,engine_data.Left_beta);
    engine_control(R1,engine_data.Right_alpha);
    engine_control(R2,engine_data.Right_beta);

}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     腿高输出滤波
// 参数说明     无
// 返回参数     void
// 使用示例     engine_output_fllter();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void engine_output_fllter(void)
{

//    static float last_Lalpha=0,last_Ralpha=0,last_Lbeta=0,last_Rbeta=0;
//
//    engine_data.Left_alpha=0.5*engine_data.Left_alpha+0.5*last_Lalpha;
//    last_Lalpha=engine_data.Left_alpha;
//
//    engine_data.Left_beta=0.5*engine_data.Left_beta+0.5*last_Lbeta;
//    last_Lbeta=engine_data.Left_beta;
//
//    engine_data.Right_alpha=0.5*engine_data.Right_alpha+0.5*last_Ralpha;
//    last_Ralpha=engine_data.Right_alpha;
//
//    engine_data.Right_beta=0.5*engine_data.Right_beta+0.5*last_Rbeta;
//    last_Rbeta=engine_data.Right_beta;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     腿高变化判断--用于退出单边桥
// 参数说明     无
// 返回参数     void
// 使用示例     engine_define();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int leg_state_left,leg_state_right,unilateral_bridge_num,leg_state,leg_same;
void engine_define(void)//
{

        if(unilateral_bridge_state)
        {
            if(roll_pwm>200)
            {
                leg_state_left=1;
            }
            else if(leg_state_left==1&&abs(roll_pwm)<150)
            {
                leg_state_left=2;
            }
            else if(leg_state_left==2)
            {
                unilateral_bridge_num++;
                leg_state_left=0;
            }

            if(roll_pwm<-200)
             {
                leg_state_right=1;
             }
             else if(leg_state_right==1&&roll_pwm>(-150))
             {
                 leg_state_right=2;
             }
             else if(leg_state_right==2)
             {
                 unilateral_bridge_num++;
                 leg_state_right=0;
             }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     缓慢变腿高
// 参数说明     无
// 返回参数     void
// 使用示例     engine_slow(70);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void engine_slow(int high)
{
    if(Y_demand<high)
    {
        if(bridge_delay%15==0)Y_demand += (high-Y_demand)/10;
    }
    else if(Y_demand>high)
    {
        if(bridge_delay_2%15==0)Y_demand -= (Y_demand-high)/10;
    }
    else if(abs(Y_demand-high)<3)Y_demand=(float)high;
}

float yaw_err=0.0f,yaw_err_last=0.0f,yaw_ture;
void yaw_amend(int high,int low)
{
//    yaw_err = 0.0f;
//    for(int i=high;i<low;i++)
//    {
//        yaw_err+=(Left_Line[i]+Right_Line[i])/2-90;
//    }
//    if(yaw_err)yaw_err/=10;
//    else yaw_err=0;
//    yaw_err=(float)((int)yaw_err*100)/100;
//    yaw_err=0.8*yaw_err+0.2*yaw_err_last;
//    yaw_err_last = yaw_err;
//    if(yaw_err>5)yaw_ture=5;
//    else if(yaw_err<-5)yaw_ture=-5;
//    else yaw_ture=0;
}
