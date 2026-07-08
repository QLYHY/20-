/*
 * pid.c
 *
 *  Created on: 2025年3月10日
 *      Author: lenovo
 */

#include "pid.h"

/*
 * 参数定义
 */

//float angle_KP=35,angle_KI=0.00015,angle_KD=0.03;
float angle_KP=45,angle_KI=0,angle_KD=0;


float angle_speed_KP=0.65,angle_speed_KI=0.038,angle_speed_KD=0.0;
//float angle_speed_KP=0.70,angle_speed_KI=0.028,angle_speed_KD=0; //1.5 0.026 0//0.61 0.0225 0.03

//float Balance_Velocity_KP=-0.75,Balance_Velocity_KI=0,Balance_Velocity_KD=0.0;    //增量式
float Balance_Velocity_KP=0.04,Balance_Velocity_KI=-0.045,Balance_Velocity_KD=-0.018;//位置式
//float Balance_Velocity_KP=0.38,Balance_Velocity_KI=0.025,Balance_Velocity_KD=-0.0;
//float Balance_Velocity_KP=-0.012,Balance_Velocity_KI=-0.058,Balance_Velocity_KD=-0.0;
//450速度转向环
//float steering_KP=25,steering_KP2=0,steering_KD=0,steering_GKD=8;
float steering_KP=23,steering_KP2=0.8,steering_KD=0,steering_GKD=0.09;

//float steering_KP_small=7,steering_KP_large=32,steering_KD_small=0.02,steering_KD_large=0.85;
//轮子在大转弯过程中打滑跳轮加D则可抑制
//float steering_KP_small=28,steering_KP_large=52,steering_KD_small=0.025,steering_KD_large=1.0;
//float steering_KP=20,steering_KP2=0,steering_KD=0,steering_GKD=0.6;//单调下面那个时发现刚出弯道走直线会发生左右摆动，
////这时就可以调上面这个，加P就可以解决以上问题
//
////float steering_KP=23.95,steering_KP2=5,steering_KD=2,steering_GKD=5;
//float steering_KP_small=5,steering_KP_large=25,steering_KD_small=0.1,steering_KD_large=0.5;
float yaw_angle_KP=88.8,yaw_angle_KI=0.0,yaw_angle_KD=180.5,yaw_angle_GKD=1.4;        //航向角pid参数，用于单边桥的闭环方向

float Steering_compensation=0,Steering_compensation_L=0,Steering_compensation_R=0;
float check,check1,check2,check3,check4,speed_error=0;

//float roll_angle_KP=0.2,roll_angle_KI=0.020,roll_angle_KD=0.2,roll_angle_GKD=-0.3;
float roll_angle_KP=-60,roll_angle_KI=0,roll_angle_KD=50;
float roll_angle_speed_KP=0,roll_angle_speed_KI=0,roll_angle_speed_KD=0.1;


float bend_KP=0.0012,bend_KI=0.0061,bend_KD=0;

float bend_gain=0;

float Err_streeing=0;

float Balance_Velocity_KP_TEMP=0.04,Balance_Velocity_KI_TEMP=-0.045,Balance_Velocity_KD_TEMP=-0.025;

int Distance_Measure(void)
{
    int distance;
    ips[8] = distance;

    distance = (right_speed_data-left_speed_data)/2;

    return distance;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度环 PID
// 参数说明     Angle           当前角度
// 参数说明     Angle_Zero      目标角度
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float angle_pid(float Angle,float Angle_Zero)
{
    float PWM,Bias;
    static float Bias_last=0,Bias_last2=0,Bias_integral=0;


    Bias=Angle-Angle_Zero+0.5;
    Bias_integral+=Bias;

    if(Bias_integral>5000)Bias_integral=5000;
    if(Bias_integral<-5000)Bias_integral=-5000;

    PWM=angle_KP*Bias+angle_KI*Bias_integral+angle_KD*(Bias-Bias_last);//位置式

    Bias_last=Bias;

    if(PWM>1000)PWM=1000;
    if(PWM<-1000)PWM=-1000;
    //此处可加限幅
    return PWM;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角速度环增量式PID
// 参数说明     Gyro            当前角速度
// 参数说明     PWM_expectation 目标角速度
// 返回参数     float           输出的PWM增量
// 使用示例     angle_speed_pid_inc(目标角速度, 当前角速度);
// 备注信息     增量式PID，需保存上一次和上上次的误差
//-------------------------------------------------------------------------------------------------------------------
float angle_speed_pid(float PWM_expectation, float Gyro)
{
    static float bias_last = 0.0f;    // 上一次误差
    static float bias_last2 = 0.0f;   // 上上次误差
    static float pwm_last = 0.0f;     // 上一次输出值

    float bias = PWM_expectation - Gyro;  // 当前误差

    // 增量式PID公式:
    // Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
    float delta_pwm = angle_speed_KP * (bias - bias_last)
                    + angle_speed_KI * bias
                    + angle_speed_KD * (bias - 2*bias_last + bias_last2);

    // 更新历史误差
    bias_last2 = bias_last;
    bias_last = bias;

    // 计算当前输出值 = 上一次输出 + 增量
    float pwm = pwm_last + delta_pwm;

    // 输出限幅 (根据实际需求调整限幅值)
    if(pwm > 8000) pwm = 8000;
    if(pwm < -8000) pwm = -8000;

    // 保存当前输出值供下次使用
    pwm_last = pwm;

    return pwm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     平衡速度环 PID
// 参数说明     encoder     当前速度
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float Balance_Velocity_pid(int expect_speed,int encoder)
{
    static float Encoder_Last=0,Encoder_Last2=0,Encoderadd=0;
    float Encoder,Velocity=0;
    float err=(expect_speed-encoder)*1.0;

    //一阶低通滤波

    Encoder = 0.8*err + Encoder_Last*0.2;

    Encoderadd+=Encoder;
    if(Encoderadd>80)Encoderadd=80;
    else if(Encoderadd<-80)Encoderadd=-80;
    Velocity += Balance_Velocity_KP * (Encoder - Encoder_Last) + Balance_Velocity_KI * Encoder +Balance_Velocity_KD * (Encoder - 2 * Encoder_Last + Encoder_Last2);

    if(Velocity>50)Velocity=50;
    if(Velocity<-50)Velocity=-50;

    Encoder_Last2 = Encoder_Last;
    Encoder_Last=(float)Encoder*1.0;

    return Velocity;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向环 PID
// 参数说明     ExpectAngle     期望转角
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------

float steering_pid(float ExpectAngle)
{
    float turn;
    float error = ExpectAngle;  // 当前误差
    static float last_error = 0.0f;  // 上一次的误差

    turn = steering_KP * ExpectAngle + error * fabs(error) * steering_KP2 + steering_KD * (error - last_error) + steering_GKD * icm_data.gyro_z *1500;

    // 输出限幅
    if (turn > 8000.0f) turn = 8000.0f;
    if (turn < -8000.0f) turn = -8000.0f;

    last_error = error;  // 更新上一次误差

    return turn;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向优化
// 参数说明     无
// 返回参数     无
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float last_bend_gain=0,last_err=0,ERR_bend=0;
void streeing_optimize(void)
{
    if(streeing_switch==0)
    {
        err=-Err_Sum();
//        if(Island_State!=0)
//        {
//            err=0.7*err+0.3*last_err;
//        }
            last_err=err;
            ERR_bend=0;
            if(unilateral_bridge_state==0)
            {

                bend_gain=(0.7*(bend_pid(ERR_bend))+0.3*last_bend_gain);

                last_bend_gain=bend_gain;
            }

        Steering_compensation=steering_pid(err);
    }
    else
    {
        err=0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     航向角 PID
// 参数说明     Angle           当前角度
// 参数说明     Angle_Zero      目标角度
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float yaw_angle_pid(float to_yaw,float act_yaw)
{
    float PWM,Bias;
    static float Bias_last=0,Bias_last2=0,Bias_integral=0;

    //Bias=Angle-Angle_Zero-4;
    Bias=to_yaw-act_yaw;
    Bias_integral+=Bias;

    if(Bias_integral>5000)Bias_integral=5000;
    if(Bias_integral<-5000)Bias_integral=-5000;

    PWM=yaw_angle_KP*Bias+yaw_angle_KI*Bias_integral+yaw_angle_KD*(Bias-Bias_last)+
            yaw_angle_GKD * icm_data.gyro_x *1500;;//位置式

    Bias_last=Bias;

    if(PWM>1000)PWM=1000;
    if(PWM<-1000)PWM=-1000;
    //此处可加限幅
    return PWM;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     横滚角度环 PID
// 参数说明     Angle           当前角度
// 参数说明
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float roll_angle_pid(float to_roll, float Angle)
{
    static float roll_Bias_last = 0.0f;    // 上一次误差
    static float roll_Bias_last2 = 0.0f;   // 上上次误差
    static float roll_PWM_last = 0.0f;     // 上一次输出值
    float roll_PWM = 0, roll_Bias = 0;

    // 计算当前误差
    roll_Bias = to_roll - Angle;

    roll_Bias=0.1*roll_Bias+0.9*roll_Bias_last;

    // 死区处理 - 根据实际系统调整阈值
    if(fabs(roll_Bias) < 0.2f)  // 建议调整为1度左右(根据单位)
    {
        roll_Bias = 0;
    }

    // 增量式PID计算
    roll_PWM = roll_angle_KP * roll_Bias
                    //+ roll_angle_KI * roll_Bias  // 积分项(注意：可能需要累积历史误差)
                    + roll_angle_KD * (roll_Bias - roll_Bias_last);

    roll_PWM=0.8*roll_PWM+0.2*roll_PWM_last;

    // 输出限幅 - 根据执行器能力设置
    if(roll_PWM > 2000) roll_PWM = 2000;
    if(roll_PWM < -2000) roll_PWM = -2000;
    roll_PWM_last=roll_PWM;
    // 更新历史值
    //roll_PWM_last = roll_PWM;
    //roll_Bias_last2 = roll_Bias_last;
    roll_Bias_last = roll_Bias;

    return roll_PWM;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     横滚角度环 PID
// 参数说明     Angle           当前角度
// 参数说明
// 返回参数     float
// 使用示例
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
float roll_angle_speed_pid(float expect_gyro,float gyro)
{
    static float roll_Bias_last = 0.0f;    // 上一次误差
    static float roll_Bias_last2 = 0.0f;   // 上上次误差
    static float roll_PWM_last = 0.0f;     // 上一次输出值
    float roll_PWM = 0, roll_Bias = 0;

    // 计算当前误差
    roll_Bias = expect_gyro-gyro;

    roll_Bias=0.5*roll_Bias+0.5*roll_Bias_last;

    // 死区处理 - 根据实际系统调整阈值
//    if(fabs(roll_Bias) < 0.05f)  // 建议调整为1度左右(根据单位)
//    {
//        roll_Bias = 0;
//    }

    // 增量式PID计算
    float delta_PWM = roll_angle_speed_KP * (roll_Bias - roll_Bias_last)
                    + roll_angle_speed_KI * roll_Bias  // 积分项(注意：可能需要累积历史误差)
                    + roll_angle_speed_KD * (roll_Bias - 2*roll_Bias_last + roll_Bias_last2);

    // 输出限幅 - 根据执行器能力设置

    if(roll_PWM > 45) roll_PWM = 45;
    if(roll_PWM < -45) roll_PWM = -45;
    // 更新历史值
    roll_PWM_last = roll_PWM;
    roll_Bias_last2 = roll_Bias_last;
    roll_Bias_last = roll_Bias;

    roll_PWM = roll_PWM_last + delta_PWM;

    return roll_PWM;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     压弯增量式PID
// 参数说明     ERR 图像误差
// 返回参数     float           压弯增益
// 使用示例     angle_speed_pid_inc(目标角速度, 当前角速度);
// 备注信息     增量式PID，需保存上一次和上上次的误差
//-------------------------------------------------------------------------------------------------------------------
float bend_pid(float ERR)
{
    static float bias_last = 0.0f;    // 上一次误差
    static float bias_last2 = 0.0f;   // 上上次误差
    static float pwm_last = 0.0f;     // 上一次输出值

    float bias = ERR;  // 当前误差

    bias = bias *0.1+bias_last*0.9;

    // 增量式PID公式:
    // Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
    float delta_pwm = bend_KP * (bias - bias_last)
                    + bend_KI * bias
                    + bend_KD * (bias - 2*bias_last + bias_last2);

    // 更新历史误差
    bias_last2 = bias_last;
    bias_last = bias;

    if(fabs(ERR_bend)<=5)
    {
        pwm_last*=0.9;
    }
    check2=delta_pwm;
    // 计算当前输出值 = 上一次输出 + 增量
    float pwm = pwm_last + delta_pwm;
    check1=pwm;
    // 输出限幅 (根据实际需求调整限幅值)
    if(pwm > 5) pwm = 5;
    if(pwm < -5) pwm = -5;

    // 保存当前输出值供下次使用
    pwm_last = pwm;

    return pwm;
}


