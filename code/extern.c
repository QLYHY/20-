/*
 * extern.c
 *
 *  Created on: 2025年4月27日
 *      Author: zhangjie
 */
#include "extern.h"

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];
uint8 line_state;
uint8 initiatal;
uint32 image_time;
uint16 first_state_1;
int first_lose[2],first_lastylose[2]; //[0]是左第1丢线，最后丢线；[1]是右第1丢线，最后丢线
int ips[20];
int aclture_distance,detour_integer,detour_state,detour_state_for,deter_time;
int right_barrier_up,right_barrier_down,left_barrier_up,left_barrier_down,barrier_point;
int right_bridge_up,right_bridge_down,left_bridge_up,left_bridge_down;
int right_jump_up,right_jump_down,left_jump_up,left_jump_down;
int del;
int last_distance;
int protect_flag;
int bridge_high,bridge_cancle,bridge_delay,bridge_delay_2,bridge_delay_cancle,bridge_err,bridge_last_delay;
int barreir_delay;
int pwm_servo;
int podao_state=0,podao_reday=0,podao_delay=0;
float high_leg;
void open_rgb565_image()
{
    if(initiatal==0)
    {
        ips200_show_rgb565_image(0, 0, (const uint16 *)gImage_doro_1, 240, 305, 240, 305, 0);    // 显示一个RGB565色彩图片 原图240*80 显示240*80 低位在前
        system_delay_ms(5000);
        ips200_clear();
    }
    initiatal=1;
}

uint8 key1_state = 1;                                                               // 按键动作状态
uint8 key2_state = 1;                                                               // 按键动作状态
uint8 key3_state = 1;                                                               // 按键动作状态
uint8 key4_state = 1;                                                               // 按键动作状态
uint8 key1_state_last;                                                          // 上一次按键动作状态
uint8 key2_state_last;                                                         // 上一次按键动作状态
uint8 key3_state_last;                                                         // 上一次按键动作状态
uint8 key4_state_last;                                                        // 上一次按键动作状态
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     Key_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Key_init()//按键与LED初始化
{
       gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
       gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
       gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
       gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描函数
// 参数说明     void
// 返回参数     void
// 使用示例     key_scan();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void key_scan()//按键扫描
{
        //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费

        //保存按键状态
        key1_state_last = key1_state;
        key2_state_last = key2_state;
        key3_state_last = key3_state;
        key4_state_last = key4_state;

        //读取当前按键状态
        key1_state = gpio_get_level(KEY1);
        key2_state = gpio_get_level(KEY2);
        key3_state = gpio_get_level(KEY3);
        key4_state = gpio_get_level(KEY4);


        //检测到按键按下之后  并放开置位标志位
        if(key1_state && !key1_state_last)   {key1_flag = 1;}
        if(key2_state && !key2_state_last)   {key2_flag = 1;}
        if(key3_state && !key3_state_last)   {key3_flag = 1;}
        if(key4_state && !key4_state_last)   {key4_flag = 1;}

        //标志位置位之后，可以使用标志位执行自己想要做的事件
       // system_delay_ms(10);//延时，按键程序应该保证调用时间不小于10ms
}

bool displayEnabled = false;  // 使用bool类型和更具描述性的变量名
bool displayEnabled1 = false;
bool displayEnabled3 = false;
bool displayEnabled4 = false;
int wish;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键识别函数
// 参数说明     void
// 返回参数     void
// 使用示例     key_work();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void key_work()
{
    if(key2_flag)  // 检测到按键按下
    {
        // 状态切换（更简洁的实现）
        displayEnabled = !displayEnabled;

        // 清除按键标志
        key2_flag = 0;
        ips[18]++;
        // 清屏操作
        ips200_clear();
    }
    if(key3_flag)  // 检测到按键按下
    {
        // 状态切换（更简洁的实现）
        displayEnabled3 = !displayEnabled3;
        wish--;
        ips[18]--;
        // 清除按键标志
        key3_flag = 0;

        // 清屏操作
        ips200_clear();
    }
    if(key4_flag)  // 检测到按键按下
    {
        // 状态切换（更简洁的实现）
        displayEnabled4 = !displayEnabled4;
        wish--;
        ips[18]--;
        // 清除按键标志
        key4_flag = 0;

        // 清屏操作
        ips200_clear();
    }
    if(key1_flag)  // 检测到按键按下
    {
        // 状态切换（更简洁的实现）
        wish--;
        ips[18]++;
        // 清除按键标志
        key1_flag = 0;

        // 清屏操作
        ips200_clear();
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数据显示函数（仅用于检查，正式发车无需使用)
// 参数说明     void
// 返回参数     void
// 使用示例     effect();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void effect()
{

    key_work();
//    if(displayEnabled3)aclture_distance=4000;
//    else if(!displayEnabled3)aclture_distance=0;

    if(!displayEnabled)
    {
        ips200_show_int(0,0,eulerAngle.roll,3);
        ips200_show_float(140,0,yaw,3,3);
//        ips200_show_float(100,0,gyro_z,5,3);//ips200_show_int(130,0,Left_Line[wish],3);ips200_show_int(150,0,Right_Line[wish],3);
        ips200_show_int(40,0,ips[6],5);
        //ips200_show_int(150,0,Right_Line[69],3);
        ips200_show_int(90,16*4,ips[7],2);
        ips200_show_int(140,16*4,ips[6],2);
        ips200_show_string(0,16*5,"angle:");
                    ips200_show_int(90,16*5,ips[1],2);
                    ips200_show_int(120,16*5,ips[2],2);
                    ips200_show_int(150,16*5,ips[3],2);
                    ips200_show_int(180,16*5,ips[4],2);
        ips200_show_string(0,16*1,"first_lose:");
            ips200_show_int(90,16*1,first_lose[0],2);
            ips200_show_int(120,16*1,first_lose[1],2);
            ips200_show_int(150,16*1,first_lastylose[0],2);
            ips200_show_int(180,16*1,first_lastylose[1],2);
        ips200_show_string(0,16*2,"barrier:");
                ips200_show_int(60,16*2,barrier_point,2);
                ips200_show_int(90,16*2,left_barrier_up,3);
                ips200_show_int(120,16*2,left_barrier_down,3);
                ips200_show_int(150,16*2,right_barrier_up,3);
                ips200_show_int(180,16*2,right_barrier_down,3);
        ips200_show_string(0,16*3,"bridge:");
            ips200_show_int(90,16*3,left_bridge_up,2);
            ips200_show_int(120,16*3,left_bridge_down,2);
            ips200_show_int(150,16*3,right_bridge_up,2);
            ips200_show_int(180,16*3,right_bridge_down,2);
        //ips200_show_int(30,208,motor_value.receive_data_count,5);
        // 此处编写需要循环执行的代码
        //printf("%d,%d,%d\r\n",imu660ra_gyro_x,imu660ra_acc_y,imp_posture_value.filtering_angle);
        //open_rgb565_image();
        ips200_show_gray_image(0,16*6,image_two_value[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H,0);

//      ips200_show_gray_image(0,210,bin_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H,0);
        //Left_Island_Flag==0&&Right_Island_Flag==0&&Cross_Flag==0&&Island_State==0&&unilateral_bridge_state==0&&jump_state==0Cross_Flag
//        ips200_show_int(100,16*12,unilateral_bridge_num,3);
//        ips200_show_int(0,16*12,leg_state_left,3);ips200_show_int(50,16*12,leg_state_right,3);
        ips200_show_int(100,16*12,podao_state,3);
        ips200_show_int(0,16*12,leg_state_left,3);ips200_show_int(50,16*12,leg_state_right,3);
        ips200_show_string(0,16*13,"all_state:");
            ips200_show_string(0,16*14,"yuan_state:");
                ips200_show_int(100,16*14,Left_Island_Flag,1);
                ips200_show_int(150,16*14,Right_Island_Flag,1);
                ips200_show_int(200,16*14,Island_State,1);
            ips200_show_string(0,16*15,"bridge_state:");
                ips200_show_int(110,16*15,unilateral_bridge_state,1);
                ips200_show_int(150,16*15,ips[8],2);
                ips200_show_int(180,16*15,ips[9],2);
            ips200_show_string(0,16*16,"jump_state:");
                ips200_show_int(100,16*16,jump_state,1);
                ips200_show_int(120,16*16,ips[11],2);
                ips200_show_int(140,16*16,ips[12],2);
                ips200_show_int(160,16*16,ips[13],2);
                ips200_show_int(180,16*16,Left_Line[85]-Left_Line[80],2);
            ips200_show_string(0,16*17,"cross:");
                ips200_show_int(150,16*17,Cross_Flag,1);
            ips200_show_string(0,16*18,"barrier:");
                ips200_show_int(150,16*18,barrier_state,1);


                //ips200_show_int(0,16*2,ips[3],2);
                ips200_show_int(0,16*4,aclture_distance,5);

//                ips200_show_int(0,16*6,ips[1],2);
//                ips200_show_int(40,16*6,ips[2],2);
//                ips200_show_int(80,16*6,ips[3],2);
//                ips200_show_int(120,16*6,ips[4],2);
//                ips200_show_int(0,16*7,ips[6],2);
//                //ips200_show_string(0,16*18,"3-4:");    ips200_show_int(50,16*18,ips[5],2); ips200_show_int(50,16*18,ips[7],2);
                ips200_show_string(0,16*19,"lost:");    ips200_show_int(50,16*19,Left_Lost_Time,2);  ips200_show_int(80,16*19,Right_Lost_Time,2);
    }
}
int wrong_distance_state;
void wrong_distance(int star_data, int *data, int *data_last, float data_p)
{
    // 数据有效性判断
    if (abs(*data - star_data) < 0.4 * star_data)
    {
        wrong_distance_state = 1;  // 数据有效
    }
    else
    {
        wrong_distance_state = 0;  // 数据无效
    }

    // 数据跳变检测与修正
    if (wrong_distance_state && abs(*data - *data_last) > data_p * *data_last)
    {
        // 数据跳变超过阈值，使用上一次有效值
        *data = *data_last;
    }

    // 更新历史数据（仅在数据有效时更新）
    if (wrong_distance_state)
    {
        *data_last = *data;
    }
}
