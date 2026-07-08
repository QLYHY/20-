/*
 * menu.c
 *
 *  Created on: 2025年1月4日
 *      Author: lenovo
 */

#include "menu.h"

int begine_flag=0;
int podao_switch=1;
int detour_switching=0;

//菜单显示内容结构体
caidan menu[] = {
        {"1","Speed Ring"},
        {"11","speed:"},
        {"12","kp:"},
        {"13","ki:"},
        {"14","kd:"},

        {"2","Upright angle Ring"},
        {"21","kp:"},
        {"22","ki:"},
        {"23","kd:"},

        {"3","Upright angle speed Ring"},
        {"31","kp:"},
        {"32","ki:"},
        {"33","kd:"},

        {"4","Steering Ring"},
        {"41","steering_KP:"},
        {"42","steering_KP2:"},
        {"43","steering_KD:"},
        {"44","steering_GKD:"},
        {"45","Gkd:"},

        {"5","jump&exposure"},
        {"51","jump_distance:"},//leg_high
        {"52","leg_high:"},

        {"6","bend Ring"},
        {"61","kp:"},
        {"62","ki:"},
        {"63","kd:"},

        {"7","Begine"},
        {"71","podao_switch"},//Element_Order
        {"72","Element_Order"},
        {"73","aclture_distance"},
        {"72","detour_state"},
        {"73","detour_state_for"}

};

int gb_flag = 3;                                    // 光标位置
int max_cursor_positions;                           // 光标最大的移动位置
int flag_main = 1;                                  // 页面位于主菜单标志位
int flag_menu = 0;                                  // 页面位于子菜单标志位
int a=0;
int menu_page_index[2];
int aclture_distance_flag=0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单初始化函数(混杂了参数调整)
// 参数说明     void
// 返回参数     void
// 使用示例     menu_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu_init(void)
{

    if(flag_main == 1 && flag_menu == 0)
    {
        max_cursor_positions = 7;
        ips200_show_string(16*5,16*1,"Main Menu");
        ips200_show_string(16*1,16*3,menu[0].zifu);
        ips200_show_string(16*1,16*4,menu[5].zifu);
        ips200_show_string(16*1,16*5,menu[9].zifu);
        ips200_show_string(16*1,16*6,menu[13].zifu);
        ips200_show_string(16*1,16*7,menu[19].zifu);
        ips200_show_string(16*1,16*8,menu[22].zifu);
        ips200_show_string(16*1,16*9,menu[26].zifu);
//        ips200_show_string(16*1,16*10,menu[28].zifu);
//        ips200_show_string(16*1,16*11,menu[29].zifu);
    }
    switch(flag_menu)
    {
        case 1:
            max_cursor_positions = 4;
            ips200_show_string(16*5,16*1,"Speed Ring");
            ips200_show_string(16*1,16*3,menu[1].zifu);
            ips200_show_int       (16*4, 16*3, Normal_speed ,3);
            ips200_show_string(16*1,16*4,menu[2].zifu);
            ips200_show_float     (16*4, 16*4, Balance_Velocity_KP_TEMP ,3 ,5);
            ips200_show_string(16*1,16*5,menu[3].zifu);
            ips200_show_float     (16*4, 16*5, Balance_Velocity_KI_TEMP ,3 ,5);
            ips200_show_string(16*1,16*6,menu[4].zifu);
            ips200_show_float     (16*4, 16*6, Balance_Velocity_KD_TEMP ,3 ,5);

        break;
        case 2:
            max_cursor_positions = 3;
            ips200_show_string    (16*3,16*1,"Upright angle Ring");
            ips200_show_string    (16*1,16*3,menu[6].zifu);
            ips200_show_float     (16*4, 16*3, angle_KP ,3 ,5);
            ips200_show_string    (16*1,16*4,menu[7].zifu);
            ips200_show_float     (16*4, 16*4, angle_KI ,3 ,5);
            ips200_show_string    (16*1,16*5,menu[8].zifu);
            ips200_show_float     (16*4, 16*5, angle_KD ,3 ,5);
        break;
        case 3:
            max_cursor_positions = 3;
            ips200_show_string    (16*2,16*1,"Upright angle speed Ring");
            ips200_show_string    (16*1,16*3,menu[10].zifu);
            ips200_show_float     (16*4, 16*3, angle_speed_KP ,3 ,5);
            ips200_show_string    (16*1,16*4,menu[11].zifu);
            ips200_show_float     (16*4, 16*4, angle_speed_KI ,3 ,5);
            ips200_show_string    (16*1,16*5,menu[12].zifu);
            ips200_show_float     (16*4, 16*5, angle_speed_KD ,3 ,5);
        break;
        case 4:
            max_cursor_positions = 5;
            ips200_show_string    (16*5,16*1,"Steering Ring");
            ips200_show_string    (16*1,16*3,menu[14].zifu);
            ips200_show_float       (16*8,16*3 , steering_KP, 3,5);
            ips200_show_string    (16*1,16*4,menu[15].zifu);
            ips200_show_float       (16*8,16*4 , steering_KP2, 3,5);
            ips200_show_string    (16*1,16*5,menu[16].zifu);
            ips200_show_float       (16*8,16*5 , steering_KD, 3,5);
            ips200_show_string    (16*1,16*6,menu[17].zifu);
            ips200_show_float       (16*8,16*6 , steering_GKD, 3,5);

        break;
        case 5:
            max_cursor_positions = 2;
            ips200_show_string    (16*6,16*1,"jump&exposure");
            ips200_show_string    (16*1,16*3,menu[20].zifu);
            ips200_show_int       (16*9, 16*3, jump_begin_distance ,3);//leg_high
            ips200_show_string    (16*1,16*4,menu[21].zifu);
            ips200_show_int       (16*9, 16*4, leg_high ,3);

        break;
        case 6:
            max_cursor_positions = 3;
            ips200_show_string    (16*6,16*1,"bend Ring");
            ips200_show_string    (16*1,16*3,menu[23].zifu);
            ips200_show_float       (16*5, 16*3, bend_KP ,3,5);
            ips200_show_string    (16*1,16*4,menu[24].zifu);
            ips200_show_float       (16*5, 16*4, bend_KI ,3,5);
            ips200_show_string    (16*1,16*5,menu[25].zifu);
            ips200_show_float       (16*5, 16*5, bend_KD ,3,5);
        break;
        case 7:
            max_cursor_positions = 9;
            ips200_show_string  (16*5,  16*1,   "begine");
            ips200_show_string    (16*1,16*3,"begine confirm again");
            ips200_show_string    (16*1,16*4,"Podao switch");
            ips200_show_string    (16*1,16*5,"Element Order--NULL");
            ips200_show_string    (16*1,16*6,"Element Order--Obstacle");
            ips200_show_string    (16*1,16*7,"Element Order--Bridge");
            ips200_show_string    (16*1,16*8,"aclture_distance:");
            ips200_show_string    (16*1,16*9,"detour_state");
            ips200_show_string    (16*1,16*10,"detour_state_for");

            ips200_show_int       (16*8, 16*4, podao_switch ,3);
            ips200_show_int       (16*12, 16*8, aclture_distance ,3);
            ips200_show_int       (16*12, 16*9, detour_state ,3);
            ips200_show_int       (16*12, 16*10, detour_state_for ,3);

            ips200_show_int       (16*6, 16*12, Element_Order[0] ,3);
            ips200_show_int       (16*7, 16*12, Element_Order[1] ,3);
            ips200_show_int       (16*8, 16*12, Element_Order[2] ,3);
            ips200_show_int       (16*9, 16*12, Element_Order[3] ,3);

            ips200_show_gray_image(16*2,16*13,image_two_value[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H,0);

            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&& menu_page_index[1] == 3)
            {
                while(!gpio_get_level(key2)){
                }
                for(int32 i=3;i<7;i++)
                {
                    flash_write_Element_Order(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX);
                }
                begine_flag=1;
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&& menu_page_index[1] == 4)
            {
                while(!gpio_get_level(key2)){}
                podao_switch^=1;
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&& (menu_page_index[1] == 5 || menu_page_index[1] == 6 || menu_page_index[1] == 7))
            {
                while(!gpio_get_level(key2)){}
                Element_Order[a]=menu_page_index[1]-5;
                a++;
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&&menu_page_index[1] == 8 )
            {
                while(!gpio_get_level(key2)){}
                aclture_distance_flag^=1;
                if(aclture_distance_flag==0)
                {
                    aclture_distance=0;
                }
                else if(aclture_distance_flag==1)
                {
                    aclture_distance=250;
                }
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&&menu_page_index[1] == 9 )
            {
                while(!gpio_get_level(key2)){}
                detour_state^=1;
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&&menu_page_index[1] == 10 )
            {
                while(!gpio_get_level(key2)){}
                detour_state_for^=1;
            }
            if(!gpio_get_level(key2)&&menu_page_index[0] == 9&&menu_page_index[1] == 11 )
            {
                while(!gpio_get_level(key2)){}
                detour_switching^=1;
            }
        break;
    }

}

int modify_pwm_flag = 0;                                // 表示PWM修改是否激活的标志
int modify_pwm_index;                                   // 存储当前正在修改的PWM索引
int modify_zhili_flag = 0;                              // 表示直立环PD修改是否激活的标志
int modify_zhili_index;                                 // 存储当前正在修改的直立环PD索引

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     光标移动函数
// 参数说明     void
// 返回参数     void
// 使用示例     guangbiao_move();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void guangbiao_move(void)
{
        ips200_show_string(16*13, 16*gb_flag, "<-");// 显示新位置的光标
//        ips200_show_string(16*13, 16*gb_flag, "   "); // 消除原有光标
        if (!gpio_get_level(key1)&&((menu_page_index[0] != 0&& menu_page_index[1] != 0)
                &&!(menu_page_index[0] == 9))){
            if ( menu_page_index[0] == 3&& menu_page_index[1] == 3)
                {Normal_speed += 10.0;
                }
            else if (menu_page_index[0] == 3&& menu_page_index[1] == 4)
                {Balance_Velocity_KP_TEMP += 0.01;
                }
            else if (menu_page_index[0] == 3&& menu_page_index[1] == 5)
                {Balance_Velocity_KI_TEMP += 0.01;

                }
            else if (menu_page_index[0] == 3&& menu_page_index[1] == 6)
                {Balance_Velocity_KD_TEMP += 0.001;
                }

            if ( menu_page_index[0] == 4&& menu_page_index[1] == 3)
                {angle_KP += 2.0;
                }
            else if (menu_page_index[0] == 4&& menu_page_index[1] == 4)
                {angle_KI += 0.5;
                }
            else if (menu_page_index[0] == 4&& menu_page_index[1] == 5)
                {angle_KD += 0.5;
                }

            if ( menu_page_index[0] == 5&& menu_page_index[1] == 3)
                {angle_speed_KP += 0.01;
                }
            else if (menu_page_index[0] == 5&& menu_page_index[1] == 4)
                {angle_speed_KI += 0.01;
                }
            else if (menu_page_index[0] == 5&& menu_page_index[1] == 5)
                {angle_speed_KD += 0.01;
                }

            if ( menu_page_index[0] == 6&& menu_page_index[1] == 3)
                {
                steering_KP += 1.0;
                }
            else if (menu_page_index[0] == 6&& menu_page_index[1] == 4)
                {
                steering_KP2 += 0.01;
                }
            else if (menu_page_index[0] == 6&& menu_page_index[1] == 5)
                {
                steering_KD += 0.1;
                }
            else if (menu_page_index[0] == 6&& menu_page_index[1] == 6)
                {
                steering_GKD += 0.001;
                }
            else if (menu_page_index[0] == 6&& menu_page_index[1] == 7)
                {
                }

            if ( menu_page_index[0] == 7&& menu_page_index[1] == 3)
                {jump_begin_distance += 1.0;
                }
            else if (menu_page_index[0] == 7&& menu_page_index[1] == 4)
                {leg_high += 1.0;
                }

            if ( menu_page_index[0] == 8&& menu_page_index[1] == 3)
                {bend_KP += 0.0001;
                }
            else if (menu_page_index[0] == 8&& menu_page_index[1] == 4)
                {bend_KI += 0.0001;
                }
            else if (menu_page_index[0] == 8&& menu_page_index[1] == 5)
                {bend_KD += 0.0001;
                }


            while (!gpio_get_level(key1)) {} // 等待按键释放
        }
        else if (!gpio_get_level(key4)&&((menu_page_index[0] != 0&& menu_page_index[1] != 0)
                &&!(menu_page_index[0] == 9 ))){
            if ( menu_page_index[0] == 3&& menu_page_index[1] == 3)
            {Normal_speed
                -= 10.0;
            }

        else if (menu_page_index[0] == 3&& menu_page_index[1] == 4)
            {Balance_Velocity_KP_TEMP -= 0.01;
            }
        else if (menu_page_index[0] == 3&& menu_page_index[1] == 5)
            {Balance_Velocity_KI_TEMP -= 0.01;
            }

        else if (menu_page_index[0] == 3&& menu_page_index[1] == 6)
            {Balance_Velocity_KD_TEMP -= 0.001;
            }

        if ( menu_page_index[0] == 4&& menu_page_index[1] == 3)
            {angle_KP -= 2.0;
            }

        else if (menu_page_index[0] == 4&& menu_page_index[1] == 4)
            {angle_KI -= 0.5;
            }

        else if (menu_page_index[0] == 4&& menu_page_index[1] == 5)
            {angle_KD -= 0.5;
            }

        if ( menu_page_index[0] == 5&& menu_page_index[1] == 3)
            {angle_speed_KP -= 0.01;
            }

        else if (menu_page_index[0] == 5&& menu_page_index[1] == 4)
            {angle_speed_KI -= 0.01;
            }

        else if (menu_page_index[0] == 5&& menu_page_index[1] == 5)
            {angle_speed_KD -= 0.01;
            }


        if ( menu_page_index[0] == 6&& menu_page_index[1] == 3)
            {
            steering_KP += 1.0;
            }

        else if (menu_page_index[0] == 6&& menu_page_index[1] == 4)
            {
            steering_KP2 -= 1.0;
            }

        else if (menu_page_index[0] == 6&& menu_page_index[1] == 5)
            {
            steering_KD -= 0.01;
            }

        else if (menu_page_index[0] == 6&& menu_page_index[1] == 6)
            {
            steering_GKD -= 0.001;
            }

        else if (menu_page_index[0] == 6&& menu_page_index[1] == 7)
            {
            }


        if ( menu_page_index[0] == 7&& menu_page_index[1] == 3)
            {jump_begin_distance -= 1.0;
            }

        else if (menu_page_index[0] == 7&& menu_page_index[1] == 4)
            {leg_high -= 1.0;
            }

        if ( menu_page_index[0] == 8&& menu_page_index[1] == 3)
            {bend_KP -= 0.0001;
            }

        else if (menu_page_index[0] == 8&& menu_page_index[1] == 4)
            {bend_KI -= 0.0001;
            }

        else if (menu_page_index[0] == 8&& menu_page_index[1] == 5)
            {bend_KD -= 0.0001;
            }

            while (!gpio_get_level(key4)) {} // 等待按键释放
        }
//        ips200_show_string(16*13, 16*gb_flag, "   ");       // 消除原有光标
        if (!gpio_get_level(key1))                          // 检测按键并移动光标
        {
            gb_flag--;
            if((menu_page_index[0] == 9 && (menu_page_index[1] == 5 || menu_page_index[1] == 6 || menu_page_index[1] == 7)))menu_page_index[1]=0;
            else if((menu_page_index[0] == 9 && (menu_page_index[1] == 4 ||menu_page_index[1] == 8 || menu_page_index[1] == 9 || menu_page_index[1] == 10)))menu_page_index[1]=gb_flag;
            ips200_clear();
            while(!gpio_get_level(key1))
            {}
        }
        else if (!gpio_get_level(key4))
        {
            gb_flag++;
            if((menu_page_index[0] == 9 && (menu_page_index[1] == 5 || menu_page_index[1] == 6 || menu_page_index[1] == 7)))menu_page_index[1]=0;
            else if((menu_page_index[0] == 9 && (menu_page_index[1] == 4 ||menu_page_index[1] == 8 || menu_page_index[1] == 9 || menu_page_index[1] == 10)))menu_page_index[1]=gb_flag;
            ips200_clear();
            while(!gpio_get_level(key4))
            {}
        }
        if(gb_flag < 3)
            gb_flag = 2 + max_cursor_positions;
        else if(gb_flag > 2 + max_cursor_positions)
            gb_flag = 3;

        ips200_show_string(16*13, 16*gb_flag, "<-");    // 显示新位置的光标

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单更新函数
// 参数说明     void
// 返回参数     void
// 使用示例     display_menu();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int aaa=0;
void display_menu(void)
{
    if (!gpio_get_level(key2) && flag_menu == 0)                          // 检测按键并移动光标
    {
        switch(gb_flag)
        {
            case 3: flag_menu = 1;  ips200_clear();  break;
            case 4: flag_menu = 2;  ips200_clear();  break;
            case 5: flag_menu = 3;  ips200_clear();  break;
            case 6: flag_menu = 4;  ips200_clear();  break;
            case 7: flag_menu = 5;  ips200_clear();  break;
            case 8: flag_menu = 6;  ips200_clear();  break;
            case 9: flag_menu = 7;  ips200_clear();  break;
        }
        menu_page_index[0]=gb_flag;
        gb_flag=3;
        while (!gpio_get_level(key2)) {}
    }
    else if(!gpio_get_level(key2) && flag_menu != 0)
    {
        menu_page_index[1]=gb_flag;
    }
    if (!gpio_get_level(key3))                          // 检测按键并移动光标
    {
        flag_main = 1;
        flag_menu = 0;
        menu_page_index[0] = 0;
        menu_page_index[1] = 0;
        ips200_clear();
        while (!gpio_get_level(key3)) {}
    }

}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无线串口初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     WIFI_Init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void WIFI_Init(void)
{
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
        break;
    }
//
    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
            break;
        }

    }
}

