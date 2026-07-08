/*
 * menu.h
 *
 *  Created on: 2025年1月4日
 *      Author: lenovo
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#include "zf_device_type.h"

#define key1    P20_6
#define key2    P20_7
#define key3    P11_2
#define key4    P11_3


#define WIFI_SSID_TEST          "lyh123456"
#define WIFI_PASSWORD_TEST      "12345678"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.137.1"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666


typedef struct{
        char id[2];
        char zifu[50];
}caidan;

extern caidan menu[];
extern int menu_page_index[];
extern int aaa;
extern int begine_flag,podao_switch,detour_switching;

void menu_init(void);
void guangbiao_move(void);
void display_menu(void);
void WIFI_Init(void);

#endif /* CODE_MENU_H_ */
