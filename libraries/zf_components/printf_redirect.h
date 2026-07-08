/*
 * printf_redirect.h
 *
 *  Created on: 2025年6月23日
 *      Author: zhangjie
 */

#ifndef _PRINTF_REDIRECT_H_
#define _PRINTF_REDIRECT_H_

#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_driver_spi.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_spi.h"


#define  PRINTF_OUTPUT_TYPE     1                    // printf 打印输出类型 0 使用串口   1 使用SPI


#if PRINTF_OUTPUT_TYPE == 0

// 逐飞科技开源库 串口输出模块 默认串口宏定义如下：
// DEBUG_UART_INDEX                                     // debug 调试串口
// BLE6A20_INDEX                                        // ble6a20 无线蓝牙模块
// BLUETOOTH_CH9141_INDEX                               // ch9141 蓝牙模块
// WIFI_UART_INDEX                                      // wifi 串口模块
// WIRELESS_UART_INDEX                                  // 无线串口模块

#define  PRINTF_USE_UART        (DEBUG_UART_INDEX)      // 默认 printf 使用 DEBUG_UART_INDEX 串口输出


// 如果 PRINTF_OUTPUT_TYPE 为 1，下方配置则为有效状态 printf 将会通过 SPI 模块输出
#else

// ====================================  注意  ====================================
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// 注意：printf使用wifi-spi模块输出字符串必须以"\n"作为单句输出的结尾，否则将导致无法打印或者内存溢出
// ====================================  注意  ====================================
#define  PRINTF_USE_SPI         (WIFI_SPI_INDEX)        // 默认 printf 使用 WIFI_SPI_INDEX 模块输出(该模式暂时仅支持 WIFI-SPI)

// 逐飞科技开源库 SPI输出模块 默认串口宏定义如下：
// WIFI_SPI_INDEX               // WIFI-SPI 模块

#endif


#endif

