/*
 * flash.h
 *
 *  Created on: 2025年7月12日
 *      Author: lenovo
 */

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

#define FLASH_SECTION_INDEX       (1)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (11)                                // 存储数据用的页码 倒数第一个页码


void flash_write_int8(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,int8 data);
void flash_read_int8(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,int8* data);
void flash_write_float(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,float data);
void flash_read_float(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,float* data);
void flash_write_float_all(void);

void flash_write_Element_Order(uint32 flash_section_index,uint32 flash_page_index);
void flash_read_Element_Order(uint32 flash_section_index,uint32 flash_page_index);

#endif /* CODE_FLASH_H_ */
