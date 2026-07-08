/*
 * flash.c
 *
 *  Created on: 2025年7月12日
 *      Author: lenovo
 */


#include "flash.h"

//20-6,20-7,11-2,11-3

void flash_write_int8(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,int8 data)
{
    flash_buffer_clear();
    flash_union_buffer[digit].int8_type  = data;
    if(flash_check(flash_section_index, flash_page_index))                      // 判断是否有数据
    {
        flash_erase_page(flash_section_index, flash_page_index);                // 擦除这一页
    }
    flash_write_page_from_buffer(flash_section_index, flash_page_index);
}

void flash_read_int8(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,int8* data)
{
    if(flash_check(flash_section_index, flash_page_index))
    {
        flash_read_page_to_buffer(flash_section_index, flash_page_index);
        *data=flash_union_buffer[digit].int8_type;
    }
}

void flash_write_Element_Order(uint32 flash_section_index,uint32 flash_page_index)
{
    flash_buffer_clear();
    flash_union_buffer[0].int8_type  = Element_Order[0];
    flash_union_buffer[1].int8_type  = Element_Order[1];
    flash_union_buffer[2].int8_type  = Element_Order[2];
    flash_union_buffer[3].int8_type  = Element_Order[3];
    if(flash_check(flash_section_index, flash_page_index))                      // 判断是否有数据
    {
        flash_erase_page(flash_section_index, flash_page_index);                // 擦除这一页
    }
    flash_write_page_from_buffer(flash_section_index, flash_page_index);
}

void flash_read_Element_Order(uint32 flash_section_index,uint32 flash_page_index)
{
    if(flash_check(flash_section_index, flash_page_index))
    {
        flash_read_page_to_buffer(flash_section_index, flash_page_index);
        Element_Order[0]=flash_union_buffer[0].int8_type;
        Element_Order[1]=flash_union_buffer[1].int8_type;
        Element_Order[2]=flash_union_buffer[2].int8_type;
        Element_Order[3]=flash_union_buffer[3].int8_type;
    }
}

void flash_write_float(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,float data)
{
    flash_buffer_clear();
    flash_union_buffer[digit].float_type  = data;
    if(flash_check(flash_section_index, flash_page_index))                      // 判断是否有数据
    {
        flash_erase_page(flash_section_index, flash_page_index);                // 擦除这一页
    }
    flash_write_page_from_buffer(flash_section_index, flash_page_index);
}

void flash_read_float(uint32 flash_section_index,uint32 flash_page_index,uint32 digit,float* data)
{
    if(flash_check(flash_section_index, flash_page_index))
    {
        flash_read_page_to_buffer(flash_section_index, flash_page_index);
        *data=flash_union_buffer[digit].float_type;
    }
}

void flash_write_float_all(void)
{
//    flash_write_int8(FLASH_SECTION_INDEX,10,0,speed);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,2,Balance_Velocity_KP);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,4,Balance_Velocity_KI);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,6,Balance_Velocity_KD);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,8,angle_KP);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,10,angle_KI);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,12,angle_KD);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,14,angle_speed_KP);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,16,angle_speed_KI);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,18,angle_speed_KD);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,20,steering_KP_small);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,22,steering_KP_large);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,24,steering_KD_small);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,26,steering_KD_large);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,28,steering_GKD);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,30,roll_angle_KP);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,32,roll_angle_KI);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,34,roll_angle_KD);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,36,bend_KP);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,38,bend_KI);
//    flash_write_float(FLASH_SECTION_INDEX,FLASH_PAGE_INDEX,40,bend_KD);
}


