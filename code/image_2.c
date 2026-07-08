/*
 * image_2.c
 *
 *  Created on: 2025年6月24日
 *      Author: zhangjie
 */
#include "image_2.h"
int Unilateral_Line_state;
int Unilateral_Line_left,Unilateral_Line_right;      //单边线的选择标志
int Line_wid_right[90]={                           //89~20
        31,31,32,32,32,33,33,33,34,
        34,34,34,34,34,35,35,35,37,39,
        40,41,41,41,42,42,42,43,43,43,
        44,45,45,46,46,47,47,48,48,49,
        49,50,51,52,52,53,54,55,57,59,
        59,60,60,61,61,62,62,63,63,64,
        64,64,65,66,68,69,70,71,72,73
};
int Line_wid_left[90]={
        152,152,151,151,150,150,149,149,149,149,
        148,148,148,148,148,147,147,146,146,145,
        144,143,142,142,141,140,140,140,139,139,
        138,137,136,135,134,134,134,134,134,134,
        133,133,132,132,131,130,129,128,125,124,
        124,124,124,122,122,122,120,120,119,119,
        118,118,117,117,115,115,114,113,112.111
};
void Unilateral_Line_Inspection()
{
    if(Unilateral_Line_state)
    {   //转弯的单线选择
        if(first_lastylose[0]>Unilateral_Line_state)
        {
            Unilateral_Line_left=1;
        }
        else if(first_lastylose[1]>Unilateral_Line_state)
        {
            Unilateral_Line_right=1;
        }
    }
}
//void line_correct_right()
//{
//    if(Unilateral_Line_left)
//    {
//        for(int x=first_lose[0];x<first_lastylose[0];x--)
//        {
//            Left_Line[x]
//        }
//    }
//    if(Unilateral_Line_right)
//    {
//        for(int x=first_lose[1];x<first_lastylose[1];x--)
//        {
//
//        }
//    }
//}
