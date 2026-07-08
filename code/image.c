#include "image.h"
#include "math.h"


//常用基本变量
const uint8 Image_Flags[][9][8];     //放在图上的数字标记
uint8 image_two_value[MT9V03X_H][MT9V03X_W];//二值化后的原数组
uint8 original_image[image_h][image_w];
uint8 bin_image[image_h][image_w],image_copy[image_h][image_w];

int8 Element_Order[4]={0,0,0,0};
int flag_count=0,flag=0;

volatile int Left_Line[MT9V03X_H]; //左边线数组
volatile int Right_Line[MT9V03X_H];//右边线数组
volatile int Mid_Line[MT9V03X_H];  //中线数组
volatile int Road_Wide[MT9V03X_H]; //赛宽数组
volatile int White_Column[MT9V03X_W];    //每列白列长度
volatile int Search_Stop_Line;     //搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
volatile int Boundry_Start_Left;   //左右边界起始点
volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
volatile int Left_Lost_Time;       //边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0

int Threshold=0;
//斑马线
volatile int Zebra_State=0;

//环岛
volatile int Island_State;     //环岛状态标志
volatile int Left_Island_Flag; //左右环岛标志
volatile int Right_Island_Flag;//左右环岛标志
volatile int island_delay_state;//没有完整进入环岛状态：弯接环岛未进1。

//十字
volatile int Cross_Flag=0;
volatile int Left_Down_Find=0; //十字使用，找到被置行数，没找到就是0
volatile int Left_Up_Find=0;   //四个拐点标志
volatile int Right_Down_Find=0;
volatile int Right_Up_Find=0;

//单边桥
volatile int unilateral_bridge_state=0;
//障碍
volatile int barrier_state=0;
//jump
volatile int jump_state=0;
volatile uint8 Img_Disappear_Flag=0; //图像消失标志位，1就说明丢图了

int streeing_switch=0;

int piont_x[10]={0},piont_y[10]={0};
const uint8 Standard_Road_Wide[MT9V03X_H]=//标准赛宽
{ 10, 12, 14, 16, 18, 20, 22, 24, 26, 28,
  30, 32, 34, 36, 38, 40, 42, 44, 46, 48,
  50, 52, 54, 56, 58, 60, 62, 64, 66, 68,
  70, 72, 74, 76, 78, 80, 82, 84, 86, 88,
  90, 92, 94, 96, 98,100,102,104,106,108,
 110,112,114,116,118,120,122,124,126,128,
 130,132,134,136,138,140,142,144,146,148};


//环岛变量
volatile int Island_State=0;     //环岛状态标志
volatile int Left_Island_Flag=0; //左右环岛标志
volatile int Right_Island_Flag=0;//左右环岛标志
volatile int Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列

int point_of_contact[2],symmetry_point[2];//圆弧切点坐标，[0]存y，第某行，{1}存x，第某列



int limit_a_b(int x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}


/*变量声明*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num     1   //1就是不压缩，2就是压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
{
  uint8 i,j;
 image_thereshold = otsuThreshold(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);     找到起点再赋值
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);     找到起点再赋值
            r_found = 1;

            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");

        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num image_h*3   //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            //printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[image_h];//左线数组
uint8 r_border[image_h];//右线数组
uint8 center_line[image_h];//中线数组
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //初始化
    for (i = 0;i<image_h;i++)
    {
        l_border[i] = border_min;
    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*3//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//黑

            }
        }
    }

}
void optimizeBinaryImage(uint8(*bin_image)[image_w]) {
    uint16_t i, j;
    uint32_t num;
    uint8_t temp_image[image_h][image_w];;
    memcpy(temp_image[0], image_two_value[0], MT9V03X_IMAGE_SIZE);

    // 处理非边缘像素
    for (i = 1; i < image_h - 1; i++) {
        for (j = 1; j < image_w - 1; j++) {
            // 统计8邻域像素值总和
            num = temp_image[i-1][j-1] + temp_image[i-1][j] + temp_image[i-1][j+1]
                + temp_image[i][j-1]              + temp_image[i][j+1]
                + temp_image[i+1][j-1] + temp_image[i+1][j] + temp_image[i+1][j+1];

            // 根据邻域状态调整当前像素
            if (num >= threshold_max && bin_image[i][j] == 0) {
                bin_image[i][j] = 255;  // 黑点转白点
            }
            if (num <= threshold_min && bin_image[i][j] == 255) {
                bin_image[i][j] = 0;    // 白点转黑点
            }
        }
    }
}               //史
/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w] 图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        image[image_h][i] = 0;
        image[image_h - 1][i] = 0;
        //image[image_h - 2][i] = 0;
        //image[image_h-1][i] = 0;

    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     int型 快速求绝对值
  @param     value
  @return    返回绝对值
  Sample     value=my_abs(value);
  @note      null
-------------------------------------------------------------------------------------------------------------------*/
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     普通大津求阈值
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    threshold   返回int类型的的阈值
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//普通大津
  @note      据说没有山威大津快，我感觉两个区别不大
-------------------------------------------------------------------------------------------------------------------*/
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height)   //大津算法，注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height;///4;
    int  threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    for (i = 0; i < height; i++/*=2*/)//统计灰度级中每个像素在整幅图像中的个数
    {
        for (j = 0; j <width; j++/*=2*/)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    for (i = 0; i < GrayScale; i++) //计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//遍历灰度级[0,255]
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//平方
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold>255)
        threshold=255;
    if(threshold<0)
        threshold=0;
  return threshold;
}
void histogram_equalization(uint8_t* image, int width, int height)
{
    // 计算总像素数
    int total_pixels = width * height;

    // 初始化灰度直方图和概率数组
    int pixel_count[256] = {0};
    float pixel_pro[256] = {0};

    // 统计每个灰度级的像素数量
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            uint8_t pixel = image[i * width + j];
            pixel_count[pixel]++;
        }
    }

    // 计算概率分布 (归一化直方图)
    for (int i = 0; i < 256; i++) {
        pixel_pro[i] = (float)pixel_count[i] / total_pixels;
    }

    // 计算累积分布函数 (CDF)
    float cdf[256] = {0};
    cdf[0] = pixel_pro[0];
    for (int i = 1; i < 256; i++) {
        cdf[i] = cdf[i-1] + pixel_pro[i];
    }

    // 计算映射表 (CDF归一化到0-255)
    uint8_t map[256];
    for (int i = 0; i < 256; i++) {
        map[i] = (uint8_t)(cdf[i] * 255 + 0.5);  // +0.5用于四舍五入
    }

    // 应用映射表到原始图像
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            uint8_t old_pixel = image[i * width + j];
            image[i * width + j] = map[old_pixel];
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @param     原始图像数组，高度，宽度，二值化阈值
  @return    二值化之后的图像数组
  Sample     Image_Binarization(Threshold);//图像二值化
  @note      二值化处理，0x00黑，0xff是白，已经用宏定义替换过了
-------------------------------------------------------------------------------------------------------------------*/
void Image_Binarization(int threshold)//图像二值化
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_two_value[i][j]=IMG_WHITE;//白
            else
                image_two_value[i][j]=IMG_BLACK;//黑
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column()//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=MT9V03X_W-20;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数

    for (i = 0; i <=MT9V03X_H-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W-1;
    }
    for(i=0;i<=MT9V03X_W-1;i++)
    {
        White_Column[i] = 0;
    }
    for(j=0;j<BOUNDARY_NUM;j++)         //给图传数据清零，便于显示图像
    {
        xy_x1_boundary[j]=0;
        xy_y1_boundary[j]=0;
        xy_x2_boundary[j]=0;
        xy_y2_boundary[j]=0;
        xy_x3_boundary[j]=0;
        xy_y3_boundary[j]=0;
    }
//环岛需要对最长白列范围进行限定
    //环岛3状态需要改变最长白列寻找范围
    if(Right_Island_Flag==1)//右环
    {
//        if(Island_State==3)
//        {
//            start_column=40;
//            end_column=MT9V03X_W-20;
//        }
        if(Island_State==2)
        {
            start_column=50;
            end_column=MT9V03X_W-20;
        }
    }
    else if(Left_Island_Flag==1)//左环
    {
//        if(Island_State==3)
//        {
//            start_column=20;
//            end_column=MT9V03X_W-40;
//        }
        if(Island_State==2)
        {
            start_column=20;
            end_column=MT9V03X_W-50;
        }
    }
//记住确定是哪一列用的是该点横坐标，确定是哪一行列用的是该点纵坐标。
    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)              //  外循环为列，内循环为行。先从最左列开始检测白点，确定白列开始，
    {                                                      //  从这一列的最低行开始依次向上检测，至黑点，即白列结束，并记下
        for (i = MT9V03X_H - 1; i >= 0; i--)               //  该白列的高度。
        {                                                  //
            if(image_two_value[i][j] == IMG_BLACK)         //
                break;                                     //
            else
                White_Column[j]++;
        }
    }

    //从左到右找左边最长白列
    Longest_White_Column_Left[0] = 0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
            //if(White_Column[i]==White_Column[i-1]==White_Column[i-2])break;             //*
        }
    }
    //从右到左找右边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
            //if(White_Column[i]==White_Column[i+1]==White_Column[i+2])break;             //*
        }
    }

    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
//    if(jump_state||unilateral_bridge_state){Search_Stop_Line=60;Longest_White_Column_Right[1]=90;Longest_White_Column_Left[1]=90;}
//    if(Search_Stop_Line<60){Unilateral_Line_state=Search_Stop_Line;}
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)          //*
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }

        }                                                                             //*
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//黑黑白认为到达左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;      //右边线线数组
         //*
    }
//    Search_Stop_Line = Longest_White_Column_Right[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的//*
//    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线               //*
//    {
//        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
//        {
//            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
//            {
//                right_border = j;
//                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
//                break;
//            }
//            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
//            {
//                right_border = j;
//                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
//                break;
//            }
//        }
//        Right_Line[i] = right_border;      //右边线线数组
//    }
    for (i = MT9V03X_H - 1; i >= MT9V03X_H-Search_Stop_Line; i--)//赛道数据初步分析
    {
        if (Left_Lost_Flag[i]  == 1)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }

    find_first_lose();
    //环岛3状态改变边界，看情况而定，我认为理论上的最优情况是不需要这些处理的
//    if(Island_State==3||Island_State==4)
//    {
//        if(Right_Island_Flag==1)//右环
//        {
//            for (i = MT9V03X_H - 1; i >= 0; i--)//右边直接写在边上
//            {
//                Right_Line[i]=MT9V03X_W-1;
//            }
//        }
//        else if(Left_Island_Flag==1)//左环
//        {
//            for (i = MT9V03X_H - 1; i >= 0; i--)//左边直接写在边上
//            {
//                if(Island_State==3)Right_Line[i]*=((MT9V03X_H-2*i)/MT9V03X_H);
//                Left_Line[i]=0;      //左边线线数组
//            }
//        }
//    }
    //debug使用，屏幕显示相关参数
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);//【0】是白列长度
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);//【1】是下标，第j列)
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);//【0】是白列长度
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);//【1】是下标，第j列)
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     边界显示，用于图传，显示到屏幕上，
  @param     null
  @return    null
  Sample     直接调用
  @note      显示左中右边界，中线，
                                           正常情况下不要用，因为直接在原图上写入了边界信息
                                           会对元素判断造成干扰的，调试时候调用
-------------------------------------------------------------------------------------------------------------------*/
void Show_Boundry(void)
{
    int16 i,j=0;
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line;i--,j++)//从最底下往上扫描
    {
        image_two_value[i][Left_Line[i]]=IMG_BLACK;
        image_two_value[i][(Left_Line[i]+Right_Line[i])>>1]=IMG_BLACK;
        image_two_value[i][Right_Line[i]]=IMG_BLACK;
//        if(Left_Line[i]!=Right_Line[i])
//        {
//            xy_x1_boundary[j]=Left_Line[i];
//            xy_y1_boundary[j]=i;
//            xy_x2_boundary[j]=Right_Line[i];
//            xy_y2_boundary[j]=i;
//            xy_x3_boundary[j]=(Left_Line[i]+Right_Line[i])>>1;
//            xy_y3_boundary[j]=i;
//        }
    }


}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Left_Line[i]=hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的，不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//右补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
    else if(y2<=0)
         y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Right_Line[i]=hx;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start,int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=8&&
              (Left_Line[i]-Left_Line[i-3])>=15&&
              (Left_Line[i]-Left_Line[i-4])>=15)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-8&&
              (Right_Line[i]-Right_Line[i-3])<=-15&&
              (Right_Line[i]-Right_Line[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start,int end)           //Find_Up_Point( MT9V03X_H-1, 0 );
{
    int i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&&
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            Left_Up_Find=i;//获取行数即可
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            Right_Up_Find=i;//获取行数即可
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
         Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }

    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            if(Left_Line[i]>=MT9V03X_W-1)
            {
                Left_Line[i]=MT9V03X_W-1;
            }
            else if(Left_Line[i]<=0)
            {
                Left_Line[i]=0;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            if(Right_Line[i]>=MT9V03X_W-1)
            {
                Right_Line[i]=MT9V03X_W-1;
            }
            else if(Right_Line[i]<=0)
            {
                Right_Line[i]=0;
            }
        }
    }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字检测
  @param     null
  @return    null
  Sample     Cross_Detect(void);
  @note      利用四个拐点判别函数，查找四个角点，根据找到拐点的个数决定是否补线
-------------------------------------------------------------------------------------------------------------------*/
void Cross_Detect()
{
    int down_search_start=0;//下点搜索开始行
    Cross_Flag=0;
    if(Island_State==0&&!jump_state&&podao_state!=1)//与环岛互斥开
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point( MT9V03X_H-1, 0 );
            if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就找到十字了
        {
            Cross_Flag=1;//对应标志位，便于各元素互斥掉
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(MT9V03X_H-5,down_search_start+2);//在上拐点下2行作为下点的截止行
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的都是斜率补线
            {//三个点                                     //01
                Lengthen_Left_Boundry(Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//三个点                                     //10
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//就俩上点                                   //00
                Lengthen_Left_Boundry (Left_Up_Find-1,MT9V03X_H-1);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
        }
        else
        {
            Cross_Flag=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     画线
  @param     输入起始点，终点坐标，补一条宽度为2的黑线
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    画一个大×
  @note     补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
int buxian_flag=0;
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i,x,y;
    int start=0,end=0;
    buxian_flag=1;
    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=MT9V03X_W-1)
        endX=MT9V03X_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=1)
                i=1;
            image_two_value[i][startX]=IMG_BLACK;
            image_two_value[i-1][startX]=IMG_BLACK;

        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=1;
            image_two_value[startY][i]=IMG_BLACK;
            image_two_value[startY-1][i]=IMG_BLACK;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=MT9V03X_W-1)
                x=MT9V03X_W-1;
            else if (x<=1)
                x=1;
            image_two_value[i][x] = IMG_BLACK;
            image_two_value[i][x-1] = IMG_BLACK;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {

            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=MT9V03X_H-1)
                y=MT9V03X_H-1;
            else if (y<=0)
                y=0;
            image_two_value[y][i] = IMG_BLACK;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     Continuity_Change_Left(int start,int end);
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end)//连续性阈值设置为5
{
    int i=0;
    int t=0;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
       if(abs(Left_Line[i]-Left_Line[i-2])>=5)//连续判断阈值是5,可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-3])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(left_up_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=4&&
              (Left_Line[i]-Left_Line[i+3])>=8&&
              (Left_Line[i]-Left_Line[i+4])>=8)
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-5&&
              (Right_Line[i]-Right_Line[i-3])<=-10&&
              (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(right_up_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
       return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护，在判断第i个点时
       start=MT9V03X_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
      return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>Left_Line[i+5]&&Left_Line[i]>Left_Line[i-5]&&
        Left_Line[i]>Left_Line[i+4]&&Left_Line[i]>Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1]&&
        (Left_Line[i]-Left_Line[i+1])<8)
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i]<Right_Line[i+5]&&Right_Line[i]<Right_Line[i-5]&&
        Right_Line[i]<Right_Line[i+4]&&Right_Line[i]<Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1]/*&&abs(Right_Line[i]-Right_Line[i+1])<-8*/)
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     元素标志位显示
  @param     二值化图片数组
  @return    null
  Sample     Image_Flag_Show(MT9V03X_W,image_two_value,Island_State);
  @note      要在图片显示前使用，传入标志位可更改
-------------------------------------------------------------------------------------------------------------------*/
void Image_Flag_Show(uint8 MT9V03XW,uint8(*InImg)[MT9V03XW],uint8 image_flag)
{
    for(uint8 H=1;H<8;H++)
    {
        for(uint8 W=1;W<7;W++)
        {
            switch (image_flag)
            {
               // case 0:     *(*(InImg+H)+W)=Image_Flags[0][H][W];    break;//环岛0~8
                case 1:     *(*(InImg+H)+W)=Image_Flags[1][H][W];    break;
                case 2:     *(*(InImg+H)+W)=Image_Flags[2][H][W];    break;
                case 3:     *(*(InImg+H)+W)=Image_Flags[3][H][W];    break;
                case 4:     *(*(InImg+H)+W)=Image_Flags[4][H][W];    break;
                case 5:     *(*(InImg+H)+W)=Image_Flags[5][H][W];    break;
                case 6:     *(*(InImg+H)+W)=Image_Flags[6][H][W];    break;
                case 7:     *(*(InImg+H)+W)=Image_Flags[7][H][W];    break;
                case 8:     *(*(InImg+H)+W)=Image_Flags[8][H][W];    break;
                case 9:     *(*(InImg+H)+W)=Image_Flags[9][H][W];    break;
                default:    break;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)//--操作，start需要大
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Left_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        buxian_flag=1;
        if(Left_Line[i]>=MT9V03X_W-1)
        {
            Left_Line[i]=MT9V03X_W-1;

        }
        else if(Left_Line[i]<=0)
        {
            Left_Line[i]=0;
        }

    }


}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null    直接补边线
  Sample     K_Add_Boundry_Right(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Right_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Right_Line[i]>=MT9V03X_W-1)
        {
            Right_Line[i]=MT9V03X_W-1;
        }
        else if(Right_Line[i]<=0)
        {
            Right_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     根据斜率划线
  @param     输入斜率，定点，画一条黑线
  @return    null
  Sample     K_Draw_Line(k, 20,MT9V03X_H-1 ,0)
  @note      补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void K_Draw_Line(float k, int startX, int startY,int endY)
{
    int endX=0;

    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1

    Draw_Line(startX,startY,endX,endY);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取平均赛宽
  @param     int start_line,int end_line，起始行，中止行
  @return    这几行赛宽平均值
  Sample     road_wide=Get_Road_Wide(68,69);
  @note      ++运算，向下寻找，算出平均赛宽
-------------------------------------------------------------------------------------------------------------------*/
int Get_Road_Wide(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    int i=0,t=0;
    int road_wide=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    for(i=start_line;i<=end_line;i++)
    {
        road_wide+=Right_Line[i]-Left_Line[i];
    }
    road_wide=road_wide/(end_line-start_line+1);//平均赛宽
    return road_wide;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取左赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Left_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Left_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Left_Line[start_line]-(float)Left_Line[end_line])/(end_line-start_line+1));
    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取右赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Right_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Right_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Right_Line[start_line]-(float)Right_Line[end_line])/(end_line-start_line+1));
    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左角点判断
  @param     int start_line,int end_line，起始行，中止行
  @return
  Sample     k=Get_Right_K(68,69);
  @note
-------------------------------------------------------------------------------------------------------------------*/
void zebra_get(void)
{
    if(Zebra_State)if(aclture_distance>=0)speed=-40;
    if(Zebra_State)
    {
        return;
    }
    int num=0,net=0;
    if(hightest<30)
    {
        for(int y=55;y<75;y++)
        {
            net=0;
            for(int x=l_border[y]+2;x<r_border[y]-2;x++)
            {
                if(bin_image[y][x]==white_pixel&&bin_image[y][x+1]==black_pixel)
                {
                    net++;

                    if(net>6)
                    {
                        num++;
                    }
                }
            }
        }
        if(num>=6&&Zebra_State==0)
        {
            Zebra_State=1;

        }
    }


}
void zebra_get_fube(void)
{
    if(Zebra_State)
    {
        return;
    }
    int num=0,net=0;
//    if(hightest<30)
//    {
        for(int y=50;y<80;y++)
        {
            net=0;
            for(int x=Left_Line[y]+2;x<Right_Line[y]-2;x++)
            {
                if(image_two_value[y][x]==white_pixel&&image_two_value[y][x+1]==black_pixel)
                {
                    net++;

                    if(net>6)
                    {
                        num++;
                        //ips[5]=num;
                    }
                }
            }
        }
        if(num>=6&&Zebra_State==0)
        {
            Zebra_State=1;
            speed=-40;
        }
//    }

}
void find_first_lose()        //求丢点的起始和终末
{
    uint8 i=0,l=0,r=0,x=0,y=0;
    first_lose[0]=0,first_lose[1]=0,first_lastylose[0]=0,first_lastylose[1]=0;
    for (i = MT9V03X_H - 3; i >= MT9V03X_H-Search_Stop_Line+2; i--)
    {
        if (l==0){if(Left_Lost_Flag[i]  == 1){first_lose[0]=i;l=1;}}
        else if (l==1&&x==0)
        {
            if(Left_Lost_Flag[i]  == 1&&Left_Lost_Flag[i-2]  == 0){first_lastylose[0]=i;x=1;}
            if(i==MT9V03X_H-Search_Stop_Line+2){first_lastylose[0]=i;x=1;}
        }
        if (r==0){if(Right_Lost_Flag[i] == 1){first_lose[1]=i;r=1;}}
        else if (r==1&&y==0)
        {
            if(Right_Lost_Flag[i] == 1&&Right_Lost_Flag[i-2]  == 0){first_lastylose[1]=i;y=1;}
            if(i==MT9V03X_H-Search_Stop_Line+2){first_lastylose[1]=i;y=1;}
        }
        if(x==1&&y==1){break;}
    }
}
int right_point_tangent(int start,int end) //从下至上,求取右圆环切点
{
    uint8 i=0;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
       if(abs(Right_Line[i]-Right_Line[i-2])>=5)//连续判断阈值是5,可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}
int left_up_angle(int start,int end) //从下至上,求左上拐点         //            /             如图
{                                                            //          /
    int i=0;                                                 //    ------
    int t;                                                    //   |
    int point=0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)大部分都丢线，没必要判断了
//       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
       if((Left_Line[i]-Left_Line[i+1])>=15&&Left_Line[i]-Left_Line[i-4]<=(-1)&&
               Left_Line[i]-Left_Line[i-6]<(-2)&&Left_Line[i]-Left_Line[i-8]<(-3)
               &&Left_Line[i]-Left_Line[i-10]<(-4)
         )//
       {
            point=i;
            break;
       }
    }
    return point;
}
int right_up_angle(int start,int end) //从下至上,求右上拐点     //         \                   如图
{                                                         //          \
                                                          //           ------
    int t;
    int point=0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)大部分都丢线，没必要判断了
//       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if((Right_Line[i]-Right_Line[i+1])<=(-15)&&Right_Line[i]-Right_Line[i-4]>=1
               &&Right_Line[i]-Right_Line[i-6]>2&&Right_Line[i]-Right_Line[i-8]>3
               &&Right_Line[i]-Right_Line[i-10]>4
               )//
       {
            point=i;
            break;
       }
    }
    return point;
}

/*    -------          右下角点
 *   \
 *    \
 *     \
 */
int right_down_angle(int start,int end)
{
    int t;
    int point=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if((Right_Line[i-1]-Right_Line[i])>=20&&Right_Line[i+4]>Right_Line[i])//
       {
            point=i;
            break;
       }
    }
    return point;
}

/*    -------             左下角点
 *          /
 *         /
 *        /
 */
int left_down_angle(int start,int end)
{
    int t;
    int point=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if((Left_Line[i]-Left_Line[i-1])>=20&&Left_Line[i+1]<Left_Line[i])//
       {
            point=i;
            break;
       }
    }
    return point;
}
void stop_car()
{

}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     环岛检测
  @param     null
  @return    null
  Sample     Island_Detect(void);
  @note      利用四个拐点判别函数，单调性改变函，连续性数撕裂点，分为8步
-------------------------------------------------------------------------------------------------------------------*/
float k1=0;
int island_road_state;
void Island_Detect()
{
//    static float k=0;//3和5状态的k
//    static int island_state_5_down[2]={0};//状态5时即将离开环岛，左右边界边最低点，[0]存y，第某行，{1}存x，第某列
//    static int island_state_3_up[2]={0};//状态3时即将进入环岛用，左右上面角点[0]存y，第某行，{1}存x，第某列
    static int left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    static int right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
    int monotonicity_change_left_flag=0;//不转折是0
    int monotonicity_change_right_flag=0;//不转折是0
    int continuity_change_right_flag=0; //连续是0
    int continuity_change_left_flag=0;  //连续是0
    point_of_contact[0]=0,point_of_contact[1]=0;
    symmetry_point[0]=0,symmetry_point[1]=0;//圆弧切点坐标，[0]存y，第某行，{1}存x，第某列
    for(uint8 i=0;i<=15;i++){ips[i]=0;}

    int down=0;
    //以下是常规判断法
    continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);//连续性判断
    continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);
    monotonicity_change_right_flag=Monotonicity_Change_Right(MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=Monotonicity_Change_Left(MT9V03X_H-1-10,10);
    ips[6]=monotonicity_change_right_flag;
    ips[7]=monotonicity_change_left_flag;
    ips[1]=left_up_angle(85,10);
    ips[2]=left_down_angle(85,10);
    ips[4]=right_down_angle(85,10);
    ips[3]=right_up_angle(85,10);
    if(Left_Island_Flag==0&&Right_Island_Flag==0&&Cross_Flag==0&&Island_State==0&&unilateral_bridge_state==0&&jump_state==0)
    {
//        continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);//连续性判断
//
//        continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);

        if(monotonicity_change_left_flag==0&&               //右环判断
           continuity_change_left_flag==0&&//右环岛左边是连续的
           continuity_change_right_flag!=1&&//右边是不连续的
           continuity_change_right_flag!=0&&
           Right_Lost_Time>=10&&           //右丢线多
           Both_Lost_Time<=10)
        {
            right_down_guai[0]=right_down_angle(MT9V03X_H-5,10);//右下点
            if(right_down_guai[0]>=35/*||(30<first_lastylose[1]&&first_lastylose[1]<50&&monotonicity_change_right_flag<first_lastylose[1]
                    &&Right_Line[monotonicity_change_right_flag]<150)*/)//条件1很松，在这里加判拐点，位置不对，则是误判，跳出
            {
                Island_State=1;
                Right_Island_Flag=1;
            }
//            else if(right_up_angle(60,10)>13&&monotonicity_change_right_flag>45) //防止弯接圆环看不到下角点
//            {
//                Island_State=2;
//                Right_Island_Flag=1;
//                island_delay_state=1;
//            }
            else
            {
                Island_State=0;
                Right_Island_Flag=0;
            }
        }


        if(             //左环判断
           continuity_change_left_flag!=0&&
           continuity_change_right_flag==0&&
           Left_Lost_Time>=10&&           //右丢线多
           Both_Lost_Time<=10
           )
        {
            left_down_guai[0]=left_down_angle(MT9V03X_H-5,10);//找左下角点
            if(left_down_guai[0]>=35/*||(30<first_lastylose[0]&&first_lastylose[0]<50&&monotonicity_change_left_flag<first_lastylose[0]
                &&Left_Line[monotonicity_change_right_flag]>30)*/)//条件1很松，在这里判断拐点，位置不对，则是误判，跳出
            {
                Island_State=1;
                Left_Island_Flag=1;
            }
//            else if(left_up_angle(60,5)>30) //防止弯接圆环看不到下角点
//            {
//                Island_State=2;
//                Left_Island_Flag=1;
//                island_delay_state=1;
//            }
            else//误判，归零
            {
                Island_State=0;
                Left_Island_Flag=0;

            }
        }
    }


    if(Left_Island_Flag==1)
    {
///////////////////////////////////////////////////////////////////////////////////////////////状态1
        if(Island_State==1)//过下口
        {
            monotonicity_change_line[0]=left_down_angle(85,10);//寻找下角点
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
            if(monotonicity_change_line[0]>=60)island_road_state=1;
            if(monotonicity_change_line[0])
            {
                down=1;
                point_of_contact[0]=Monotonicity_Change_Left(monotonicity_change_line[0]-10,10);//找到下角点后再寻找凸点
                point_of_contact[1]=Left_Line[point_of_contact[0]];
                //if(point_of_contact[1]<40){point_of_contact[1]=50;}
            }//下角点与圆凸点同时找到
            else if(!monotonicity_change_line[0])
            {
                down=0;
                point_of_contact[0]=Monotonicity_Change_Left(50,10);//寻找凸点
                if(point_of_contact[0]>100)point_of_contact[0]=100;
                point_of_contact[1]=Left_Line[point_of_contact[0]];
                if(point_of_contact[1]<40){point_of_contact[1]=40;}
            }//只找到圆凸点
            if(left_up_angle(85,11)>15&&aclture_distance>=250)
            {
                Island_State=2;aclture_distance=0;island_road_state=0;
            }
            //if(first_lose[0]<7){Island_State=0;Left_Island_Flag=0;}
            //else if(Left_Lost_Time<5&&Right_Lost_Time<5){Island_State=0;Left_Island_Flag=0;}
                                                                                        //point_of_contact[1],point_of_contact[0]
            if(down)Left_Add_Line(monotonicity_change_line[1],monotonicity_change_line[0],80,5);
            else if(!down)Left_Add_Line(20,image_h-5,point_of_contact[1],point_of_contact[0]);
//            else Left_Add_Line(20,image_h-5,Left_Line[1],1);
        }
        else if(Island_State==2)//进上口
        {

            angle_Integral_start_flag=1;
            monotonicity_change_line[0]=Continuity_Change_Left(first_lose[0],5)-6;
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
            if(abs(yaw)>=30&&Right_Lost_Time<15)//看不到直道Continuity_Change_Right(65,5)>50&&&&left_up_angle(60,10)<10&&Right_Lost_Time==0
            {
//                encoder_start_flag=1;
                Island_State=3;//最长白列寻找范围也要改，见camera.c
            }
            else if(abs(yaw)>50)
            {
                Island_State=3;
            }
//            if(monotonicity_change_line[1]<60)Right_Add_Line(150,MT9V03X_H-1,monotonicity_change_line[1]+10,monotonicity_change_line[0]);

            if(first_lastylose[0] >= 5 &&Left_Line[first_lastylose[0]-5]>10)
            {    Right_Add_Line(160,MT9V03X_H-1,Left_Line[first_lastylose[0]-5],first_lastylose[0]);}
            else //if(Left_Line[first_lastylose[0]-1]<170)
            {    Right_Add_Line(160,MT9V03X_H-1,Right_Line[first_lastylose[1]-5],first_lastylose[1]);}
        }
        else if(Island_State==3)//环内
        {

            monotonicity_change_line[0]=Monotonicity_Change_Right(80,10);//寻找单调性改变点
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            //ips[5]=monotonicity_change_line[0];
            if(monotonicity_change_line[0]>30&&abs(yaw)>200)//||(Right_Lost_Time>20&&Left_Lost_Time>20)
            {
                Island_State=4;
            }
        }
        else if(Island_State==4)//出下口
        {
            monotonicity_change_line[0]=Monotonicity_Change_Right(89,10);//寻找单调性改变点
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            if((abs(yaw)>250&&monotonicity_change_line[0]==0&&first_lastylose[1]>60)||left_up_angle(80,10)>20)//&&left_up_angle(80,10)>10
            {
                Island_State=5;
            }
            else if(abs(yaw)>350)Island_State=5;
//            if(monotonicity_change_line[0])
//            {
//                Right_Add_Line(monotonicity_change_line[1],monotonicity_change_line[0],1,first_lastylose[0]+4);
//            }
//            else //if(first_lose[1]>=60)//右边先丢线
//            {
//             if(first_lastylose[1]<80)Right_Add_Line(image_w-15,image_h-2,20,first_lastylose[0]-5);
//             if(left_up_angle(80,10)>10)Left_Add_Line(8,image_h-2,Left_Line[left_up_angle(80,10)],left_up_angle(80,10));
             if(monotonicity_change_line[0])Right_Add_Line(Right_Line[monotonicity_change_line[0]],monotonicity_change_line[0],20,first_lastylose[0]-5);
             if(first_lastylose[1]>20)   Right_Add_Line(179,image_h-2,100,50);
//            }
        }
        else if(Island_State==5)//过上口
        {
            if(/*first_lose[0]<60&&first_lose[1]<60&&*/left_up_angle(80,10)>=35)
            {
                Island_State=6;
            }
            if(left_up_angle(80,10)>10)Left_Add_Line(8,image_h-2,Left_Line[left_up_angle(80,10)],left_up_angle(80,10));
            else if(first_lastylose[0]>6) Left_Add_Line(8,image_h-2,Left_Line[first_lastylose[0]-5],first_lastylose[0]-5);
            if(first_lose[1]>70)Right_Add_Line(178,image_h-2,Right_Line[50],50);
        }
        else if(Island_State==6)//过上口
        {
            if(left_up_angle(80,10)==0||left_up_angle(80,10)>=70)
            {
                Island_State=0;
                Left_Island_Flag=0;
                angle_Integral_start_flag=0;
                yaw=0;
            }
            if(left_up_angle(80,10)>10)Left_Add_Line(8,image_h-2,Left_Line[left_up_angle(80,10)],left_up_angle(80,10));
            if(first_lose[1]>70)Right_Add_Line(178,image_h-2,Right_Line[50],50);
        }
    }
//
    if(Right_Island_Flag==1)
    {
   ///////////////////////////////////////////////////////////////////////////////////////////////状态1
        if(Island_State==1)//过下口
        {
            monotonicity_change_line[0]=right_down_angle(85,10);//寻找下角点
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            if(monotonicity_change_line[0]>=60)island_road_state=1;
            if(monotonicity_change_line[0])
            {
                down=1;
                point_of_contact[0]=Monotonicity_Change_Right(monotonicity_change_line[0]-10,10);//找到下角点后再寻找凸点
                point_of_contact[1]=Right_Line[point_of_contact[0]];
            }
            else if(!monotonicity_change_line[0])
            {
                down=0;
                point_of_contact[0]=Monotonicity_Change_Right(50,10);//寻找凸点
//                if(point_of_contact[0]<80)point_of_contact[0]=80;
                point_of_contact[1]=Right_Line[point_of_contact[0]];
                if(point_of_contact[1]>140){point_of_contact[1]=140;}
            }//只找到圆凸点
            if(right_up_angle(85,11)>15&&aclture_distance>=250)
            {
                Island_State=2;
                aclture_distance=0;island_road_state=0;
            }
            //if(first_lose[1]<7){Island_State=0;Right_Island_Flag=0;}
            if(down)Right_Add_Line(monotonicity_change_line[1],monotonicity_change_line[0],80,5);
            else if(!down)
            {
                Right_Add_Line(160,image_h-5,point_of_contact[1],point_of_contact[0]);
//                Right_Add_Line(160,image_h-5,100,10);
            }

        }
    else if(Island_State==2)//进上口
    {
        angle_Integral_start_flag=1;
        monotonicity_change_line[0]=Continuity_Change_Right(first_lose[1],5)-6;
        monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
        //ips[9]=Continuity_Change_Left(65,5);
        if(abs(yaw)>=30&&Left_Lost_Time<15)//看不到直道Continuity_Change_Left(65,5)>40&&&&right_up_angle(60,10)<10
        {
//                encoder_start_flag=1;
            Island_State=3;//最长白列寻找范围也要改，见camera.c
        }
        else if(abs(yaw)>50)
        {
            Island_State=3;
        }
//        if(monotonicity_change_line[1]>120)Left_Add_Line(30,MT9V03X_H-1,monotonicity_change_line[1]+10,monotonicity_change_line[0]);
        if(first_lastylose[1] >= 5 && Right_Line[first_lastylose[1]-5]<170)
        {Left_Add_Line(30,MT9V03X_H-1,Right_Line[first_lastylose[1]-5],first_lastylose[1]);}
        else
        {Left_Add_Line(30,MT9V03X_H-1,Left_Line[first_lastylose[0]-5],first_lastylose[0]);}
    }
    else if(Island_State==3)//环内
    {
//            encoder_finish_flag=0;
        monotonicity_change_line[0]=Monotonicity_Change_Left(80,30);//寻找单调性改变点
        monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
        if(monotonicity_change_line[0]>30&&abs(yaw)>200)
        {
                Island_State=4;
        }
    }
    else if(Island_State==4)//出下口
    {
        monotonicity_change_line[0]=Monotonicity_Change_Left(89,10);//寻找单调性改变点
        monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];                       //right_up_angle(69,10)>10
        if((monotonicity_change_line[0]==0&&abs(yaw)>200&&first_lastylose[0]>60)||right_up_angle(80,10)>20)
        {
            Island_State=5;
        }
        else if(abs(yaw)>350)Island_State=5;

//        if(first_lastylose[0]<80)Left_Add_Line(Left_Line[monotonicity_change_line[0]],monotonicity_change_line[0],160,first_lastylose[1]-5);
        if(monotonicity_change_line[0])Left_Add_Line(Left_Line[monotonicity_change_line[0]],monotonicity_change_line[0],160,first_lastylose[1]-5);
        if(first_lastylose[0]>20)  Left_Add_Line(1,image_h-2,60,50);//Left_Add_Line(1,image_h-2,Left_Line[50],50);
//        }

    }
    else if(Island_State==5)//过上口
    {
//        ips[19]=Monotonicity_Change_Right(first_lose[1],10);
        if(/*first_lose[0]<60&&first_lose[1]<60&&*/right_up_angle(80,10)>=35)//环岛结束 Right_Lost_Time<5&&Left_Lost_Time<5&&Monotonicity_Change_Right(first_lose[1],10)>70
        {
            Island_State=6;
//            Right_Island_Flag=0;
//            angle_Integral_start_flag=0;
//            yaw=0;
        }
        if(right_up_angle(80,10)>10)Right_Add_Line(172,image_h-2,Right_Line[right_up_angle(80,10)],right_up_angle(80,10));

        if(first_lastylose[1]>6) Right_Add_Line(172,image_h-2,Right_Line[first_lastylose[1]-5],first_lastylose[1]-5);
        else if(first_lose[0]>70)Left_Add_Line(2,88,Left_Line[50],50);

    }
    else if(Island_State==6)//tuichu
     {
         if(right_up_angle(80,10)==0||right_up_angle(80,10)>=70)//环岛结束 Right_Lost_Time<5&&Left_Lost_Time<5&&Monotonicity_Change_Right(first_lose[1],10)>70
         {
             Island_State=0;
             Right_Island_Flag=0;
             angle_Integral_start_flag=0;
             yaw=0;
         }
         if(first_lose[0]>70)Left_Add_Line(2,88,Left_Line[50],50);
         if(right_up_angle(80,10)>10)Right_Add_Line(image_w-20,image_h-2,Right_Line[right_up_angle(80,10)],right_up_angle(80,10));
     }
  }

}
/**************************************************************单边桥***************************************************************/
int left_bridge,right_bridge;
int unilateral_bridge_or_barrier(int start,int end)//识别单边桥或避障
{
    int t;
    int point=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(end<=MT9V03X_H-Search_Stop_Line-1)
       end=MT9V03X_H-Search_Stop_Line-1;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if((Right_Line[i+3]-Right_Line[i])>=15)//&&abs(Right_Line[i]-90)<10
       {
            point=i;
            right_bridge=1;
            break;
       }
       else if((Left_Line[i]-Left_Line[i+3])>=15)//&&abs(Left_Line[i]-90)<10
       {
            point=i;
            left_bridge=1;
            break;
       }
//       else if(abs(Left_Line[i]-90)<10&&i>=85)
//       {
//
//       }
//       if(Right_Line[i]<150&&Right_Line[i-1]>150&&Right_Line[i-2]>150)
//       {
//           point=i;
//           break;
//       }
//       else if(Left_Line[i]>10&&Left_Line[i-1]<10&&Left_Line[i-2]<10)
//       {
//           point=i;
//           break;
//       }
//       else if((Right_Line[i]-Right_Line[i-1])>30&&abs(Right_Line[i-1]-Right_Line[i-2])<5)
//       {
//           point=i;
//           break;
//       }
//       else if((Left_Line[i]-Left_Line[i-1])<(-30)&&abs(Left_Line[i-1]-Left_Line[i-2])<5)
//       {
//           point=i;
//           break;
//       }
    }
    return point;
}
int define_unilateral_bridge(int start,int end)//确认单边桥，在识别到第一单边桥或障碍后使用
{
    int t;
    int point=0;
    right_bridge_up=0,right_bridge_down=0,left_bridge_up=0,left_bridge_down=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
           if(((Right_Line[i+3]-Right_Line[i])>=17||(Right_Line[i]!=(MT9V03X_H-1)&&Right_Line[i-1]==(MT9V03X_H-1)))
                   &&left_bridge)//&&abs(Right_Line[i]-90)<10
           {
                point=i;
                break;
           }
           else if(((Left_Line[i]-Left_Line[i+3])>=17||(Left_Line[i]!=0&&Left_Line[i-1]==0))&&right_bridge)//&&abs(Left_Line[i]-90)<10
           {
                point=i;
                break;
           }

    }
    return point;
}
int cancle_unilateral_bridge(int start,int end)//退出单边桥
{
    int t;
    int point=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if(Right_Line[i]-Left_Line[i]<50)//找第一被遮挡列
       {
            point=i;
            break;
       }
    }
    return point;
}
int left_one_line_up,left_one_line_down,right_one_line_up,right_one_line_down;
void unilateral_bridge_one_line(void)
{
    int i=0;left_one_line_up=0;left_one_line_down=0;right_one_line_up=0;right_one_line_down=0;
    for(i=80;i>90-Search_Stop_Line+3;i--)
    {
        if((Right_Line[i]-Right_Line[i-2])>=6&&!right_one_line_down)//
         {
            right_one_line_down=i;
         }
        else if((Right_Line[i-2]-Right_Line[i])>=6&&!right_one_line_up)//
         {
            right_one_line_up=i;
         }
         if((Left_Line[i]-Left_Line[i+2])>=6&&!left_one_line_down)
         {
            left_one_line_down=i;
         }
         else if((Left_Line[i+2]-Left_Line[i])>=6&&!left_one_line_up)
         {
            left_one_line_up=i;
         }
    }
}

void unilateral_bridge_and_avoide_barriers()
{
    int first_point=0,second_point=0;
    //define_unilateral_bridge(87,10);
//    first_point=(right_bridge_down > left_bridge_down) ?
//                 right_bridge_down : left_bridge_down;
    first_point=unilateral_bridge_or_barrier(85,10);
//    first_point=ips[11];
//    second_point=ips[12];
    if(first_point>10)second_point=unilateral_bridge_or_barrier(first_point-7,5);
    ips[8]=first_point;
    ips[9]=second_point;
    if(!Left_Island_Flag&&!Right_Island_Flag&&!Cross_Flag&&!Island_State&&!unilateral_bridge_state&&!jump_state/*&&abs(first_point-second_point)>23*/
            &&Search_Stop_Line<85&&first_lose[0]<5&&first_lose[1]<5&&first_point>20&&first_point>(90-Search_Stop_Line+3)
            &&!barrier_state&&Zebra_State==0)//&&second_point>8
    {
            unilateral_bridge_state=1;
//            angle_Integral_start_flag=1;

    }
    else if(unilateral_bridge_state/*&&!bridge_cancle*/)
    {
                                        //具体实现代码
            bridge_cancle=1;

        //补线
            if(right_one_line_down>left_one_line_down&&right_one_line_down>30&&left_one_line_down)
                 {
                     Right_Add_Line(Right_Line[right_one_line_down+2],right_one_line_down+2,Right_Line[left_one_line_down-5],left_one_line_down-5);
                 }
                 else if(right_one_line_up>30)
                 {
                     int j=0;
                     for(j=0;(Right_Line[right_one_line_up-j]-Right_Line[right_one_line_up-j-1]<=3)&&
                     (Right_Line[right_one_line_up-j-2]-Right_Line[right_one_line_up-j-3]<=3);j++)
                     if(Right_Line[right_one_line_up-j]<150)Right_Add_Line(155,80,Right_Line[right_one_line_up-j],right_one_line_up-j);
                 }
      //           else if(right_one_line_down>30&&left_one_line_down<10)
      //           {
      //               Right_Add_Line(Right_Line[right_one_line_down+2],right_one_line_down+2,Right_Line[20],20);
      //           }

                 if(left_one_line_down>right_one_line_down&&right_one_line_down&&left_one_line_down>30)
                 {
                     Left_Add_Line(Left_Line[left_one_line_down+2],left_one_line_down+2,Left_Line[right_one_line_down-5],right_one_line_down-5);
                 }
                 else if(left_one_line_up>30)
                 {
                     int j=0;
      //               for(j=0;(Left_Line[left_one_line_up-j]-Left_Line[left_one_line_up-j-1])<=(-3)&&
      //               (Left_Line[left_one_line_up-j-2]-Left_Line[left_one_line_up-j-3])<=(-3);j++)
                     if(Left_Line[left_one_line_up-j]>30)
                         Left_Add_Line(30,80,Left_Line[left_one_line_up-5],left_one_line_up-5);
                 }
      //           else if(left_one_line_down>30&&right_one_line_down<10)
      //           {
      //               Left_Add_Line(Left_Line[left_one_line_down+2],left_one_line_down+2,Left_Line[20],20);
      //           }     }
    }
    if(bridge_cancle>=800)
     {
//         bridge_cancle=0;
//         bridge_delay=0;
//         unilateral_bridge_state=0;
//         angle_Integral_start_flag=0;
//         yaw=0;
     }
}
/**************************************************************单边桥***************************************************************/
/************************************************************barrier**************************************************************/
int gain_barrier(int start,int end)//
{
    int t;
    int point=0;
    right_barrier_up=0,right_barrier_down=0,left_barrier_up=0,left_barrier_down=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {                                                                                                                       //Right_Line[i-5]-90
       if((Right_Line[i]-Right_Line[i+4])<(-10)/*&&abs(Right_Line[i-2]-Right_Line[i-3])<=3*/
               )//找第一被遮挡列
       {
           right_barrier_down=1;
           //right_barrier_down=Right_Line[i-5];
           point=i;
           break;
       }
       else if(Left_Line[i]-Left_Line[i+4]>10/*&&abs(Left_Line[i-2]-Left_Line[i-3])<=3*/)//找第一被遮挡列
       {
           left_barrier_down=1;
           //left_barrier_down=Left_Line[i-5];
           point=i;
           break;
       }
    }
    return point;
}
int left_barrier,right_barrier;
void past_barrier()
{

    barrier_point=0;
    barrier_point=gain_barrier(84,35);
    ips[12]=barrier_point;
    if(Right_Lost_Time<5&&Left_Lost_Time<5&&barrier_point>40&&Search_Stop_Line>70
            &&!Left_Island_Flag&&!Right_Island_Flag&&!Cross_Flag&&!Island_State&&!unilateral_bridge_state&&!jump_state&&!barrier_state)
    {
        barrier_state=1;
    }
    if(barrier_state)
    {
        if(barrier_point<20)
        {
            if(barreir_delay>=500)
            {
                barrier_state=0;
                barreir_delay=0;
                if(order_flag<3)
                {
                    order_flag++;
                }
            }
        }
//        else if(right_barrier_down)Right_Add_Line(Right_Line[89],89,Right_Line[barrier_point]-15,barrier_point+3);
//
//        else if(left_barrier_down)Left_Add_Line(Left_Line[89],89,Left_Line[barrier_point]+15,barrier_point+3);
        if(right_barrier_down){Right_Add_Line(Right_Line[89]-45,89,Right_Line[barrier_point-5]-15,barrier_point-5);right_barrier=1; }
        else if(right_barrier)Right_Add_Line(Right_Line[89],89,Right_Line[40]-50,40);
        if(left_barrier_down){Left_Add_Line(Left_Line[89]+45,89,Left_Line[barrier_point-5]+15,barrier_point-5);left_barrier=1;}
        else if(left_barrier)Left_Add_Line(Left_Line[89],89,Left_Line[40]+50,40);
    }
}
/************************************************************barrier**************************************************************/
/**************************************************************jump***************************************************************/
int gain_jump(int start,int end)//识别jump
{
    int t;
    int point=0;
    int left_state=0,right_state=0,left_stop=0,right_stop=0;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0,j=0;
//        for(j=89;j>10;j--)
//        {
//            if(Left_Line[j]<70&&Left_Line[j]>2&&left_stop==0)
//            {
//                left_state=j;ips[11]=left_state;
//
//            }
//            if(Left_Line[j]>70)left_stop=1;
//            if(Right_Line[j]>110&&Right_Line[j]<170&&right_stop==0)
//            {
//                right_state=j;ips[12]=right_state;
//
//            }
//            if(Right_Line[j]<110)right_stop=1;
//            if(left_stop&&right_stop)break;
//        }
    for(j=89;j>10;j--)
    {
        if(Left_Line[j]&&!Left_Line[j-1]&&!left_state)
        {
              left_state=j;ips[11]=left_state;
        }
        if(Right_Line[j]!=(MT9V03X_W-1)&&Right_Line[j-1]==(MT9V03X_W-1)&&!right_state)
        {
            right_state=j;ips[12]=right_state;
        }
    }
        if(abs(left_state-right_state)<5)
        {
        point=(left_state+right_state)/2;
        }
    return point;
}
void gain_jump_time(int start,int end)//识别jump时机
{
    int t;
//    int point=0;
    left_jump_down=0,left_jump_up=0,right_jump_down=0,right_jump_up=0;
//    if(Search_Stop_Line<=5)//搜所截止行很矮
//       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if(Left_Line[i]>20&&Left_Line[i-1]<10&&Left_Line[i-2]<10)//
       {
           left_jump_down=i;
       }
       if(Left_Line[i]>30&&Left_Line[i+1]<10&&Left_Line[i+2]<10)//
       {
           left_jump_up=i;
       }
       if(Right_Line[i]<160&&Right_Line[i-1]>170&&Right_Line[i-2]>170)//
       {
           right_jump_down=i;
       }
       if(Right_Line[i]<150&&Right_Line[i+1]>170&&Right_Line[i+2]>170)//
       {
           right_jump_up=i;
       }
       if(left_jump_up&&right_jump_up)
       {
//           point=i;
           break;

       }
    }
//    return point;
}

int jump_begin_distance=58;

void pass_jump()
{
    //gain_jump_time(63,15);
    if(Right_Lost_Time<5&&Left_Lost_Time<5&&gain_jump(70,5)>(jump_begin_distance-8)/*&&Search_Stop_Line<70*/
                &&!Left_Island_Flag&&!Right_Island_Flag&&!Cross_Flag&&!Island_State&&!unilateral_bridge_state&&!jump_state&&!barrier_state)
    {
        engine_control(ALL,0);
        streeing_switch=1;
    }
    if(Right_Lost_Time<5&&Left_Lost_Time<5&&gain_jump(70,5)>jump_begin_distance
            &&!Left_Island_Flag&&!Right_Island_Flag&&!Cross_Flag&&!Island_State&&!unilateral_bridge_state&&!jump_state&&!barrier_state)
    {
        jump_state=1;
    }
}
int detour_error;
void detour()
{
    if(detour_integer)angle_Integral_start_flag=1;
}
float error_change()                 //0~-20~0~20~0~-20~0    //-20+40-40+20    distance=1000
{
    if(detour_integer<50)detour_error=-20;
    else if(detour_integer>50&&detour_integer<250)detour_error=0;
    else if(detour_integer>250&&detour_integer<300)detour_error=20;
    else if(detour_integer>300&&detour_integer<500)detour_error=0;
    else if(detour_integer>500&&detour_integer<550)detour_error=20;
    else if(detour_integer>550&&detour_integer<750)detour_error=0;
    else if(detour_integer>750&&detour_integer<800)detour_error=-20;
    else if(detour_integer>800&&detour_integer<1000)detour_error=0;
    return detour_error;
}
int podao_time(int start,int end)//识别jump时机
{
    int podaoa_point=0;
    int point=0;
    int t;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    int i=0;
    for(i=start;i>=end;i--)
    {
       if((Right_Line[i]-Left_Line[i])>100&&(Right_Line[5]-Left_Line[5])>60)//
       {
           podaoa_point++;
       }
    }
    if(podaoa_point>10)point=1;
    return point;
}

void podao()
{
//    int podao=0;
//    podao=podao_time(40,20);
    if(dl1b_distance_cm<=30&&dl1b_distance_cm>=5&&Search_Stop_Line>85&&Island_State==0&&jump_state==0&&unilateral_bridge_state==0&&barrier_state==0&&Cross_Flag==0)
    {
        engine_slow(50);
        podao_reday=1;
        speed=400;
        engine_limit_max=L5*0.7-7;
        engine_limit_min=-L5*0.7;
    }
    else if(podao_reday==1&&(dl1b_distance_cm>=30||dl1b_distance_cm<5))
    {
        podao_reday=0;
        podao_state=1;
    }
    else if(podao_delay>=1200&&podao_state==1)
    {
        podao_state=0;
        podao_reday=0;
        podao_delay=0;
        speed=Normal_speed;
        engine_limit_max=L5*1.5-7;
        engine_limit_min=-L5;
        engine_slow(40);
    }

}
/**************************************************************jump***************************************************************/
/***********************************************************handle middle*********************************************************/
void middle_line_amend()
{
    int dif_w;
    if(first_lose[0]>20)
    {
//        end_h=(first_lose[0]-first_lastylose[0])/2;w=180-Mid_Line[first_lose[0]];
        dif_w=Left_Line[first_lose[0]]-Mid_Line[first_lose[0]];
        for(int i = first_lose[0];i>MT9V03X_H-Search_Stop_Line-1;i--)
        {
            Mid_Line[i]=Left_Line[Mid_Line[i]]-dif_w;
        }
    }
    if(first_lose[1]>20)
    {
//        end_h=(first_lose[1]-first_lastylose[1])/2;w=Mid_Line[first_lose[1]];
        dif_w=Right_Line[first_lose[1]]-Mid_Line[first_lose[1]];
        for(int i = first_lose[0];i>MT9V03X_H-Search_Stop_Line-1;i--)
        {
            Mid_Line[i]=Right_Line[Mid_Line[i]]-dif_w;
        }
    }
}
void reduce_mutational_site()
{

}
/***********************************************************handle middle*********************************************************/

int zaber_start_flag=0;
/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
    /*这是离线调试用的*/
    Get_image(mt9v03x_image);
    turn_to_bin();//大津法获得二值化图像 bin_image
    /*提取赛道边界*/
    image_filter(bin_image);//滤波
    image_draw_rectan(bin_image);//预处理
//    //清零
    data_stastics_l = 0;
    data_stastics_r = 0;
    for(int j=0;j<BOUNDARY_NUM;j++)         //给图传数据清零，便于显示图像
        {
            xy_x1_boundary[j]=0;
            xy_y1_boundary[j]=0;
            xy_x2_boundary[j]=0;
            xy_y2_boundary[j]=0;
            xy_x3_boundary[j]=0;
            xy_y3_boundary[j]=0;
        }
    //ips[1]=0;ips[2]=0;ips[3]=0;
    if (get_start_point(image_h - 2))//找到图像下端起点了，再执行八领域，没找到就一直找
    {
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);

        // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);

        //处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍
        if(zaber_start_flag==1)
        {
            zebra_get();
        }
    }
    showimage();
}

void showimage(void)
{
    int i,j=0;
    uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
    //显示图像   改成你自己的就行 等后期足够自信了，显示关掉，显示屏挺占资源的
//    ips200_show_gray_image(0,16*6,bin_image[0], image_w, image_h, image_w, image_h,0);
        //根据最终循环次数画出边界点
//        for (i = 0; i < data_stastics_l; i++)
//        {
//            ips200_draw_point(points_l[i][0]+2, points_l[i][1], uesr_BLUE);//显示起点
//        }
//        for (i = 0; i < data_stastics_r; i++)
//        {
//            ips200_draw_point(points_r[i][0]-2, points_r[i][1], uesr_RED);//显示起点
//        }
//
        for (i = hightest; i < image_h-1; i++)
        {
            center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
//            //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
//            //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
//                ips200_draw_point(center_line[i], i, uesr_GREEN);//显示起点 显示中线
//                ips200_draw_point(l_border[i], i, uesr_GREEN);//显示起点 显示左边线
//                ips200_draw_point(r_border[i], i, uesr_GREEN);//显示起点 显示右边线
        }

//        for(i = hightest; i < image_h-1; i++)
//        {
//            xy_x1_boundary[j]=l_border[i];
//            xy_y1_boundary[j]=i;
//            xy_x2_boundary[j]=r_border[i];
//            xy_y2_boundary[j]=i;
//            xy_x3_boundary[j]=center_line[i];
//            xy_y3_boundary[j]=i;
//            j++;
//        }
        for(i=MT9V03X_H-1;i>=3;i--,j++)//从最底下往上扫描
            {
//            bin_image[i][l_border[i]]=IMG_BLACK;
//            bin_image[i][(l_border[i]+r_border[i])>>1]=IMG_BLACK;
//            bin_image[i][Right_Line[i]]=IMG_BLACK;

                    xy_x1_boundary[j]=l_border[i]+2;
                    xy_y1_boundary[j]=i;
                    xy_x2_boundary[j]=r_border[i]-2;
                    xy_y2_boundary[j]=i;
                    xy_x3_boundary[j]=(l_border[i]+r_border[i])>>1;
                    xy_y3_boundary[j]=i;
            }
}

int order_flag,stop_glag=0;

void image_disposal(void)
{

    if(mt9v03x_finish_flag)//图像获取
            {
//                if(Img_Disappear_Flag==0)//图没有丢，正常计算阈值
//                {
                //histogram_equalization(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
                    Threshold=My_Adapt_Threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//大津计算阈值
//                }
//                else
//                {
//                  //出界后不算阈值，确保出去之后屏幕是黑的，防止出现雪花屏
//                }
                Image_Binarization(Threshold);//图像二值化
                //optimizeBinaryImage(image_two_value);
                //image_filter(image_two_value);//滤波
//                yaw_err=0;

                Longest_White_Column();//最长白列巡线

                //zebra_get_fube();
                Island_Detect();//圆环
                Cross_Detect();//十字
//                pass_jump();//跳跃
                if(stop_glag==0)
                {
                    switch(Element_Order[order_flag])
                    {

                        case 0:
                                Element_Order[0]=0;
                                Element_Order[1]=0;
                                Element_Order[2]=0;
                                Element_Order[3]=0;
                                order_flag=0;
                                stop_glag=1;
                                break;
                        case 1:
                                past_barrier();
                                break;
                        case 2:
                                unilateral_bridge_one_line();
                                unilateral_bridge_and_avoide_barriers();//单边桥
                                break;

                    }
                }
                if(podao_switch==1)
                {
                    podao();
                }
                mt9v03x_finish_flag=0;//标志位清除，自行准备采集下一帧数据
            }
            Show_Boundry();//显示

}

