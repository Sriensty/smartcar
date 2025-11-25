#ifndef __SEARCH_H_
#define __SEARCH_H_
void Search();
#define KEIL 0
typedef unsigned char      uint8;   //定义与平台无关的无符号8 位数据类型
typedef   signed char      int8;   //定义与平台无关的有符号8 位数据类型
typedef unsigned short     uint16;  //定义与平台无关的无符号16位数据类型
typedef   signed short     int16;  //定义与平台无关的有符号16位数据类型
typedef unsigned int      uint32;  //定义与平台无关的无符号32位数据类型
typedef   signed int      int32;  //定义与平台无关的有符号32位数据类型
typedef unsigned char INT8U;
typedef unsigned char UINT8;
typedef signed char INT8S;
typedef signed char INT8;
typedef unsigned short INT16U;
typedef unsigned short UINT16;
typedef signed short INT16S;
typedef signed short INT16;
typedef unsigned int INT32U;
typedef unsigned int UINT32;
typedef signed int INT32S;
typedef signed int INT32;
typedef unsigned long long INT64U;
typedef unsigned long long UINT64;
typedef long long INT64S;
typedef long long INT64;
/*----------------------- M A C R O S ----------------------------------------*/

/*********** 摄像头参数***********/
#define   MAX_VIDEO_POINT           160             //每行采集的点数
#define   MID_VIDEO_POINT           127              //图像中心点
#define   MAX_VIDEO_LINE	        97              //每场实际用到的行数

typedef struct      //图像数据结构  以左下角为坐标原点 向上为x，向右为y
{
	int16 x;
	int16 y;
}Int16_point;
typedef struct 		//向上为x向右为y 以(0,0)点为原点
{
	INT16S x;
	INT16S y;
}_POINT;


/*************图像处理***********/
#define   MAX_POINT_NUM                 127                  //边沿和中心最大个数
#define   WIDTH                         15                 //第一行白块宽度要求 根据前瞻以及看到的图像情况适当调整
#define   INIT_DATA			            250                 //无效数据

#define MIN(a,b)               ((a<b)?(a):(b))
#define MAX(a,b)               ((a>b)?(a):(b))
#define ABS(x)                 (((x)>0)?(x):(-(x)))

// 声明为外部变量，使外部的原文件可使用
extern uint8        g_VideoImageData[MAX_VIDEO_LINE][MAX_VIDEO_POINT];  //图像原始数据，g_VideoImageData[m][n]代表图像的从上数第m行，从左数第n点，0为最黑，255为最白
extern int IMAGE_MIDDLE;
//==============================图像数据================================//
extern _POINT  g_CenterPosition[MAX_POINT_NUM];
extern _POINT  g_LeftEdge[MAX_POINT_NUM], g_RightEdge[MAX_POINT_NUM];
extern int          g_CenterNum;
extern int          g_LeftEdgeNum, g_RightEdgeNum;
extern int			g_DirectionControl;
extern short g_isStart, g_isEnd, g_stopFrame, notEnd, cross_cont, last_is_cross;
extern short g_element_type; //0：默认，1:左弯道，2:右弯道，3：十字，4：斑马线
extern double g_variance;
extern short barrier_flag;
// 停车库数据
extern int g_garage_state;    // 0=不在，1=在
extern int g_garage_event;    // 0=无，1=进入，2=离开
extern int g_garage_cool;     // 冷却计数（进入后N帧抑制“离开”）
extern int g_garage_anchor_x; // 竖直补线的列锚点
#endif