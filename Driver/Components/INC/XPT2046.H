//XPT2046触摸控制IC
#ifndef __XPT2046_H
#define __XPT2046_H

#include "STM32_SPI.H"

typedef struct	_xpt2046
{
	spi_def		port;
}xpt2046def;





void api_xpt2046_configuration(spi_def* pInfo);
unsigned short api_xpt2046_get_coordinate(unsigned short *x,unsigned short *y);		//带滤波的坐标读取,最小值不能少于100




static unsigned short xpt2046_get_ad(unsigned char CMD);		//读AD数据
static unsigned short xpt2046_get_ad_filter(unsigned char xy);	//滤波处理


void xpt2046_DelayNop(unsigned	short Time);

/* touch panel interface define */
#define  CMD_RDX  0xD0	 //触摸IC读坐标积存器
#define  CMD_RDY  0x90
//IO连接
//sbit DCLK	   =    P1^7;   
//sbit CS        =    P1^4;
//sbit DIN       =    P3^0;	
//sbit DOUT      =    P3^1;																						   
//sbit Penirq    =    P3^4;   //检测触摸屏响应信号

struct tp_pix_
{
	unsigned short x;
	unsigned short y;
};
struct tp_pixu32_
{
	unsigned long x;
	unsigned long y;
};
extern struct tp_pix_  tp_pixad,tp_pixlcd;	 //当前触控坐标的AD值,前触控坐标的像素值   
extern unsigned short vx,vy;  //比例因子，此值除以1000之后表示多少个AD值代表一个像素点
extern unsigned short chx,chy;//默认像素点坐标为0时的AD起始值
unsigned char tpstate(void);
void spistar(void);  
void Touch_Adjust(void);
void point(void); //绘图函数





#endif  

