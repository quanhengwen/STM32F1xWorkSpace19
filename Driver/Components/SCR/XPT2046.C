

/********************************************************************************
***XPT2046触摸控制IC
********************************************************************************/
#include "XPT2046.H"
#include "stdint.h"



//读取一个坐标值
//连续读取READ_TIMES次数据,对这些数据升序排列,
//然后去掉最低和最高LOST_VAL个数,取平均值 
#define READ_TIMES 15 //读取次数
#define LOST_VAL 5	  //丢弃值


//***因触摸屏批次不同等原因，默认的校准参数值可能会引起触摸识别不准，建议校准后再使用，不建议使用固定的默认校准参数
unsigned short vx=4809,vy=7382;  //比例因子，此值除以1000之后表示多少个AD值代表一个像素点
unsigned short chx=102,chy=296;//默认像素点坐标为0时的AD起始值
//***因触摸屏批次不同等原因，默认的校准参数值可能会引起触摸识别不准，建议校准后再使用，不建议使用固定的默认校准参数

struct tp_pix_  tp_pixad,tp_pixlcd;	 //当前触控坐标的AD值,前触控坐标的像素值

spi_def* pXpt2046;

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_xpt2046_configuration(spi_def* pInfo)
{
	pXpt2046	=	pInfo;
  //api_spi_configuration_gpio(pInfo);				//普通SPI通讯方式配置
	api_spi_configurationNR(pInfo);				//SPI接口配置

}
//-----------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	api_xpt2046_get_coordinate
*功能描述		:	带滤波的坐标读取,最小值不能少于100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_xpt2046_get_coordinate(unsigned short *x,unsigned short *y)
{
	unsigned short xtemp,ytemp;	
	xtemp=xpt2046_get_ad_filter(CMD_RDX);		//获取X轴位置
	ytemp=xpt2046_get_ad_filter(CMD_RDY);		//获取Y轴位置
	if(xtemp<100||ytemp<100)
		return 0;//读数失败
	*x=xtemp;
	*y=ytemp;
	return 1;//读数成功
}
//-----------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	xpt2046_get_ad
*功能描述		:	从7846/7843/XPT2046/UH7843/UH7846读取adc值	  0x90=y   0xd0-x
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short xpt2046_get_ad(unsigned char CMD)          
{
	unsigned short l;
	spi_set_nss_low(pXpt2046);
	/* 在差分模式下，XPT2046 转换需要24个时钟，8个时钟输入命令，之后1个时钟去除 */
	//1)--------------------发送转换命令
	api_spi_ReadWrite_byte(pXpt2046,CMD);        				//送控制字即用差分方式读X坐标 详细请见有关资料
	//2)--------------------读取数据，其中第一个时钟要去除
	xpt2046_DelayNop(50);
	l	=	api_spi_ReadWrite_byte(pXpt2046,0xFF)&0x7F;    	//第一个时钟去除，//送控制字即用差分方式读X坐标 详细请见有关资料
	l<<=8;
	l	|=	api_spi_ReadWrite_byte(pXpt2046,0xFF);        //送控制字即用差分方式读X坐标 详细请见有关资料
	//3)--------------------只有12位有效值，后3位无效
	l>>=3;
	spi_set_nss_high(pXpt2046);
	return l;
}
/*******************************************************************************
*函数名			:	xpt2046_get_ad_filter
*功能描述		:	滤波处理
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short xpt2046_get_ad_filter(unsigned char xy)
{
	/* 去掉最大值，去掉最小值，求平均值 */
	unsigned short i, j;
	unsigned short buf[READ_TIMES];
	unsigned short sum=0;
	unsigned short temp;
	//1)-------------------------采样数据
	for(i=0;i<READ_TIMES;i++)
	{				 
		buf[i]=xpt2046_get_ad(xy);	    
	}
	//2)-------------------------从小到大排序
	for(i=0;i<READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
	//3)-------------------------去掉几个最大和最小值
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
		sum+=buf[i];
	//4)-------------------------求平均值
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
}

//------------------------------------------------------------------------------

   


unsigned char tpstate(void)
{
//	return 	Penirq;
}
//**********************************************************
void spistar(void)                                     //SPI开始
{
//	CS=1;
//	DCLK=1;
//	DIN=1;
//	DCLK=1;
}

//2次读取ADS7846,连续读取2次有效的AD值,且这两次的偏差不能超过
//50,满足条件,则认为读数正确,否则读数错误.	   
//该函数能大大提高准确度
#define ERR_RANGE 20 //误差范围 
unsigned char Read_ADS2(unsigned short *x,unsigned short *y) 
{
	unsigned short x1,y1;
 	unsigned short x2,y2;
 	unsigned char flag;    
    flag=api_xpt2046_get_coordinate(&x1,&y1);   
    if(flag==0)return(0);
    flag=api_xpt2046_get_coordinate(&x2,&y2);	
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//前后两次采样在+-ERR_RANGE内
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)>>1;
        *y=(y1+y2)>>1;		
        return 1;
    }else return 0;	  
} 
//精确读取一次坐标,校准的时候用的	   
unsigned char Read_TP_Once(void)
{
	unsigned char re=0;
	unsigned short x1,y1;
	unsigned short x2,y2;
	while(re==0)
	{
		while(!api_xpt2046_get_coordinate(&x1,&y1));
//		delayms(10);
		while(!api_xpt2046_get_coordinate(&x2,&y2));
		if((x2<=x1&&x1<x2+3)||(x1<=x2&&x2<x1+3))
		{
			tp_pixad.x=(x1+x2)/2;
			tp_pixad.y=(y1+y2)/2;
			re=1; 
		}	
	} 
	return re;
}


//////////////////////////////////////////////////
//与LCD部分有关的函数  
//画一个触摸点
//用来校准用的
void Drow_Touch_Point(unsigned short x,unsigned short y)
{
//	LCD_DrawLine(x-12,y,x+13,y);//横线
//	LCD_DrawLine(x,y-12,x,y+13);//竖线
//	LCD_DrawPoint(x+1,y+1);
//	LCD_DrawPoint(x-1,y+1);
//	LCD_DrawPoint(x+1,y-1);
//	LCD_DrawPoint(x-1,y-1);
}	  
//转换结果
//根据触摸屏的校准参数来决定转换后的结果,保存在X0,Y0中
unsigned char Convert_Pos(void)
{		 	 
	unsigned char l=0; 
	if(api_xpt2046_get_coordinate(&tp_pixad.x,&tp_pixad.y))
	{
		l=1;
		tp_pixlcd.x=tp_pixad.x>chx?((unsigned long)tp_pixad.x-(unsigned long)chx)*1000/vx:((unsigned long)chx-(unsigned long)tp_pixad.x)*1000/vx;
		tp_pixlcd.y=tp_pixad.y>chy?((unsigned long)tp_pixad.y-(unsigned long)chy)*1000/vy:((unsigned long)chy-(unsigned long)tp_pixad.y)*1000/vy;
	}
	return l;
}	   
//触摸屏校准代码
//得到四个校准参数
#define tp_pianyi 80   //校准坐标偏移量	
#define tp_xiaozhun 1000   //校准精度
void Touch_Adjust(void)
{	
	float vx1,vx2,vy1,vy2;  //比例因子，此值除以1000之后表示多少个AD值代表一个像素点
	unsigned short chx1,chx2,chy1,chy2;//默认像素点坐标为0时的AD起始值
	unsigned short lx,ly;				 
	struct tp_pixu32_ p[4];
	unsigned char  cnt=0;	 
	cnt=0;				
//	POINT_COLOR=BLUE;
//	BACK_COLOR =WHITE;
//	LCD_Clear(WHITE);//清屏   
//	POINT_COLOR=RED;//红色 
//	LCD_Clear(WHITE);//清屏 
//	Drow_Touch_Point(tp_pianyi,tp_pianyi);//画点1 
//	while(1)
//	{
//		if(Penirq==0)//按键按下了
//		{
//			if(Read_TP_Once())//得到单次按键值
//			{  								   
//				p[cnt].x=tp_pixad.x;
//				p[cnt].y=tp_pixad.y;
//				cnt++; 
//			}			 
//			switch(cnt)
//			{			   
//				case 1:
//					LCD_Clear(WHITE);//清屏 
//					while(!Penirq)  //等待松手
//					{
//					}
//					Drow_Touch_Point(LCD_W-tp_pianyi-1,tp_pianyi);//画点2
//					break;
//				case 2:
//					LCD_Clear(WHITE);//清屏 
//					while(!Penirq)  //等待松手
//					{
//					}
//					Drow_Touch_Point(tp_pianyi,LCD_H-tp_pianyi-1);//画点3
//					break;
//				case 3:
//					LCD_Clear(WHITE);//清屏 
//					while(!Penirq)  //等待松手
//					{
//					}
//					Drow_Touch_Point(LCD_W-tp_pianyi-1,LCD_H-tp_pianyi-1);//画点4
//					break;
//				case 4:	 //全部四个点已经得到
//	    		   	LCD_Clear(WHITE);//清屏 
//				   	while(!Penirq)  //等待松手
//					{
//					}
//			   		vx1=p[1].x>p[0].x?(p[1].x-p[0].x+1)*1000/(LCD_W-tp_pianyi-tp_pianyi):(p[0].x-p[1].x-1)*1000/(LCD_W-tp_pianyi-tp_pianyi);
//				 	chx1=p[1].x>p[0].x?p[0].x-(vx1*tp_pianyi)/1000:p[0].x+(vx1*tp_pianyi)/1000;
//				   	vy1=p[2].y>p[0].y?(p[2].y-p[0].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi):(p[0].y-p[2].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi);
//					chy1=p[2].y>p[0].y?p[0].y-(vy1*tp_pianyi)/1000:p[0].y+(vy1*tp_pianyi)/1000; 
//					
//					vx2=p[3].x>p[2].x?(p[3].x-p[2].x+1)*1000/(LCD_W-tp_pianyi-tp_pianyi):(p[2].x-p[3].x-1)*1000/(LCD_W-tp_pianyi-tp_pianyi);
//					chx2=p[3].x>p[2].x?p[2].x-(vx2*tp_pianyi)/1000:p[2].x+(vx2*tp_pianyi)/1000;
//				   	vy2=p[3].y>p[1].y?(p[3].y-p[1].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi):(p[1].y-p[3].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi);
//					chy2=p[3].y>p[1].y?p[1].y-(vy2*tp_pianyi)/1000:p[1].y+(vy2*tp_pianyi)/1000; 


//					if((vx1>vx2&&vx1>vx2+tp_xiaozhun)||(vx1<vx2&&vx1<vx2-tp_xiaozhun)||(vy1>vy2&&vy1>vy2+tp_xiaozhun)||(vy1<vy2&&vy1<vy2-tp_xiaozhun))
//					{
//						cnt=0;
//						LCD_Clear(WHITE);//清屏 
//						Drow_Touch_Point(tp_pianyi,tp_pianyi);//画点1 
//						continue;
//					}
//					vx=(vx1+vx2)/2;vy=(vy1+vy2)/2;
//					chx=(chx1+chx2)/2;chy=(chy1+chy2)/2;	
//																
//					//显示校准信息
//					LCD_Clear(WHITE);//清屏 
//					POINT_COLOR=BLACK;
//					BACK_COLOR=BLUE;	
//			
//					lx=0;ly=50;			
//					LCD_ShowString(lx,ly,"VX1:");lx+=40;LCD_ShowNum(lx,ly,vx1,4);					
//					lx=0;ly+=20;
//					LCD_ShowString(lx,ly,"Vy1:");lx+=40;LCD_ShowNum(lx,ly,vy1,4);					
//					lx=0;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX1:");lx+=40;LCD_ShowNum(lx,ly,chx1,4);					
//				    lx=0;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY1:");lx+=40;LCD_ShowNum(lx,ly,chy1,4);

//					lx=100;ly=50;			
//					LCD_ShowString(lx,ly,"VX2:");lx+=40;LCD_ShowNum(lx,ly,vx2,4);					
//					lx=100;ly+=20;
//					LCD_ShowString(lx,ly,"Vy2:");lx+=40;LCD_ShowNum(lx,ly,vy2,4);					
//					lx=100;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX2:");lx+=40;LCD_ShowNum(lx,ly,chx2,4);					
//				    lx=100;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY2:");lx+=40;LCD_ShowNum(lx,ly,chy2,4);
//				
//					lx=50;ly=150;			
//					LCD_ShowString(lx,ly,"VX:");lx+=40;LCD_ShowNum(lx,ly,vx,4);					
//					lx=50;ly+=20;
//					LCD_ShowString(lx,ly,"Vy:");lx+=40;LCD_ShowNum(lx,ly,vy,4);					
//					lx=50;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX:");lx+=40;LCD_ShowNum(lx,ly,chx,4);					
//				    lx=50;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY:");lx+=40;LCD_ShowNum(lx,ly,chy,4);

//					lx=30;ly+=30;
//					LCD_ShowString(lx,ly,"Adjust OK!  Touch Anywhere To Continue");										  
//					Read_TP_Once(); //等待任意键后继续

//					LCD_Clear(WHITE);//清屏
//					return;//校正完成				 
//			}
//		}
//	} 
}
void point(void) //绘图函数
{
	double t=0;
	
//    while(1)
//	{  	
//		if(Penirq==0)
//		{
//			t=0;
//			if(Convert_Pos())	//得到坐标值
//			{
//			//	LCD_ShowString(10,250,"X:");LCD_ShowNum(30,250,(u32)tp_pixad.x,6);	
//				//LCD_ShowString(180,250,"Y:");LCD_ShowNum(200,250,(u32)tp_pixad.y,6);	
//				LCD_ShowString(10,250,"X:");LCD_ShowNum(30,250,tp_pixad.x,4);
//				LCD_ShowString(180,250,"Y:");LCD_ShowNum(200,250,tp_pixad.y,4);					
//				LCD_DrawPoint_big(tp_pixlcd.x,tp_pixlcd.y);   
//			 }
//			
//		}
//		else
//		{	
//			t++;		
//			if(t>65000)
//			{
//				return;
//			}
//		}		

//		}
}	    
//-----------------------------------------------------------------------------









/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void xpt2046_DelayNop(unsigned	short Time)
{
	while(Time--)
	{
		__nop();
	}
}


