#ifndef __SDQ_H
#define __SDQ_H

#include "stm32f10x_gpio.h"


typedef	enum	_sdq_status				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
	sdq_ilde					=	0,	//空闲
	sdq_start					=	1,	//启动
	sdq_master_read		= 2,	//读
	sdq_master_write	=	3,	//写
}sdq_status_def;

typedef	struct	_sdq_port				//
{
	//-------------------------SDQ驱动
	GPIO_TypeDef*	SDQ_Port;
	u16	SDQ_Pin;
}sdq_port_def;

typedef	struct	_sdq_time				//
{	
	unsigned char	 timeper;			//根据系统时钟计时的时间分频
	unsigned short count;				//计时时间
	unsigned short high;				//高电平时间
	unsigned short low;					//低电平时间
	unsigned long	 time1;				//开始前时间
	unsigned long  time2;				//新时间
	unsigned long  time_sys;		//总时间
}sdq_time_def;

typedef	struct	_sdq_data			//
{	
	unsigned long intrrcout;		//中断次数计数
	unsigned long startcout;		//启动次数计数
	unsigned char bitlen;				//位数计数
	unsigned char messgaelen;		//消息长度计数
	unsigned char temp;					//位数据
	unsigned char message[256];	//消息数组
}sdq_data_def;

typedef	struct	_sdq				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
	sdq_port_def 		port;
	sdq_status_def	status;
	sdq_time_def		time;
	sdq_data_def		data;
}sdq_def;




void api_sdq_slave_configuration(sdq_def *sSDQ);		//SDQ从机设备配置
void api_sdq_slave_server(void);		//SDQ从机服务

static void api_sdq_process(void);
static void api_sdq_data_process(void);

unsigned char api_sdq_get_bit(void);
unsigned char api_sdq_get_byte(void);
unsigned char api_sdq_set_start(void);

unsigned char api_sdq_get_level(void);














#endif

