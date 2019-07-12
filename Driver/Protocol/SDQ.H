#ifndef __SDQ_H
#define __SDQ_H

#include "stm32f10x_gpio.h"


typedef	struct	_sdq_port				//
{
	//-------------------------SDQ驱动
	GPIO_TypeDef*	SDQ_Port;
	u16	SDQ_Pin;
}sdq_port_def;


typedef	enum	_sdq_status				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
	sdq_ilde						=	0,	//空闲
	sdq_start						=	1,	//启动
	sdq_master_readCRC	= 2,	//读CRC
	sdq_master_read			= 3,	//读
	sdq_master_read_digest			= 3,	//读
	sdq_master_write		=	4,	//写
}sdq_status_def;
typedef enum
{
	sdq_rom_idle			=	(unsigned char)0x00,			//
	sdq_rom_MatchID		=	(unsigned char)0x55,			//
	sdq_rom_SkipID		=	(unsigned char)0xCC,			//
	sdq_rom_ReadID		=	(unsigned char)0x33,			//
	sdq_rom_SearchID	=	(unsigned char)0xF0,			//
}sdq_rom_cmd;
typedef enum
{
	sdq_mem_idle					=	(unsigned char)0x00,			//
	sdq_mem_read_memory		=	(unsigned char)0xF0,			//
	sdq_mem_write_memory	=	(unsigned char)0x0F,
	sdq_mem_read_page			=	(unsigned char)0xC3,			//
	sdq_mem_read_digest		=	(unsigned char)0xDD,			//
	sdq_mem_write_message	=	(unsigned char)0x22,			//
	sdq_mem_read_status		=	(unsigned char)0xAA,
	sdq_mem_write_status	=	(unsigned char)0x55,
	sdq_mem_read_control	=	(unsigned char)0x88,
	sdq_mem_write_control	=	(unsigned char)0x77,
	sdq_mem_read_eeprom		=	(unsigned char)0xE0,
	sdq_mem_write_eeprom	=	(unsigned char)0x0E,
	sdq_mem_profile				=	(unsigned char)0x99,
	sdq_mem_read_page4		=	(unsigned char)0xFA,
	sdq_mem_write_page4		=	(unsigned char)0xAF,
}sdq_mem_cmd;

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
	unsigned long 	intrrcout;		//中断次数计数
	unsigned long 	startcout;		//启动次数计数
	unsigned short 	address;			//地址
	unsigned char 	ackbitlen;		//应答位计数
	unsigned char 	ackdata;			//应答数据
	unsigned char 	bitlen;				//位数计数
	unsigned char 	messgaelen;		//消息长度计数
	unsigned char 	digestlen;		//摘要长度计数
	unsigned char 	receivelen;		//消息长度计数
	unsigned char 	temp;					//位数据
	unsigned char 	receive[128];	//消息数组
	unsigned char 	message[20];	//消息数组
	unsigned char 	digest[20];		//
}sdq_data_def;

typedef	struct	_sdq				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
//	sdq_port_def 		port;
	sdq_status_def	status;
	sdq_time_def		time;
	sdq_data_def		data;
	sdq_rom_cmd			RomCmd;
	sdq_mem_cmd			MemCmd;
}sdq_def;




void api_sdq_slave_configuration(sdq_def *sSDQ);		//SDQ从机设备配置
void api_sdq_slave_server(void);		//SDQ从机服务

static void api_sdq_process(void);
static void api_sdq_data_process(void);
unsigned char api_sdq_receive_process(void);

unsigned char api_sdq_set_bit(unsigned char bit);
unsigned char api_sdq_send_byte(void);

unsigned char api_sdq_get_bit(void);
unsigned char api_sdq_get_byte(unsigned char bit);
unsigned char api_sdq_set_start(void);

//unsigned char api_sdq_get_level(void);














#endif

