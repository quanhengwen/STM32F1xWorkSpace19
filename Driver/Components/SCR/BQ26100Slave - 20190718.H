#ifndef __BQ26100Slave_H
#define __BQ26100Slave_H

#include "stm32f10x_gpio.h"

#define sdq_receive_buffer_size	256		//数据接收缓冲大小
#define sdq_sample_size			270			//样品数据个数

typedef	struct	_sdq_port				//
{
	//-------------------------SDQ驱动
	GPIO_TypeDef*	SDQ_Port;
	u16	SDQ_Pin;
}sdq_port_def;


typedef	enum	_sdq_status				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
	sdq_ilde								=	0,	//空闲
	sdq_start								=	1,	//启动
	sdq_master_readCRC			= 2,	//主机读CRC
	sdq_master_read					= 3,	//主机读
	//sdq_master_read_digest	= 5,	//读
	sdq_master_write				=	6,	//主机写.
	sdq_error								=	7,	//主机写
}sdq_status_def;
typedef enum
{
	sdq_rom_idle			=	(unsigned char)0x00,			//
	sdq_rom_MatchID		=	(unsigned char)0x55,			//
	sdq_rom_SkipID		=	(unsigned char)0xCC,			//
	sdq_rom_ReadID		=	(unsigned char)0x33,			//
	sdq_rom_SearchID	=	(unsigned char)0xF0,			//
}sdq_rom_cmd;
typedef enum		//相对于主机
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
//	unsigned short high;				//高电平时间
//	unsigned short low;					//低电平时间
	unsigned long	 time1;				//开始前时间
	unsigned long  time2;				//新时间
	unsigned long  time_sys;		//总时间
}sdq_time_def;

typedef	struct	_sdq_message			//
{	
	unsigned char 	MessageLen;				//摘要长度计数
	//unsigned char 	MessageCrcFlag;		//摘要CRC设置标志
	//unsigned char 	MessageGetFlag;		//获取对message对应的Digest标志
	//unsigned char 	MessageGetStart;	//开始传输数据
	//unsigned char 	Serial;						//已发接收message数据计数
	unsigned char 	Buffer[20];				//
}bq26100slave_message_def;

typedef	struct	_sdq_digest			//
{
	struct
	{
		unsigned char Auth		:1;		//0--未设置控制位/未启动转换上报主机发送的message；1--已设置控制位，可以匹配digest并准备上报
		unsigned char Start		:1;		//0--未启动传输，需要做传输前数据处理；1--已启动传输，做传输处理
	}Flag;
	unsigned char 	Serial;					//已发送摘要计数
	unsigned char 	Buffer[20];			//消息摘要
}bq26100slave_digest_def;

typedef	struct	_bq26100slave_ctrl			//
{	
	struct
	{
		unsigned char Start		:1;		//0--未启动传输，需要做传输前数据处理；1--已启动传输，做传输处理
	}Flag;
	struct
	{
		unsigned char Ctrl0;		//地址0
		unsigned char Ctrl1;		//地址1
		unsigned char CrcData;	//RAM,AddressL,AddressH校验结果
		unsigned char Times;		//第Times读取，从0开始
		unsigned char Serial;		//主机已读取计数
	}Read;
	struct
	{
		unsigned char Ctrl0R;		//地址0接收的数据
		unsigned char Ctrl0T;		//地址0发送的数据
		unsigned char Ctrl1;		//地址1
		unsigned char Serial;		//主机已写入计数
	}Write;
}bq26100slave_ctrl_def;

typedef	struct	_sdq_data			//
{	
	unsigned long 	intrrcout;		//中断次数计数
	unsigned long 	startcout;		//启动次数计数
	unsigned short 	receivelen;		//消息长度计数
	unsigned char 	addressL;			//地址-低字节
	unsigned char 	addressH;			//地址-高字节
	unsigned char 	SendBitLen;		//应答位计数
	unsigned char 	SendByte;			//应答数据	
	unsigned char 	RcvBitlen;		//位数接收计数
	unsigned char 	RcvByte;			//位数据
	unsigned char 	Ctllen;				//控制数据计算
	unsigned char 	CtlFlag;			//控制标志位---判断是否发送控制消息
//	unsigned char 	messgaelen;		//消息长度计数
//	unsigned char 	digestlen;		//摘要长度计数
//	unsigned char 	DigestGetFlag;		//获取对message对应的Digest标志
//	unsigned char 	DigestGetStart;		//开始传输数据
	unsigned char 	receive[sdq_receive_buffer_size];	//消息数组
//	unsigned char 	message[20];	//消息数组
	unsigned char 	CtlData[10];			//控制命令
	
	
	bq26100slave_message_def		Message;
	bq26100slave_digest_def			Digest;
	bq26100slave_ctrl_def				Ctrl;	
}bq26100slave_data_def;

typedef	struct	_sdq				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
//	sdq_port_def 		port;	
	sdq_time_def						time;
	bq26100slave_data_def		data;
	sdq_rom_cmd							RomCmd;
	sdq_mem_cmd							MemCmd;
	sdq_status_def					status;
}bq26100slave_def;




void api_bq26100slave_configuration(bq26100slave_def *sSDQ);		//SDQ从机设备配置
void api_bq26100slave_server(void);		//SDQ从机服务

unsigned char api_bq26100slave_process(void);

unsigned char api_bq26100slave_Initialize(unsigned char bit);
unsigned char api_bq26100slave_command_process(unsigned char bit);


unsigned char api_bq26100slave_MenCmd(void);
unsigned char api_bq26100slave_read_digest(unsigned char bit);					//主机读摘要
unsigned char api_bq26100slave_write_message(unsigned char bit);				//主机写消息
unsigned char api_bq26100slave_read_control(unsigned char bit);		//主机读控制寄存器
unsigned char api_bq26100slave_write_control(unsigned char bit);		//主机写控制寄存器


unsigned char api_bq26100slave_digest_match_error(void);
unsigned char api_bq26100slave_get_error_status(void);			//无错误返回0，有错误返回1
unsigned char api_bq26100slave_reset_error_status(void);


unsigned char api_bq26100slave_send_bit(unsigned char bit);
unsigned char api_bq26100slave_receive_bit(void);

unsigned char api_bq26100slave_send_byte(void);
unsigned char api_bq26100slave_receive_byte(unsigned char bit);

unsigned char api_bq26100slave_start(void);
unsigned char api_bq26100slave_default(void);

//unsigned char api_sdq_get_level(void);














#endif

