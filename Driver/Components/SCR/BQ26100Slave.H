#ifndef __BQ26100Slave_H
#define __BQ26100Slave_H

#include "stm32f10x_gpio.h"


#define sdq_receive_buffer_size	256		//数据接收缓冲大小
//#define sdq_sample_size			250			//样品数据个数

typedef	struct	_sdq_port				//
{
	//-------------------------SDQ驱动
	GPIO_TypeDef*	SDQ_Port;
	u16	SDQ_Pin;
}sdq_port_def;
//------------------------------------------------------------------------------
typedef	enum	_sdq_MasterStatus			//主机当前执行的状态
{
	//-------------------------SDQ驱动
	sdq_master_ilde								=	0,	//空闲
	sdq_master_read					= 3,	//主机读
	sdq_master_write				=	6,	//主机写.
}sdq_MasterStatus_def;
//------------------------------------------------------------------------------
typedef	enum	_sdq_TransType				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDQ驱动
	sdq_Trans_ilde		=	0,		//空闲
	sdq_Trans_CRC			= 1,		//传输CRC
	sdq_Trans_Data		= 2,		//传输数据
	sdq_Trans_error		=	0xFF,	//传输出错
}sdq_TransType_def;
//------------------------------------------------------------------------------
typedef enum
{
	sdq_rom_idle			=	(unsigned char)0x00,			//
	sdq_rom_MatchID		=	(unsigned char)0x55,			//
	sdq_rom_SkipID		=	(unsigned char)0xCC,			//
	sdq_rom_ReadID		=	(unsigned char)0x33,			//
	sdq_rom_SearchID	=	(unsigned char)0xF0,			//
}sdq_rom_cmd;
//------------------------------------------------------------------------------
typedef enum					//相对于主机
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
//------------------------------------------------------------------------------
typedef	struct	_sdq_time				//
{	
	unsigned char	 timeper;			//根据系统时钟计时的时间分频
	unsigned short count;				//计时时间
	unsigned long	 time1;				//开始前时间
	unsigned long  time2;				//新时间
	unsigned long  time_sys;		//总时间
	unsigned long  time_out;		//超时时间:开始接收数据后超时时间开始计时
}sdq_time_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_message			//
{	
	unsigned char				MessageLen;					//接收的摘要长度计数
	unsigned char 			MessageBuffer[20];	//消息缓存
	sdq_TransType_def		MessageTransType;		//当前消息传输类型
}bq26100slave_message_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_digest			//
{
	struct
	{
		unsigned char Auth				:1;		//0--未设置控制位/未启动转换上报主机发送的message；1--已设置控制位，可以匹配digest并准备上报
		unsigned char Digested		:1;		//0--未准备好digest,1-准备好digeset
		unsigned char match_error	:1;		//0--匹配正确,1-未找到匹配数据
	}Flag;	
	unsigned char 			Serial;							//已发送摘要计数
	unsigned char 			DigestBuffer[20];		//消息摘要
	sdq_TransType_def		DigestTransType;		//当前消息传输类型
}bq26100slave_digest_def;
//------------------------------------------------------------------------------
typedef	struct	_bq26100slave_ctrl			//
{	
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
	sdq_TransType_def		CtrlTransType;		//当前消息传输类型
}bq26100slave_ctrl_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_TransData			//位传输数据
{	
	unsigned char 	SendBitLen;				//发送位计数
	unsigned char 	SendByte;					//发送数据	
	unsigned char 	RcvBitlen;				//接收位计数
	unsigned char 	RcvByte;					//接收数据
}bq26100slave_TransData_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_PublicData			//
{	
	unsigned long 	intrrcout;		//中断次数计数
	unsigned long 	startcout;		//启动次数计数
	unsigned short 	receivelen;		//消息长度计数
	unsigned char 	receive[sdq_receive_buffer_size];	//消息数组
}bq26100slave_PublicData_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_InitData			//每条消息的起始标准格式
{	
	sdq_rom_cmd			RomCmd;			//第一字节:ROM命令
	sdq_mem_cmd			MemCmd;			//第二字节:RAM命令
	unsigned char 	addressL;		//第三字节:地址-低字节
	unsigned char 	addressH;		//第四字节:地址-高字节
	unsigned char 	InitFlag;		//0--未传输完初始化数据，1--传输完初始化数据
}bq26100slave_InitData_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq_MemData	//RAM命令数据
{	
	bq26100slave_message_def		Message;		//消息数据
	bq26100slave_digest_def			Digest;			//摘要数据
	bq26100slave_ctrl_def				Ctrl;				//控制命令数据
}bq26100slave_MemData_def;
//------------------------------------------------------------------------------
typedef	struct	_sdq				//带反馈锁控制标志及吸合时间
{
//	sdq_port_def 		port;	
	
	bq26100slave_TransData_def		TransData;			//正在传输的数据--按位接收或者发送
	bq26100slave_PublicData_def		PublicData;			//公共数据--临时数据
	bq26100slave_InitData_def			InitData;				//每条消息的起始标准格式数据	
	bq26100slave_MemData_def			MemData;				//RAM命令数据
	
	sdq_MasterStatus_def					MasterStatus;		//主机当前执行的状态
	sdq_time_def									time;
}bq26100slave_def;
//------------------------------------------------------------------------------

#define is_bq26100_rom_cmd(cmd)	(														\
																(sdq_rom_idle			==	cmd)	\
															||(sdq_rom_MatchID	==	cmd)	\
															||(sdq_rom_SkipID		==	cmd)	\
															||(sdq_rom_ReadID		==	cmd)	\
															||(sdq_rom_SearchID	==	cmd))

#define is_bq26100_ram_cmd(cmd)	(																		\
																(sdq_mem_idle			==	cmd)					\
															||(sdq_mem_read_memory	==	cmd)			\
															||(sdq_mem_write_memory		==	cmd)		\
															||(sdq_mem_read_page		==	cmd)			\
															||(sdq_mem_read_digest		==	cmd)		\
															||(sdq_mem_write_message		==	cmd)	\
															||(sdq_mem_read_status		==	cmd)		\
															||(sdq_mem_write_status		==	cmd)		\
															||(sdq_mem_read_control		==	cmd)		\
															||(sdq_mem_write_control		==	cmd)	\
															||(sdq_mem_read_eeprom		==	cmd)		\
															||(sdq_mem_profile		==	cmd)				\
															||(sdq_mem_read_page4		==	cmd)			\
															||(sdq_mem_write_page4	==	cmd))


void api_bq26100slave_configuration(bq26100slave_def *sSDQ);		//SDQ从机设备配置
void api_bq26100slave_server(void);		//SDQ从机服务

unsigned char* api_bq26100slave_get_sample_message_address(unsigned short serial);
unsigned char* api_bq26100slave_get_sample_digest_address(unsigned short serial);
unsigned short api_bq26100slave_get_sample_data_size(void);


unsigned char bq26100slave_process(void);

unsigned char bq26100slave_Initialize(void);						//BQ初始化：接收ROM命令、RAM命令和地址过程
unsigned char bq26100slave_command_process(void);				//根据初始化后的RAM命令执行相应的操作：数据接收或者读取:发送CRC，message原值、status、digest

unsigned char bq26100slave_MenCmd_InitializeData(void);		//根据命令对相应的数据进行初始化

unsigned char	bq26100slave_write_message(void);		//主机写消息
unsigned char	bq26100slave_read_control(void);		//主机读控制寄存器
unsigned char	bq26100slave_write_control(void);		//主机写控制寄存器
unsigned char bq26100slave_read_digest(void);			//主机读摘要


unsigned char api_bq26100slave_digest_match_error(void);
unsigned char api_bq26100slave_get_error_status(void);			//无错误返回0，有错误返回1
unsigned char api_bq26100slave_reset_error_status(void);

unsigned short api_bq26100slave_get_receivelen(void);		//返回接收长度


unsigned char bq26100slave_send_bit(unsigned char bit);
unsigned char bq26100slave_receive_bit(void);

unsigned char	bq26100slave_send_byte(void);			//返回0--未发送完一字节，返回1--发送完一字节
unsigned char	bq26100slave_receive_byte(void);	//返回0--未读完一字节，返回1--读完一字节，返回0xFF--复位信号

unsigned char	bq26100slave_start(void);
unsigned char	bq26100slave_start_data_init(void);
unsigned char api_bq26100slave_all_data_default(void);


void RecordData(unsigned char bytedata);




void bq26100slave_set_digest(void);

//unsigned char api_sdq_get_level(void);














#endif

