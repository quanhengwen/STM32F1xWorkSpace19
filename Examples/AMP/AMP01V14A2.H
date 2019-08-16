#ifndef __AMP01V14A2_H
#define __AMP01V14A2_H

#include	"AMP_Protocol.H"

//#include	"AMP_PHY.H"

#define	ampAckSize    	10       		//每个发送端口待发送缓存排序个数
#define	ampArrySize    	20       		//每个发送端口待发送缓存排序个数

#define	ampledtime	40	//2000ms

//	起始符		长度		命令	地址1			地址2			地址3			数据0~N		BCC16校验	结束符
//	START		Length	CMD		Address1	Address2	Address3			

//typedef struct _ampdata       //拨码数据：
//{
//  unsigned char Cab_Addr;         	//柜地址
//  unsigned char MB_Flag;          	//主板标志://0--副柜，1--主柜
//}ampdatadef;
typedef struct _ampAck       //拨码数据：
{
  unsigned char head;     	//0x7E
  unsigned char length;   	//长度位不包含head,leng,crcl,crch,end长度	
	ampcmddef			cmd;				//命令/状态
	unsigned char address1;		//柜地址
  unsigned char address2;		//层地址
  unsigned char address3;		//位地址
	unsigned char status;			//状态-默认0
	unsigned char Crc16L;
	unsigned char Crc16H;
	unsigned char end;				//0x7F
	
	unsigned char flag;				//标志:0-无应答，1-需要上报应答
}ampAckDef;
//---------------------------------------------------------------------
typedef struct _ampCache       //拨码数据：
{
  unsigned char arry;           		//发送排序：0时为空，从第1个开始发送，发送完一个，排序减1，直到0，增加时，增加一个待发送缓存，新缓存排最后
  unsigned char size;           		//当前缓存有效数据个数(待发送字节数)
  unsigned char data[maxFramesize];	//缓存
}ampCachedef;
//---------------------------------------------------------------------
typedef struct _ampLight       //灯光标志
{
  struct
	{
		//unsigned char	LightStatus 	:1;	//灯状态:0-灭状态，1-亮状态
		unsigned char	SetLevel			:2;	//设置亮度等级,点亮根据亮度等级:0-不处理，1-最低亮度/灭灯，计时或者需要灭灯时请求，开门时LCD背光亮度；2-中等亮度，开门时柜体背光亮度；3-最高亮度，关门时需要亮灯时的亮度
		unsigned char	WorkLevel			:2;	//当前亮度等级,点亮根据亮度等级:0-不处理，1-最低亮度/灭灯，计时或者需要灭灯时请求，开门时LCD背光亮度；2-中等亮度，开门时柜体背光亮度；3-最高亮度，关门时需要亮灯时的亮度
	}flag;	
	unsigned long	Time;       	//点亮时间倒计时
}ampLightDef;
//---------------------------------------------------------------------
typedef struct _ampTims       //拨码数据：
{
//  unsigned short runningtime;     //运行计数器
  unsigned short swicthidtime;    //运行计数器
  unsigned short SYSLEDTime;      //LED运行计数器
  //unsigned short LockTime;        //锁计数器
  //unsigned short PcSendTime;      //运行计数器
  //unsigned short CabSendTime;     //运行计数器
	//unsigned short LaySendTime;     //运行计数器
  unsigned short CardSendTime;    //运行计数器
	unsigned short LayPowerOnTime;    //层控制接口上电时间
//	unsigned short BackLightTime;  	//运行计数器
}ampTimesdef;
//---------------------------------------------------------------------
typedef struct _ampdata       					//拨码数据：
{
  unsigned char Cab_Addr;         	//柜地址 6位拨码 345678
  unsigned char MB_Flag;          	//主板标志:	//0--副柜，1--主柜
	unsigned char Display_Flag;       //透明屏标志://0--普通/无屏，1--主柜透明屏
	unsigned char LayPowerOnFlag;   	//层板供电://0--未供电，1--已供电
}ampdatadef;
//---------------------------------------------------------------------
typedef struct _ampCommLed       				//通讯LED指示灯
{
	union
	{
		unsigned char code;
		struct
		{
			unsigned char pc_rx:1;		//0-输出低电平/亮灯，1-输出高电平/灭灯
			unsigned char pc_tx:1;
			
			unsigned char cb_rx:1;
			unsigned char cb_tx:1;
			
			unsigned char ly_rx:1;
			unsigned char ly_tx:1;
			
			unsigned char rf_rx:1;
			unsigned char rf_tx:1;
		}flag;
	}displaycode;
	unsigned char display;
  struct
	{
		unsigned char pc_rx:1;
		unsigned char pc_tx:1;
		
		unsigned char cb_rx:1;
		unsigned char cb_tx:1;
		
		unsigned char ly_rx:1;
		unsigned char ly_tx:1;
		
		unsigned char rf_rx:1;
		unsigned char rf_tx:1;
	}flag;
	struct
	{
		unsigned short pc_rx;
		unsigned short pc_tx;
		
		unsigned short cb_rx;
		unsigned short cb_tx;
		
		unsigned short ly_rx;
		unsigned short ly_tx;
		
		unsigned short rf_rx;
		unsigned short rf_tx;
	}time;		//闪烁次数
	unsigned char	 ClearFlag;		//0-正常显示，1-清除显示
	unsigned short FlashTime;		//刷新时间
}ampCommLedDef;
//---------------------------------------------------------------------
typedef struct _ampLock       					//锁
{
  struct
	{
		//unsigned char	unlock  		:1;	//开锁请求：0-无请求，1-有请求
		//unsigned char	relslock		:1;	//执行开锁：0-无，1-正在执行  
		//unsigned char	reslock     :1;	//释放锁驱动请求：0-无请求，1-有请求
		//unsigned char	reslockqust :1;	//释放锁请求，0-无请求，1-有请求，检测到锁状态为打开时，延时一段时间再关闭驱动
		unsigned char	LockStatus	:1;	//锁状态：0-开状态，1-锁状态
		unsigned char	LockUpdata	:1;	//锁更新：0-无更新，1-有状态更新
//		unsigned char	unlockerr  	:1;	//开锁错误/失败:0-无，1-开锁失败
	}flag;
	unsigned short LockTime;        //锁计数器
}ampLockDef;
//---------------------------------------------------------------------
typedef struct _ampMagneticLock       	//磁吸锁
{
  struct
	{
		//unsigned char	unlock  		:1;	//开锁请求：0-无请求，1-有请求
		//unsigned char	relslock		:1;	//执行开锁：0-无，1-正在执行  
		//unsigned char	reslock     :1;	//释放锁驱动请求：0-无请求，1-有请求
		//unsigned char	reslockqust :1;	//释放锁请求，0-无请求，1-有请求，检测到锁状态为打开时，延时一段时间再关闭驱动
		unsigned char	LockStatus	:1;	//锁状态：0-开状态，1-锁状态
		unsigned char	LockUpdata	:1;	//锁更新：0-无更新，1-有状态更新
//		unsigned char	unlockerr  	:1;	//开锁错误/失败:0-无，1-开锁失败
	}flag;
	unsigned short LockTime;        //锁计数器
}ampMagneticLockDef;

//---------------------------------------------------------------------
typedef struct _ampCommPort        			//通讯端口
{
	unsigned short 	SendWaitTime;      		//发送等待时间--为0时，发送数据
	unsigned char 	SlaveSendEnable;    	//从机允许发送标志:0-等待，1-允许上传
	unsigned char 	WaitAckFlag;      		//等待应答标志:0--无需等待应答，1--需等待应答	
	unsigned char 	ReSendCount;      		//上传重发计数
	unsigned char		GetSlaveAddress;			//当前查询的从机地址
	ampAckDef				AckFrame;							//应答帧
  ampCachedef			Buffer[ampArrySize];	//缓存
}ampCommPortDef;
//---------------------------------------------------------------------
//=====================================================================
typedef struct _amp
{
//	ampReqdef				request;
//	ampStadef     	status;    		//状态码
//	ampWaitAckdef		AckW;					//等待应答标志
//  ampResenddef    ReSend;	
//	ampCCdef				commdata;			//通讯数据
	ampdatadef					sysdata;			//常用数据和标识
	ampTimesdef					time;	
	
	ampCommLedDef				CommLed;			//通讯LED指示灯	
	
	ampLockDef					Lock;					//锁相关	
	ampMagneticLockDef	MagneticLock;	//磁吸锁
	
	ampLightDef					BackLight;		//柜体背光
	ampLightDef					LcdLight;			//LCD背光
	
	ampCommPortDef			Pc;						//PC端口数据
	ampCommPortDef			Cab;					//柜端口数据
	ampCommPortDef			Lay;					//层接口数据
}ampdef;






void API_AMP01V14A2_Configuration(void);
void API_AMP01V14A2_Server(void);
static void AMP01V14A2_HIGH_Process(void);
static void AMP01V14A2_LOW_Process(void);


static void SysLed_server(void);
//--------------------------------static-cache_buffer
static ampCachedef* get_cache_addr(ampPortDef port);
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame);
static ampCachedef* get_cache_data(ampPortDef port);
static unsigned char del_cache_data(ampPortDef port);
//--------------------------------static-ack_flag
static unsigned char get_ack_wait_flag(ampPortDef port);
static void set_ack_wait_flag(ampPortDef port);
//--------------------------------static-port_server
static void Pc_Server(void);
static void Cab_Server(void);
static void Lay_Server(void);
static void CardReader_Server(void);
//--------------------------------static-port_data_process
static void pc_receive_data_process(void);
static void pc_send_data_process(void);
static void cab_receive_data_process(void);
static void cab_send_data_process(void);
static void lay_receive_data_process(void);
static void lay_send_data_process(void);
static void cmd_process(ampphydef* frame);
//--------------------------------static-hardware_control
static void set_door(unsigned char flag);
//static void set_backlight(unsigned char flag);
//static void set_lcdlight(unsigned char flag);
static void set_layer_power(unsigned char flag);
//--------------------------------static-set_frame
static void set_ackup_frame(ampPortDef Port);
static void set_ackdown_frame(ampPortDef Port);
static void set_lock_status_frame(void);
static void set_comm_time_out_frame(ampPortDef Port,ampphydef* frame);
//--------------------------------static-Server
static void SwitchID_Server(void);
static void Lock_server(void);
static void Magnetic_Lock_server(void);		//磁吸锁
static void CommLed_Server(void);
static void BackLight_Server(void);
static void LcdLight_Server(void);
static void Tim_Server(void);
//--------------------------------static-configuration
static void Hardware_Configuration(void);
static void Communication_Configuration(void);
static void SwitchID_Configuration(void);
static void BackLight_Configuration(void);
static void LayPower_Configuration(void);
static void Lock_Configuration(void);
static void Led_Configuration(void);
//--------------------------------static-test
static void test_cabinet_address_none_process(void);		//未拨码柜接口发LCD测试程序
static void set_time_sync_frame(void);		//未拨码柜接口发LCD测试程序

#endif
