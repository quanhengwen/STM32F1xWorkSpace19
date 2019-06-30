#ifndef __AMP01V11A3_H
#define __AMP01V11A3_H

#include	"AMP_Protocol.H"

//#include	"AMP_PHY.H"

#define	ampArrySize    	30       		//每个发送端口待发送缓存排序个数

#define	ampledtime	40	//2000ms
//typedef struct _ampdata       //拨码数据：
//{
//  unsigned char Cab_Addr;         	//柜地址
//  unsigned char MB_Flag;          	//主板标志://0--副柜，1--主柜
//}ampdatadef;
typedef struct _ampCache       //拨码数据：
{
  unsigned char arry;           		//发送排序：0时为空，从第1个开始发送，发送完一个，排序减1，直到0，增加时，增加一个待发送缓存，新缓存排最后
  unsigned char size;           		//当前缓存有效数据个数(待发送字节数)
  unsigned char data[maxFramesize];	//缓存
}ampCachedef;
typedef struct _ampCC       		//通讯缓存
{
	ampCachedef	PcAck;						//应答
	ampCachedef	CbAck;						//应答
	ampCachedef	LyAck;						//应答
  ampCachedef	pc[ampArrySize];
	ampCachedef	cab[ampArrySize];
	ampCachedef	lay[ampArrySize];
}ampCCdef;

typedef struct _ampRequest
{
  unsigned short  unlock  		:1;//开锁请求：0-无请求，1-有请求
  unsigned short  relslock		:1;//执行开锁：0-无，1-正在执行  
  unsigned short  reslock     :1;//释放锁驱动请求：0-无请求，1-有请求
  unsigned short  reslockqust :1;//释放锁请求，0-无请求，1-有请求，检测到锁状态为打开时，延时一段时间再关闭驱动
  unsigned short  BLon        :1;//开背光请求：0-无请求，1-有请求
  unsigned short  BLoff       :1;//关背光请求：0-无请求，1-有请求
  unsigned short  PLon        :1;//层板供电开请求：0-无请求，1-有请求
  unsigned short  PLoff       :1;//层板供电关请求：0-无请求，1-有请求
}ampReqdef;

typedef struct _ampWaitAck       //通讯缓存
{
  unsigned char PC:1;			//0--无需等待应答，1--需等待应答
	unsigned char CB:1;			//0--无需等待应答，1--需等待应答
	unsigned char LY:1;			//0--无需等待应答，1--需等待应答
	unsigned char RF:1;			//0--无需等待应答，1--需等待应答
}ampWaitAckdef;

typedef struct _ampTims       //拨码数据：
{
  unsigned short runningtime;     //运行计数器
  unsigned short swicthidtime;    //运行计数器
  unsigned short SYSLEDTime;      //LED运行计数器
  unsigned short LockTime;        //LED运行计数器
  unsigned short PcSendTime;      //运行计数器
  unsigned short CabSendTime;     //运行计数器
  unsigned short LaySendTime;     //运行计数器
  unsigned short CardSendTime;    //运行计数器 
	unsigned short BackLightTime;  	//运行计数器
}ampTimesdef;

typedef struct _ampResend   //重发计数器
{
  unsigned char Pc;      //PC上传重发计数
  unsigned char Cab;     //柜接口重发计数
  unsigned char Lay;     //层接口重发计数
  unsigned char Card;    //读卡器接口重发计数
}ampResenddef;

typedef struct _ampdata       //拨码数据：
{
  unsigned char Cab_Addr;         	//柜地址
  unsigned char MB_Flag;          	//主板标志://0--副柜，1--主柜
}ampdatadef;
typedef struct _ampcommled       //拨码数据：
{
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
	}time;
}ampcommleddef;
typedef struct _ampStatus        //状态标志：
{
  unsigned short lock    		:1;  //锁状态：0-开状态，1-锁状态
  unsigned short unlockerr  :1;  //开锁错误/失败:0-无，1-开锁失败
  unsigned short Online     :1;  //设备掉线:0-掉线，1-在线
}ampStadef;
typedef struct _amp
{
	ampReqdef				request;
	ampStadef     	status;          //状态码
	ampWaitAckdef		AckW;				//等待应答标志
  ampResenddef    ReSend;	
	ampCCdef				commdata;			//通讯数据
	ampdatadef			sysdata;		//常用数据和标识
	ampTimesdef			time;
	ampcommleddef		led;
}ampdef;


//extern stAMPProdef   AMPPro;

void AMP01V11A3_Configuration(void);
void AMP01V11A3_Server(void);
static void status_server(void);
//--------------------------------static-cache_buffer
static ampCachedef* get_cache_addr(ampPortDef port);
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame);
static ampCachedef* get_cache_data(ampPortDef port);
static unsigned char del_cache_data(ampPortDef port);
//--------------------------------static-ack_flag
static unsigned char get_ack_wait_flag(ampPortDef port);
static void set_ack_wait_flag(ampPortDef port);
static void del_ack_wait_flag(ampPortDef port);
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
static void set_backlight(unsigned char flag);
static void set_layer_power(unsigned char flag);
//--------------------------------static-set_frame
static void set_ackup_frame(ampPortDef Port);
static void set_ackdown_frame(ampPortDef Port);
static void set_lock_status_frame(void);
static void set_comm_time_out_frame(ampPortDef Port,ampphydef* frame);
//--------------------------------static-Server
static void SwitchID_Server(void);
static void door_server(void);
static void Led_Server(void);
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
