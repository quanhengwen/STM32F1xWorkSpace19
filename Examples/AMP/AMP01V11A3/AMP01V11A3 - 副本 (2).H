#ifndef __AMP01V11A3_H
#define __AMP01V11A3_H

#include	"AMP_Protocol.H"

//#include	"AMP_PHY.H"

#define 	ampArrySize    	30       		//每个发送端口待发送缓存排序个数


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
typedef struct _ampLock       //通讯缓存
{
  unsigned char que:1;		//0--无请求，1--开锁请求
	unsigned char sta:1;		//0--锁为开状态，1--锁住状态
}amplockdef;
//typedef struct _ampcrc16
//{
//  unsigned char crcl;
//  unsigned char crch;
//}ampcrc16def;

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
typedef struct _ampStatus        //状态标志：
{
  unsigned short lockstd    :1;  //锁状态：0-开状态，1-锁状态
  unsigned short unlockerr  :1;  //开锁错误/失败:0-无，1-开锁失败
  unsigned short Online     :1;  //设备掉线:0-掉线，1-在线
}ampStadef;
typedef struct _amp
{
//  stTimesdef        Time;         //相关时间变量
//  stSWdef           SwData;       //主机标志
//  stReqdef          Req;         //请求类标志
  
//  stFlgdef          Flag;         //相关标志位
//  stAckQdef         AckQ;         //标志应答标志，0-不需要，1-需要应答相应接口

	ampStadef     	Sta;          //状态码
	ampWaitAckdef		AckW;				//等待应答标志
  ampResenddef    ReSend;	
	ampCCdef				CCdata;			//通讯数据
	ampdatadef			sysdata;		//常用数据和标识
	amplockdef			lock;				//锁相关
	ampTimesdef			time;
}ampdef;


//extern stAMPProdef   AMPPro;

void AMP01V11A3_Configuration(void);
void AMP01V11A3_Server(void);




static ampCachedef* get_cache_addr(ampPortDef port);
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame);
static ampCachedef* get_cache_data(ampPortDef port);
static unsigned char del_cache_data(ampPortDef port);


static void Hardware_Configuration(void);
static void Communication_Configuration(void);
static void SwitchID_Configuration(void);
static void BackLight_Configuration(void);
static void LayPower_Configuration(void);
static void Lock_Configuration(void);
static void Led_Configuration(void);





void Tim_Server(void);


#endif
