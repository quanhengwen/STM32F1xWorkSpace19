#ifndef __IOT5302W_H
#define __IOT5302W_H


#include "stm32f10x_type.h"
#include "STM32_USART.H"      //

//#include "font.h"

//#include	"stdio.h"			//用于printf
//#include	"string.h"			//用于printf
//#include	"stdarg.h"			//用于获取不确定个数的参数
//#include	"stdlib.h"			//malloc动态申请内存空间

/*
M1卡：总容量1Kbyte
16个扇区0~15
每个扇区4个块0~4
每个块16字节
*/
/********************************************************************************/
/*																数据包格式																			*/
/*------------------------------------------------------------------------------*/
/* 起始字节  从机地址  数据长度  命令/状态  数据n  BCC校验  结束码									*/
/* 起始码 		：0x0A																															*/
/* 从机地址 	：单机通讯用通用地址0x00																							*/
/* 数据长度 	：命令+数据的字节数，不包括起始码，BCC码和结束码												*/
/* 命令/状态	：主机下发命令，从机上传状态																					*/
/* BCC校验	：除起始码和结束码之外的所有数据异或校验																*/


#define IotReaderSTX  (unsigned char)0xAA   //协议头标识
#define IotReaderETX  (unsigned char)0xBB   //协议尾标志
  
#define IOT5302WReadCycle   300     //读周期，单位0.3秒，倒计时时间
#define IOT5302WReadTimeOut 2000    //读超时时间		//原1500
#define IOT5302WReadTimeOut_UID 	500    	//未接收到UID超时清除
#define IOT5302WReadTimeOut_NACK 	3000    //读卡器未响应超时时间3秒
#define IOT5302WInitializeTimeOut 2000    //读超时时间2秒	//原1500
#define IOT5302WBufferSize  100    //

//--------------------读卡器命令
typedef enum  _Cmd //STATUS
{
  //----------------------ISO14443 TYPE A Commands (0x03~0x06)
  REQA          = 0x03,   //ISO14443 -A寻卡请求
  Anticoll      = 0x04,   //防冲突
  Select        = 0x05,   //选定卡
  //----------------------Mifare Application Commands (0x20~0x2F)
  IotRead     	= 0x20, //集成寻卡，防冲突，选卡，验证密码，读卡等操作，一个命令完成读卡操作。
  IotWrite    	= 0x21, //集成寻卡，防冲突，选卡，验证密码，写卡等操作，一个命令完成写卡操作。.
  InitVal       = 0x22, //集成寻卡，防冲突，选卡，验证密码等操作，一个命令完成块值初始化操作。
  Decrement     = 0x23, //集成了寻卡，防冲突，选卡，验证密码，块值减操作，一个命令完成块减值操作。
  Increment     = 0x24, //集成了寻卡，防冲突，选卡，验证密码，块值加等操作，一个命令完成块值加操作。
  GET_SNR       = 0x25, //集成了寻卡，防冲突，选卡等操作，一个命令完成读取卡片序列号的操作
  ISO14443CC    = 0x28, //ISO14443 TypeA 通用命令，可以根据ISO14443 TypeA 向卡发任何数据
  //----------------------系统命令(0x80~0xFF)
  SetAddress    = 0x80, //设置读写器地址
  SetBaudrate   = 0x81, //设置通讯波特率
  SetSerlNum    = 0x82, //设置读写器的序列号
  GetSerlNum    = 0x83, //读取读写器的序列号
  WriteUserInfo = 0x84, //设置用户数据信息
  ReadUserInfo  = 0x85, //读取用户数据信息
  GetVersionNum = 0x86, //用来读取模块的版本信息
  ControlLed    = 0x88, //控制led的工作方式
  ControlBuzzer = 0x89, //控制buzzer的工作方式
}Cmddef;

//--------------------读卡器状态码
typedef enum  _status //STATUS
{
  CMDOK       = 0x00,          //表示命令执行成功
  CMDFAILED   = 0x01,   //表示命令操作失败（具体说明参见函数）
  SETOK       = 0x80,     //表示参数设置成功
  SETFAILED   = 0x81, //表示参数设置失败
  CMMTimeOut  = 0x82, //表示通讯超时.
  NoCard      = 0x83, //表示卡不存在
  DataErr     = 0x84, //表示接收卡数据出错
  PraErr      = 0x85, //表示输入参数或者输入命令格式错误
  UnknowErr   = 0x87, //表示未知的错误
  UnknowCmd   = 0x8F, //表示输入的指令代码不存在
}stsdef;
//--------------------读卡器数据结构体
typedef struct _IOTReader
{
  unsigned char STX;  //0xAA – ‘起始字节’ – 标准控制字节. 表示一个数据包的开始
  unsigned char ID;   //设备地址，在多机通讯所必需，读写器在收到数据包后判断包内的地址与自身预设地址是否相符，相符才会响应。地址0x00 是一个在单机模式下使用的特殊地址。读写器会响应任何带0地址的数据包(不进行地址判断).1
  unsigned char Len;  //数据包中数据字节的长度.包括CMD/STATUS 和DATA field,但不包括BCC.LENGTH= 字节数(CMD/STATUS + DATA[0.. N])
  unsigned char Cts;  //命令或者状态CMD/status
  unsigned char Data[128];  //数据0–123，这是一个长度与命令字有关的数据流。也有部分命令不需要附加数据.
  unsigned char BCC;  //8bits的校验字节.它包括除STX, ETX外所有字节的异或校验.
  unsigned char ETX;  //0xBB:’终止字节’ – 标准控制字节，表示数据包的结束.
}IRDdef;
//--------------------读卡器使用前需要配置的参数
typedef struct _IOT5302WConf
{
  RS485Def  IOT5302WPort;         //通讯接口
  unsigned long USART_BaudRate;   //实际应用需要的波特率
}IOT5302WConfWdef;
//--------------------读卡器数据结构体
typedef struct _IOT5302WData
{
  unsigned char   UID[4];         //卡片UID
  unsigned char   UIDbac[4];      //卡片UID备份，如果数据有变化，跟备份区不一致时，外发
  unsigned char   Data[IOT5302WBufferSize]; //读卡器数据接收缓存
  unsigned char   DataCount;      	//读卡器数据接收计数
  unsigned char   Initialized;    	//初始化标志，0-未初始化成功，1-初始化成功
  unsigned short  Time;           	//计时器
//  unsigned short  TimeOut;        	//超时计时器
	unsigned short  TimeOut_UID;  		//未接收到UID计时
	unsigned short  TimeOut_NACK;  		//读卡器无响应超时计时
	unsigned short  Time_Initialized;	//读卡器初始化时间间隔
	unsigned short  TimeGetData;  		//查询读卡器数据时间间隔计时
  unsigned short  TimeUIDCmp;   		//UID与UIDbac对比数据相同时，超过2秒，再上传数据
  unsigned long   USART_BaudRate; 	//正在使用的波特率
//  unsigned char   BaudRateSetStep; //设置波特率步骤
}IOT5302WDataWdef;
//--------------------读卡器数据结构体
typedef struct _IOT5302W
{
  IOT5302WConfWdef  Conf;   //读卡器使用前需要配置的参数
  IOT5302WDataWdef  Data;  
}IOT5302Wdef;






unsigned short IotMifareGetCmdFrame(Cmddef Cmd,unsigned char Sec,unsigned char* Buffer,unsigned char* KeyBuffer);




void api_iot5302w_configuration(IOT5302Wdef* pIOT5302W);
void api_iot5302w_server(void);

unsigned short api_get_iot5302w_uid(unsigned char* Buffer);		//获取UID

//---------------------------------------------------------hardware
static void hw_port_configuration(unsigned long USART_BaudRate);
static unsigned short iot5302w_send_msg(unsigned char* TxdBuffer,unsigned short length);
static unsigned short iot5302w_read_msg(unsigned char* RxdBuffer);

//---------------------------------------------------------software
static void iot5302w_initialize(void);
static void iot5302w_data_receive_process(void);	//数据接收处理，接收拼包，协议检查
static void iot5302w_data_process(void);		//数据协议检查
static void iot5302w_get_data(void);
static unsigned short iot5302w_GetFrame_GetSNR(unsigned char* Buffer);    		//生成获取UID的消息帧，获取UID
static unsigned short iot5302w_set_frame_SetBaudrate(unsigned char* Buffer,unsigned char Num);  //生成配置读卡器波特率的消息帧
#endif



