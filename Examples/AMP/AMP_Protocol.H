#ifndef __AMP_Protocol_H
#define __AMP_Protocol_H


#define headcode  (unsigned char)0x7E
#define endcode   (unsigned char)0x7F
	
#define minFrameSize 7   //最小帧长度---7应答帧

#define maxFramesize 256  		//5+128	//通讯最大帧字节数

#define DspMaxNameLen		64		//名字长度--字节
  
#define gDatasize       		maxmsgsize	//串口最大数据包容量
#define arrysize        		10       		//每个发送端口待发送缓存排序个数
#define MaxSlave        		30        	//最大从机数量(柜、层、位)
//#define ampReSendTime  			100 	     		//重发间隔时间单位ms--19200
//#define ampReSendTime  			15 	     		//重发间隔时间单位ms--115200
#define ampReSendTime  			5 	     		//重发间隔时间单位ms--256000
#define SendNopTime     		10  	     	//发送等待时间单位ms---应答等待时间
#define ampMaxResendCount  	5         	//最大发送次数
#define ampUnlockOuttime   	2000      	//开锁倒计时超时时间
#define reslockdelaytime   	200   			//检测到锁已打开后延时关闭驱动时间ms
#define checkswitchidtime 	2000    		//检查拨码开关时间间隔
#define GetOnlinetime   		5000      	//获取在线设备时间间隔

extern unsigned  char  ackupfarme[minFrameSize];
extern unsigned  char  ackdownfarme[minFrameSize];

//-----------------------------------------------------数据类
typedef enum  _ampPort
{
  NonPort     = 0,
  PcPort      = 1,  //PC通讯接口
  CabPort     = 2,  //柜通讯接口
  LayPort     = 3,  //层通讯接口
  CardPort    = 4   //读卡器通讯接口
}ampPortDef;
typedef enum _ampStype			//状态类型
{
  ampStypeNone      = 0,    //不支持的命令/空命令
  ampStypeLock      = 1,    //锁状态,0--
  ampStypeComm      = 2,    //连接状态
}ampStypedef;
typedef enum _ampLockSts
{
  AmpLockOpen       = 0,    //锁打开  
  AmpLockOn         = 1,    //锁关上
  AmpLockOpenFailed = 2,    //开锁失败
}ampLockStsdef;
typedef enum _amp_comm_sts
{
  AmpCommOK       = 0,    //连接正常 
  AmpCommTimeOut  = 1,    //连接超时/失败
}ampCommStsdef;



//-----------------------------------------------------通讯协议类
typedef enum _ampecmd
{
  ampCmdNone      	= (unsigned char)0x00,    	//不支持的命令/空命令
  ampCmdAck       	= (unsigned char)0x01,    	//应答类型，只带一个数据，不带地址，数据表示状态
  ampCmdLed       	= (unsigned char)0x02,    	//LED/LCD控制，带三个数据，带地址，0-表示关，1-表示开
  ampCmdLock      	= (unsigned char)0x03,    	//开锁命令，只带一个数据，带地址，0-表示读锁状态，1表示开锁
  ampCmdCard      	= (unsigned char)0x04,    	//读卡器，带地址，第一个数据为0表示读卡，为1表示写卡，后面跟随卡数据
  ampCmdPwr       	= (unsigned char)0x05,    	//层板供电控制
  ampCmdSta       	= (unsigned char)0x06,    	//状态：主要为上报状态
  ampCmdGetOnline 	= (unsigned char)0x07,    	//检查在线设备，接收到后应答表示在线
  ampCmdBKligth   	= (unsigned char)0x08,    	//背光灯控制
	ampCmdLcdData  		= (unsigned char)0x09,    	//LCD内容显示数据传输命令--物品名称及规格及显示参数
	ampCmdLcdConf  		= (unsigned char)0x0A,			//LCD配置命令，修改背景色命令,字体色,字体大小
	ampCmdLcdPwr  		= (unsigned char)0x0B,			//LCD电源供电控制指令
	ampCmdTimeSync		=	(unsigned char)0x0C,			//LCD显示同步时间
	ampCmdCheckSleve	=	(unsigned char)0x3F,			//主机查询从机数据命令
}ampecmddef;
typedef enum _ampedir
{
	ampDirUp  			= (unsigned char)0x80,			//上传命令
	ampDirDown  		= (unsigned char)0x00,			//下发命令
}ampedirdef;
typedef union _ampcmdv2
{
	unsigned char cmd;
	struct
	{
		unsigned char cmd	:6;
		unsigned char rv	:1;		//备用
		unsigned char dir	:1;		//方向：0向下级发或者应答，1向上级发或者应答
	}cmd_bit;	
}ampcmdv2def;
typedef struct _ampcmd
{
	unsigned char cmd:6;
	unsigned char rv	:1;		//备用
	unsigned char dir	:1;		//方向：0向下级发或者应答，1向上级发或者应答
}ampcmddef;
typedef struct _ampaddr
{
  unsigned char address1;
  unsigned char address2;		//层地址
  unsigned char address3;
}ampaddrdef;
typedef union _ampcrc16v2
{
	unsigned short crc16;
	struct
	{
		unsigned char crcl;
		unsigned char crch;
	}crc16_u8;
}ampcrc16v2def;
typedef struct _ampcrc16
{
  unsigned char crcl;
  unsigned char crch;
}ampcrc16def;
typedef struct _ampmsg
{
  unsigned char length;   //不包含此位
  ampcmddef 		cmd;
	ampaddrdef		addr;
	unsigned char data[maxFramesize-5];		//不包含head,leng,crcl,crch,end长度
}ampmsgdef;
typedef struct _ampphy
{
  unsigned char head;
  ampmsgdef    	msg;
  ampcrc16def 	crc16;
  unsigned char end;
}ampphydef;





//------------------------------------------------------------------头文件
ampphydef* api_get_frame(unsigned char* pbuffer,unsigned short len);
unsigned short api_set_frame(ampphydef* frame,ampecmddef cmd,unsigned char dir);




#endif



