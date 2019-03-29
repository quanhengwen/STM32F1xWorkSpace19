#ifndef __AMP01_SW_H
#define __AMP01_SW_H


#define AmpMinFrameSize 7   //最小帧长度---7应答帧

#define maxmsgsize 256  		//5+128	//通讯最大帧字节数
#define headcode  (unsigned char)0x7E
#define endcode   (unsigned char)0x7F
#define DspMaxNameLen		64		//名字长度--字节
  
#define gDatasize       		maxmsgsize	//串口最大数据包容量
#define arrysize        		10       		//每个发送端口待发送缓存排序个数
#define MaxSlave        		30        	//最大从机数量(柜、层、位)
#define ReSendWaitTime  		50 	      	//重发间隔时间单位ms
#define SendNopTime     		10  	      	//发送等待时间单位ms---应答等待时间
#define maxresendcount  		5         	//最大发送次数
#define unlockOuttime   		2000      	//开锁倒计时超时时间
#define reslockdelaytime   	200   			//检测到锁已打开后延时关闭驱动时间ms
#define checkswitchidtime 	2000    		//检查拨码开关时间间隔
#define GetOnlinetime   		5000      	//获取在线设备时间间隔



typedef enum  _CCPort
{
  NonPort     = 0,
  PcPort      = 1,  //PC通讯接口
  CabPort     = 2,  //柜通讯接口
  LayPort     = 3,  //层通讯接口
  CardPort    = 4   //读卡器通讯接口
}enCCPortDef;
//----------------------------------------------基础帧
typedef enum _euampcmd
{
  ampCmdNone      = 0x00,    	//不支持的命令/空命令
  ampCmdAck       = 0x01,    	//应答类型，只带一个数据，不带地址，数据表示状态
  ampCmdLed       = 0x02,    	//LED/LCD控制，带三个数据，带地址，0-表示关，1-表示开
  ampCmdLock      = 0x03,    	//开锁命令，只带一个数据，带地址，0-表示读锁状态，1表示开锁
  ampCmdCard      = 0x04,    	//读卡器，带地址，第一个数据为0表示读卡，为1表示写卡，后面跟随卡数据
  ampCmdPwr       = 0x05,    	//层板供电控制
  ampCmdSta       = 0x06,    	//状态：主要为上报状态
  ampCmdGetOnline = 0x07,    	//检查在线设备，接收到后应答表示在线
  ampCmdBKligth   = 0x08,    	//背光灯控制
	ampCmdLcdData  	= 0x09,    	//LCD内容显示数据传输命令--物品名称及规格及显示参数
	ampCmdLcdConf  	= 0x0A,			//LCD配置命令，修改背景色命令,字体色,字体大小
	ampCmdLcdPwr  	= 0x0B,			//LCD电源供电控制指令
}ampcmddef;
//typedef struct _ampcmd
//{
//  unsigned char cmd :6;   //CMD1~CMD63---低6位  
//  unsigned char rw  :1;   //0-write,1-read--高7位
//  unsigned char dir :1;   //0-down,1-up---高8位  
//}ampcmddef;
typedef struct _ampaddr
{
  unsigned char address1;
  unsigned char address2;		//层地址
  unsigned char address3;
}ampaddrdef;
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
	unsigned char data[256];
}ampmsgdef;
typedef struct _ampphy
{
  unsigned char head;
  ampmsgdef    	msg;
  ampcrc16def 	crc16;
  unsigned char end;
}ampphydef;











//----------------------------------------------基础帧
typedef enum _eSts
{
  AmpStsNone      = 0,    //不支持的命令/空命令
  AmpStsLock      = 1,    //锁状态,0--
  AmpStsComm      = 2,    //连接状态
}eStsdef;
typedef enum _eLockSts
{
  AmpLockOpen       = 0,    //锁打开  
  AmpLockOn         = 1,    //锁关上
  AmpLockOpenFailed = 2,    //开锁失败
}eLockStsdef;
typedef enum _eCommSts
{
  AmpCommOK       = 0,    //连接正常 
  AmpCommTimeOut  = 1,    //连接超时/失败
}eCommStsdef;




void api_pc_data_process(unsigned char* pbuffer,unsigned short len);
#endif
