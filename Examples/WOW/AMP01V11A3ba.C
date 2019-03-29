#ifdef AMP01V11A3

#include "AMP01V11A3.H"


#include "AMP_LAY.H"
#include "AMP_CABV11A2.H"

#include	"AMP_PHY.H"

#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"

#include "SWITCHID.H"
#include "IOT5302W.H"     //读卡器

#include 	"CRC.H"

#include "string.h"				//串和内存操作函数头文件

//------------------锁接口J10
#define ampSYSLEDPort    GPIOA         //锁控制接口，高电平开锁
#define ampSYSLEDPin     GPIO_Pin_0
//------------------通讯接口--PC
#define ampCommPcPort        USART2
//------------------通讯接口--柜口
#define ampCommCbPort       USART1
#define ampCommCbCTLPort    GPIOA         //锁控制接口，高电平开锁
#define ampCommCbCTLPin     GPIO_Pin_8
//------------------通讯接口--层板接口
#define ampCommLayPort       UART4
#define ampCommLayCTLPort    GPIOA         //锁控制接口，高电平开锁
#define ampCommLayCTLPin     GPIO_Pin_12
//------------------通讯接口--读卡器接口
#define ampCommCardPort      USART3
#define ampCommCardCTLPort   GPIOC         //锁控制接口，高电平开锁
#define ampCommCardCTLPin    GPIO_Pin_8
#define ampCommCardBaudRate  19200         //读卡器通讯波特率

//------------------锁接口J10
#define ampLockDrPort    GPIOB         //锁控制接口，高电平开锁
#define ampLockDrPin     GPIO_Pin_1
#define ampLockSiPort    GPIOC         //锁信号，检测到低电平表示锁已开
#define ampLockSiPin     GPIO_Pin_12
#define ampUnLock        ampLockDrPort->BSRR  = ampLockDrPin    //开锁
#define ampResLock       ampLockDrPort->BRR   = ampLockDrPin    //释放锁驱动
#define ampGetLockSts    (ampLockSiPort->IDR & ampLockSiPin)      //获取锁的状态，如果为0，表示锁已开
//------------------背光接口J11的VCC和EN脚
#define ampBackLightPort GPIOB         //高电平关闭，低电平点亮
#define ampBackLightPin  GPIO_Pin_0
#define ampBakkLightOff	(ampBackLightPort->BRR 	= ampBackLightPin)
#define ampBakkLightOn	(ampBackLightPort->BSRR = ampBackLightPin)

//------------------层板电源控制J5、J6、J9共用一个控制电源
#define ampLayPowerPort  GPIOB         //高电启动电源，低电平关闭电源
#define ampLayPowerPin   GPIO_Pin_2
#define ampLayPowerOn    ampLayPowerPort->BSRR = ampLayPowerPin   //启动电源
#define ampLayPowerOff   ampLayPowerPort->BRR  = ampLayPowerPin   //关闭电源

#define ampReSendTime					100		//ms
#define ampUnlockOuttime   		2000      	//开锁倒计时超时时间
#define ampMaxResendCount  		5         	//最大发送次数
/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Ly;   //uart4,PA15   //层板接口
static RS485Def ampRS485Cb;   //usart1,PA8    //副柜接口
//static RS485Def ampRS485Card; //usart3,PB2    //读卡器接口
static SwitchDef ampSwitchID;
static SPIDef stLed;

ampdef ampsys;

//static unsigned char CabAddr   =0;
//static unsigned char MainFlag  =0; //0--副柜，1--主柜
//unsigned char rxx[512]={0};
//unsigned char txx[]="Communication_Configuration";
//unsigned short timee=0;
//tAMPProdef   AMPPro;
/* Private function prototypes -----------------------------------------------*/
static void Hardware_Configuration(void);
static void GenyGPIO_Configuration(void);
static void Communication_Configuration(void);
//-----------------------------背光相关程序
static void BackLight_Configuration(void);
//static void BackLight_Server(void);
//static void Set_BackLight_On(void);
//static void Set_BackLight_Off(void);
//static unsigned char Get_BackLight_State(void);
//-----------------------------层板电源控制
static void LayPower_Configuration(void);
//-----------------------------锁相关程序
static void Lock_Configuration(void);
static void Lock_Server(void);
static void Set_Lock_Open(void);
static void Set_Lock_Release(void);
//-----------------------------拨码开关相关程序
static void SwitchID_Configuration(void);
static void SwitchID_Server(void);
//-----------------------------通讯LED指示灯
static void Led_Configuration(void);
static void Led_Server(void);
//-----------------------------
static void Pc_Server(void);

//-----------------------------
static void Cab_Server(void);
//-----------------------------
static void Lay_Server(void);
//-----------------------------
static void Pc_Data_Process(unsigned char* pBuffer,unsigned short len);
static void Cab_Data_Process(unsigned char* pBuffer,unsigned short len);
//-----------------------------
static void Cmd_Process(stampphydef* frame);
//-----------------------------
static void amp_ack_up(enCCPortDef port);
static void amp_ack_down(enCCPortDef port);
static char is_ack_frame(unsigned char* pBuffer,unsigned short len);

static ampCachedef* AmpData_Cache_GetAddr(enCCPortDef port);
static unsigned short AmpData_Cache_Add(enCCPortDef port,stampphydef* frame);
static ampCachedef* AmpData_Cache_Get(enCCPortDef port);
static unsigned char AmpData_Cache_Releas(enCCPortDef port);
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AMP01V11A3_Configuration(void)
{	
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	
	Hardware_Configuration();

  GPIO_Configuration_OPP50(ampSYSLEDPort,ampSYSLEDPin);

	//SysTick_DeleymS(500);				//SysTick延时nmS
  PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM设定-20161127版本	占空比1/1000
	
  IWDG_Configuration(2000);			//独立看门狗配置---参数单位ms
  
  SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
  
  while(1)
  {

  }
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AMP01V11A3_Server(void)
{ 
  unsigned char rxnum;
	IWDG_Feed();								//独立看门狗喂狗
	Tim_Server();
  Lock_Server();
  SwitchID_Server();
  Led_Server();
	Pc_Server();
	Cab_Server();
	Lay_Server();
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static ampCachedef* AmpData_Cache_GetAddr(enCCPortDef port)
{
	if(NonPort	==	port)
		return 0;
	else if(PcPort	==	port)
		return ampsys.CCdata.pc;
	else if(CabPort	==	port)
		return ampsys.CCdata.cab;
	else if(LayPort	==	port)
		return ampsys.CCdata.lay;
	else
		return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned short AmpData_Cache_Add(enCCPortDef port,stampphydef* frame)
{
	unsigned short	i=0;
	unsigned short  lastarry  = 0;
	unsigned short	len	=	0;
	unsigned char*	pBuffer	=	(unsigned char*)frame;
	ampCachedef* Cache	= AmpData_Cache_GetAddr(port);

	if(0==Cache)	//返回空地址
		return 0;
	//-------------------------检查数据有无溢出
	len	=	frame->msg.length+5;	//head,len,crcl.crch.end
	if(AmpFramesize<len)
		return 0;
	//-------------------------检查缓存中有无相同数据及查找当前最大缓存序号
	for(i=0;i<AmpArrySize;i++)
	{
		if(Cache[i].arry>lastarry)		//缓存数据不为0
		{
			lastarry	=	Cache[i].arry;	//当前最大的序号	
			if(len	==	Cache[i].size)
			{
				if(0  ==  memcmp(Cache[i].data,pBuffer,len)) //比较相同
				{
					return len;		//已有相同数据，退出，表示存储成功
				}
			}
		}
	}
	//-------------------------未在缓存中找到相同的数据
	//-------------------------判断缓存是否为满
	if(lastarry>=AmpArrySize)	//从0开始
	{
		//缓存满
		return 0;
	}
	lastarry+=1;
	for(i=0;i<AmpArrySize;i++)
  {
    if(0  ==  Cache[i].arry)  //0编号表示此为空缓存
    {
      memcpy(Cache[i].data,pBuffer,len);
      Cache[i].arry = lastarry;             //此缓存在发送队列中的排序
      Cache[i].size = len; 
      return  len;
    }
  }
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static ampCachedef* AmpData_Cache_Get(enCCPortDef port)
{
	unsigned short	i=0;
	unsigned short	arry=AmpFramesize;
	unsigned short  lastarry  = 0;
	ampCachedef* Cache	= AmpData_Cache_GetAddr(port);
	
	if(0==Cache)	//返回空地址
		return 0;
	//-------------------------先发送缓存标号为1号数据
	for(i=0;i<AmpArrySize;i++)
	{
		
		if(1==Cache[i].arry)		//缓存数据不为0
		{
			return &Cache[i];
		}
	}
	return 0;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char AmpData_Cache_Releas(enCCPortDef port)
{
	unsigned short	i=0;
	unsigned short	arry=AmpFramesize;
	unsigned short  lastarry  = 0;
	ampCachedef* Cache	= AmpData_Cache_GetAddr(port);
	
	if(0==Cache)	//返回空地址
		return 0;
	//-------------------------先发送缓存标号为1号数据
	for(i=0;i<AmpArrySize;i++)
	{
		if((Cache[i].arry))
		{
			Cache[i].arry-=1;
		}
	}
	return 1;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Pc_Server(void)
{
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
	unsigned short rxnum=0;
	unsigned char rxd[256];
	stampphydef* frame=0;
	//================================================数据接收
	rxnum	=	API_USART_ReadBufferIDLE(ampCommPcPort,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{
		char ackflag=is_ack_frame(rxd,rxnum);	//1:向上应答，-1：向下应答；0:非应答
		if((-1==ackflag)&&(ampsys.AckW.PC))		
		{
			ampsys.AckW.PC		=	0;
			ampsys.ReSend.Pc	=	0;
			ampsys.time.PcSendTime	=	0;
			AmpData_Cache_Releas(PcPort);
			return;
		}
		else if(0==ackflag)
			Pc_Data_Process(rxd,rxnum);
		return;
	}
	//================================================数据发送
	if(0==ampsys.time.PcSendTime)
	{
		unsigned short sendnum=0;
		ampCachedef*	Cache;
		//----------------------------------------------检查有无应答数据要发送
		if(ampsys.CCdata.PcAck.size)
		{
			Cache	=	&ampsys.CCdata.PcAck;
			sendnum	=	API_USART_DMA_Send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
			if(sendnum)	//已将数据转移到缓存
			{	
				ampsys.CCdata.PcAck.size	=	0;	//0--无需等待应答，1--需等待应答
				ampsys.time.PcSendTime	=	ampReSendTime;
			}
			return;
		}		
		//----------------------------------------------发送数据
		if(ampsys.ReSend.Pc>ampMaxResendCount)
		{
			ampsys.ReSend.Pc	=	ampMaxResendCount;
			AmpData_Cache_Releas(PcPort);
		}
		Cache	=	AmpData_Cache_Get(PcPort);
		if(0==Cache)	//无数据
		{
			ampsys.ReSend.Pc	=	0;
			return;
		}
		sendnum	=	API_USART_DMA_Send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.AckW.PC	=	1;	//0--无需等待应答，1--需等待应答
			ampsys.ReSend.Pc++;			
			ampsys.time.PcSendTime	=	ampReSendTime;
		}		
	}
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Cab_Server(void)
{
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
	unsigned short 	rxnum=0;
	unsigned char 	rxd[256];
	stampphydef* 		frame=0;
	//================================================数据接收
	rxnum	=	RS485_ReadBufferIDLE(&ampRS485Cb,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{
		char ackflag=is_ack_frame(rxd,rxnum);	//1:向上应答，-1：向下应答；0:非应答
		if(0!=ackflag)	//有应答
		{
			if(ampsys.AckW.CB)
			{
				if(((1==ampsys.sysdata.MB_Flag)&&(1==ackflag))			//主柜
					||((0==ampsys.sysdata.MB_Flag)&&(-1==ackflag)))		//副柜
				{
					ampsys.AckW.CB	=	0;
					ampsys.ReSend.Pc	=	0;
					ampsys.time.PcSendTime	=	0;
					AmpData_Cache_Releas(CabPort);
				}
			}
			return;
		}
		Cab_Data_Process(rxd,rxnum);
		return;
	}
	//================================================数据发送
	
	if(0==ampsys.time.CabSendTime)
	{
		unsigned short sendnum=0;
		ampCachedef*	Cache;
		//----------------------------------------------检查有无应答数据要发送
		if(ampsys.CCdata.CbAck.size)
		{
			Cache	=	&ampsys.CCdata.CbAck;
			sendnum	=	API_USART_DMA_Send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
			if(sendnum)	//已将数据转移到缓存
			{	
				ampsys.CCdata.CbAck.size	=	0;	//0--无需等待应答，1--需等待应答
				ampsys.time.PcSendTime		=	ampReSendTime;
			}
			return;
		}
		//----------------------------------------------发送数据
		if(ampsys.ReSend.Cab>ampMaxResendCount)
		{
			ampsys.ReSend.Cab	=	0;
			AmpData_Cache_Releas(CabPort);
		}
		Cache	=	AmpData_Cache_Get(CabPort);
		if(0==Cache)	//无数据
		{
			ampsys.ReSend.Cab	=	0;
			return;
		}
		sendnum	=	RS485_DMASend(&ampRS485Cb,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.AckW.CB	=	1;
			ampsys.ReSend.Cab++;			
			ampsys.time.CabSendTime	=	ampReSendTime;
		}		
	}
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lay_Server(void)
{
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
//	static unsigned short sendtime=0;
	unsigned short rxnum=0;
	unsigned char rxd[256];
	stampphydef* frame=0;
	//================================================数据接收
	rxnum	=	RS485_ReadBufferIDLE(&ampRS485Ly,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{
		char ackflag=is_ack_frame(rxd,rxnum);	//1:向上应答，-1：向下应答；0:非应答
		if((0!=ackflag)&&(ampsys.AckW.LY))
		{
			if((1==ackflag)&&(ampsys.AckW.LY))
			{
				ampsys.AckW.LY	=	0;
				ampsys.ReSend.Lay	=	0;
				AmpData_Cache_Releas(LayPort);
			}
			return;
		}			
	}
	//================================================数据发送
	if(0==ampsys.time.LaySendTime)
	{
		unsigned short sendnum=0;
		ampCachedef*	Cache;
		if(ampsys.ReSend.Lay>ampMaxResendCount)
		{
			ampsys.ReSend.Lay	=	0;
			AmpData_Cache_Releas(LayPort);
		}
		Cache	=	AmpData_Cache_Get(LayPort);
		if(0==Cache)	//无数据
		{
			ampsys.ReSend.Lay	=	0;
			return;
		}
		sendnum	=	RS485_DMASend(&ampRS485Ly,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.AckW.LY=1;
			ampsys.ReSend.Lay++;			
			ampsys.time.LaySendTime	=	ampReSendTime;
		}		
	}
}
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void Pc_Data_Process(unsigned char* pBuffer,unsigned short len)
{  
	unsigned char 	cab_addr	=	0;	//柜地址
	unsigned char 	lay_addr	=	0;	//层地址
	unsigned short	Saved_len	=	0;
	stampphydef* frame	=	(stampphydef*)API_Get_AmpFrame(pBuffer,len);
	if(0==frame)
		return;
	
	cab_addr	=	frame->msg.addr.address1;	//柜地址
	lay_addr	=	frame->msg.addr.address2;	//层地址
	//------------------------------PC应答
//	if(AmpCmdAck==frame->msg.cmd.cmd)		//应答，删除一缓存
//	{
//		ampsys.ReSend.Pc	=	0;
//		AmpData_Cache_Releas(PcPort);
//	}
	
	//------------------------------PC数据
	if(0xFF==cab_addr)	//广播数据
	{
		Saved_len	=	 AmpData_Cache_Add(CabPort,frame);			//发往其它柜
		if(0!=lay_addr)
			Saved_len	=	 AmpData_Cache_Add(LayPort,frame);		//发往本柜层板
	}
	else if(cab_addr	==	ampsys.sysdata.Cab_Addr)	//本柜数据
	{
		if(0!=lay_addr)
			Saved_len	=	 AmpData_Cache_Add(LayPort,frame);		//发往本柜层板
	}
	else
	{
		Saved_len	=	 AmpData_Cache_Add(CabPort,frame);			//发往其它柜
	}
	if(0==Saved_len)	//缓存满
	{
		return;
	}
	Cmd_Process(frame);			//命令处理
	amp_ack_up(PcPort);
}
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void Cab_Data_Process(unsigned char* pBuffer,unsigned short len)
{  
	unsigned char 	cab_addr	=	0;	//柜地址
	unsigned char 	lay_addr	=	0;	//层地址
	unsigned short	Saved_len	=	0;
	stampphydef* frame	=	(stampphydef*)API_Get_AmpFrame(pBuffer,len);
	if(0==frame)
		return;
	
	cab_addr	=	frame->msg.addr.address1;	//柜地址
	lay_addr	=	frame->msg.addr.address2;	//层地址
	//------------------------------PC应答
	if(AmpCmdAck==frame->msg.cmd.cmd)		//应答，删除一缓存
	{
		ampsys.ReSend.Pc	=	0;
		AmpData_Cache_Releas(CabPort);
	}
	//------------------------------上报的数据
	if(0!=frame->msg.cmd.dir)
	{
		Saved_len	=	 AmpData_Cache_Add(PcPort,frame);		//发往本柜层板
		if(0!=ampsys.sysdata.MB_Flag)	//此为主板，需要向下应答
		{
			//应答
			return;
		}
	}
	//------------------------------数据处理
	if((0xFF==cab_addr)||(cab_addr==ampsys.sysdata.Cab_Addr))	//广播数据/本柜数据
	{
		if(0!=lay_addr)
			Saved_len	=	 AmpData_Cache_Add(LayPort,frame);		//发往本柜层板
	}
	if(0==Saved_len)	//缓存满
	{
		return;
	}
	Cmd_Process(frame);			//命令处理
}
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void Cmd_Process(stampphydef* frame)
{  
  unsigned  char result       = 0; 
  unsigned  char address      = 0;  
  unsigned  short framlength  = 0; 
  unsigned  char  Cmd         = 0;  
////  unsigned  char* paddrbac    = pBuffer;         //备份数据缓存起始地址
  
//  stampphydef* ampframe=NULL;
  //-------------------------检查端口是否为层接口及缓存地址是否为空
  if(NULL==frame)
		return;
	//-------------------------
	Cmd	=	frame->msg.cmd.cmd;

  //2)-----------------层地址为0，不需要往下发，只在柜控制板处理：灯控制/锁控制/供电
  if((AmpCmdLcdConf == Cmd)||(AmpCmdLcdData==Cmd))       //LCD配置命令
  {
    ampLayPowerOn;    //层板供电
  }
	else if((AmpCmdLed==Cmd)||(AmpCmdLcdPwr==Cmd)||(AmpCmdPwr==Cmd))  //LED电源供电控制指令
  {
    if(frame->msg.data[0])
		{
      ampLayPowerOn;    //层板供电
			ampBakkLightOn;
		}
    else
		{
      ampLayPowerOff;    //层板断电
			ampBakkLightOff;
		}			
  }
  else if(AmpCmdLock ==  Cmd)   //锁控制命令
  {
    if(frame->msg.data[0])	//data0不为0表示要开锁
      Set_Lock_Open();
  }
  else if(AmpCmdBKligth ==  Cmd)   //背光控制指令
  {
    if(frame->msg.data[0])	//data0不为0表示要开灯，否则关灯
      ampBakkLightOn;
    else
      ampBakkLightOff;    
  }
  else if(AmpCmdSta ==  Cmd)   //查询状态
  {
    if(0==AMPPro.Sta.lockstd) //开状态
    {
      LockStatusUpdata(AmpCmdSta,AmpLockOpen);  //锁---开状态
    }
    else
    {
      LockStatusUpdata(AmpCmdSta,AmpLockOn);  	//锁---关状态
    }
  }
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void amp_ack_up(enCCPortDef port)
{
	ampCachedef*	Cache;
	if(PcPort	==	port)
		Cache	=	&ampsys.CCdata.PcAck;
	else if(CabPort	==	port)
		Cache	=	&ampsys.CCdata.CbAck;
	else if(LayPort	==	port)
		Cache	=	&ampsys.CCdata.LyAck;
	else
		return;
	Cache->arry	=	1;
	Cache->size	=	7;
	Cache->data[0]=0x7E;
	Cache->data[1]=0x02;
	Cache->data[2]=0x81;
	Cache->data[3]=0x00;
	Cache->data[4]=0xB0;
	Cache->data[5]=0x50;
	Cache->data[6]=0x7F;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void amp_ack_down(enCCPortDef port)
{
	ampCachedef*	Cache;
	if(PcPort	==	port)
		Cache	=	&ampsys.CCdata.PcAck;
	else if(CabPort	==	port)
		Cache	=	&ampsys.CCdata.CbAck;
	else if(LayPort	==	port)
		Cache	=	&ampsys.CCdata.LyAck;
	else
		return;
	
	Cache->arry	=	1;
	Cache->size	=	7;
	Cache->data[0]=0x7E;
	Cache->data[1]=0x02;
	Cache->data[2]=0x01;
	Cache->data[3]=0x00;
	Cache->data[4]=0xD1;
	Cache->data[5]=0x90;
	Cache->data[6]=0x7F;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	1:向上应答，-1：向下应答；0:非应答
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static char is_ack_frame(unsigned char* pBuffer,unsigned short len)
{
	stampphydef* frame	=	(stampphydef*)API_Get_AmpFrame(pBuffer,len);
	if(frame)
	{
		if(0x01==frame->msg.cmd.cmd)
		{
			if(0x00==frame->msg.cmd.dir)	//向下应答
				return	1;
			else if(0x01==frame->msg.cmd.dir)	//向上应答
				return -1;
		}
		else
			return 0;
	}
	return 0;
}
//=================================锁相关程序=================================

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lock_Server(void)
{
	if(ampsys.time.LockTime>0)
	{
		if(ampsys.time.LockTime<ampUnlockOuttime-100)
		{
			ampsys.lock.sta	=	1;//0--锁为开状态，1--锁住状态
			if(ampGetLockSts)	//锁未打开
			{
				ampUnLock;
			}
			else if(ampsys.time.LockTime>200)
			{
				ampsys.time.LockTime	=	200-1;	//延迟500ms释放锁
			}
		}
		else
		{
			ampUnLock;
		}
	}
	else
	{
		ampResLock;
		if(0!=ampsys.lock.que)		//请求开锁
		{
			if(ampGetLockSts)	//锁未打开
			{
				ampResLock;
				LockStatusUpdata(AmpCmdLock,AmpLockOpenFailed);  //开锁失败
				ampsys.lock.sta	=	1;//0--锁为开状态，1--锁住状态
			}
			else	//开锁失败
			{
				ampResLock;
				LockStatusUpdata(AmpCmdLock,AmpLockOpen);  //锁已打开
				ampBakkLightOn;
				ampsys.lock.sta	=	0;//0--锁为开状态，1--锁住状态
			}
			ampsys.lock.que	=	0;	//清除开锁请求
		}
		else	//手动开锁
		{
			if(ampGetLockSts)
			{
				if(0==ampsys.lock.sta)	//关闭门
				{
					ampBakkLightOff;
					ampLayPowerOff;
					LockStatusUpdata(AmpCmdLock,AmpLockOn);  //锁已关闭
					ampsys.lock.sta	=	1;//0--锁为开状态，1--锁住状态
				}
			}
			else		//手动开锁
			{
				if(1==ampsys.lock.sta)	//原来为关闭状态
				{
					ampBakkLightOn;
					LockStatusUpdata(AmpCmdLock,AmpLockOpen);  //锁已打开
					ampsys.lock.sta	=	0;//0--锁为开状态，1--锁住状态
				}			
			}
		}
	}
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Set_Lock_Open(void)
{
	ampsys.lock.que	=	1;
	ampsys.time.LockTime	=	ampUnlockOuttime;	//开锁时间2秒
  ampUnLock;
}
//=======================================拨码开关===============================

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void SwitchID_Server(void)
{
  static unsigned char id=0;
  static unsigned char power_on_flag=0; //上电时为0

	if(0!=ampsys.time.swicthidtime)
		return;
	ampsys.time.swicthidtime	=	2000;
  if(0==power_on_flag)  //刚上电
  {
    power_on_flag = 1;  //已上电完成
    id  = SWITCHID_ReadLeft(&ampSwitchID)&0x3F;
  }
  else
  {
		unsigned char new_id  = 0;
		new_id  = SWITCHID_ReadLeft(&ampSwitchID)&0x3F;
		if(id != new_id)  //重新初始化
		{
			power_on_flag = 0;
		}
  }
}

//=======================================通讯LED指示灯=============================

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Led_Server(void)
{
  static unsigned short freshen_led_time=0;
  static unsigned char led_stata=0;
  if(freshen_led_time++>10)
  {
		freshen_led_time	=	0;
    led_stata++;
		//____________使能片选
		SPI_CS_LOW(&stLed);
		SPI_I2S_SendData(stLed.Port.SPIx, led_stata);				//发送数据
//    SPI_ReadWriteByteSPI(&stLed,led_stata);
		//____________取消片选	
		SPI_CS_HIGH(&stLed);
  }  
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short HW_SendBuff(enCCPortDef Port,unsigned char* pBuffer,unsigned short length)
{ 
  unsigned  short   sendedlen = 0;

    sendedlen = AMPCAB_SendBuff(Port,pBuffer,length);

  return  sendedlen;
}



//=================================硬件接口End=============================================================


//=================================配置函数End=============================================================


//=================================时间函数ST==============================================================

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void Tim_Server(void)
{
  //----------------PC发送
  if(ampsys.time.PcSendTime>0)
  {
    ampsys.time.PcSendTime--;
  }
  //----------------柜发送
  if(ampsys.time.CabSendTime>0)
  {
    ampsys.time.CabSendTime--;
  }
  //----------------层发送
  if(ampsys.time.LaySendTime>0)
  {
    ampsys.time.LaySendTime--;
  }
  //----------------读卡器发送
  if(ampsys.time.CardSendTime>0)
  {
    ampsys.time.CardSendTime--;
  }
  //----------------锁
  if(ampsys.time.LockTime>0)
  {
    ampsys.time.LockTime--;
  }
  //----------------拨码
  if(ampsys.time.swicthidtime>0)
  {
    ampsys.time.swicthidtime--;
  }
  //----------------运行指示灯
  if(ampsys.time.SYSLEDTime>0)
  {
    ampsys.time.SYSLEDTime--;
  }
	//----------------运行指示灯
  if(ampsys.time.BackLightTime>0)
  {
    ampsys.time.BackLightTime--;
  }	
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Hardware_Configuration(void)
{
  BackLight_Configuration();
	LayPower_Configuration();
	Lock_Configuration();
  SwitchID_Configuration();
  Led_Configuration();
  Communication_Configuration();
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Communication_Configuration(void)
{
	IOT5302Wdef ampIOT5302W;
  //-----------------------------PC接口USART1
  USART_DMA_ConfigurationNR	(ampCommPcPort,19200,gDatasize);	//USART_DMA配置--查询方式，不开中断
  
  //-----------------------------读卡器接口USART3
  ampIOT5302W.Conf.IOT5302WPort.USARTx  = ampCommCardPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_CTL_PORT  = ampCommCardCTLPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_CTL_Pin   = ampCommCardCTLPin;
  ampIOT5302W.Conf.USART_BaudRate  = ampCommCardBaudRate;
	GPIO_Configuration_OOD50(GPIOC,GPIO_Pin_7);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);
  API_IOT5302WConfiguration(&ampIOT5302W);
  //-----------------------------层板接口USART2
  ampRS485Ly.USARTx  = ampCommLayPort;
  ampRS485Ly.RS485_CTL_PORT  = ampCommLayCTLPort;
  ampRS485Ly.RS485_CTL_Pin   = ampCommLayCTLPin;
  RS485_DMA_ConfigurationNR			(&ampRS485Ly,19200,gDatasize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	GPIO_Configuration_OOD50(GPIOA,GPIO_Pin_11);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
  //-----------------------------副柜接口UART4
  ampRS485Cb.USARTx  = ampCommCbPort;
  ampRS485Cb.RS485_CTL_PORT  = ampCommCbCTLPort;
  ampRS485Cb.RS485_CTL_Pin   = ampCommCbCTLPin;
  RS485_DMA_ConfigurationNR			(&ampRS485Cb,19200,gDatasize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	GPIO_Configuration_OOD50(GPIOC,GPIO_Pin_9);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void SwitchID_Configuration(void)
{
  ampSwitchID.NumOfSW	=	8;
  
  ampSwitchID.SW1_PORT	=	GPIOB;
  ampSwitchID.SW1_Pin	=	GPIO_Pin_9;
  
  ampSwitchID.SW2_PORT	=	GPIOB;
  ampSwitchID.SW2_Pin	=	GPIO_Pin_8;
  
  ampSwitchID.SW3_PORT	=	GPIOB;
  ampSwitchID.SW3_Pin	=	GPIO_Pin_7;
  
  ampSwitchID.SW4_PORT	=	GPIOB;
  ampSwitchID.SW4_Pin	=	GPIO_Pin_6;
  
  ampSwitchID.SW5_PORT	=	GPIOB;
  ampSwitchID.SW5_Pin	=	GPIO_Pin_5;
  
  ampSwitchID.SW6_PORT	=	GPIOB;
  ampSwitchID.SW6_Pin	=	GPIO_Pin_4;
  
  ampSwitchID.SW7_PORT	=	GPIOB;
  ampSwitchID.SW7_Pin	=	GPIO_Pin_3;
  
  ampSwitchID.SW8_PORT	=	GPIOD;
  ampSwitchID.SW8_Pin	=	GPIO_Pin_2;

	SwitchIdInitialize(&ampSwitchID);						//

  ampsys.sysdata.Cab_Addr  = SWITCHID_ReadLeft(&ampSwitchID)&0x3F;  
  
  if(SWITCHID_ReadLeft(&ampSwitchID)&0x80)
  {
    ampsys.sysdata.MB_Flag=1; //0--副柜，1--主柜
  }
  else
  {
    ampsys.sysdata.MB_Flag=0; //0--副柜，1--主柜
  }
}
//=================================背光相关程序=================================
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void BackLight_Configuration(void)
{
  GPIO_Configuration_OPP50(BackLightPort,BackLightPin);
  ampBakkLightOff;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void LayPower_Configuration(void)
{
  GPIO_Configuration_OPP50(ampLayPowerPort,ampLayPowerPin);
  ampLayPowerOff;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lock_Configuration(void)
{
  GPIO_Configuration_OPP50(LockDrPort,LockDrPin);
  GPIO_Configuration_IPU(LockSiPort,LockSiPin);
  ampResLock;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Led_Configuration(void)
{
  stLed.Port.SPIx 			= SPI1;
  stLed.Port.CS_PORT  	= GPIOA;
  stLed.Port.CS_Pin   	= GPIO_Pin_4;
  stLed.Port.CLK_PORT 	= GPIOA;
  stLed.Port.CLK_Pin  	= GPIO_Pin_5;
  stLed.Port.MISO_PORT  = GPIOA;
  stLed.Port.MISO_Pin   = GPIO_Pin_6;
  stLed.Port.MOSI_PORT  = GPIOA;
  stLed.Port.MOSI_Pin   = GPIO_Pin_7;
  stLed.Port.SPI_BaudRatePrescaler_x  = SPI_BaudRatePrescaler_64;
	SPI_Initialize(&stLed);		//SPI接口配置
  //SPI_InitializeSPI(&stLed);			//SPI-DMA通讯方式配置
}
#endif
