#ifdef AMP01V14A1

#include "AMP01V14A1.H"

#include	"AMP_Protocol.H"

#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_TIM.H"
#include "STM32_USART.H"
#include "STM32_SPI.H"

#include "SWITCHID.H"
#include "IOT5302W.H"     //读卡器

#include 	"CRC.H"

#include "string.h"				//串和内存操作函数头文件

//------------------锁接口J10
#define ampSYSLEDPort    						GPIOA
#define ampSYSLEDPin     						GPIO_Pin_0
//------------------通讯接口--PC
#define ampCommPcPort        				USART2
//------------------通讯接口--柜口
#define ampCommCbPort       				USART1
#define ampCommCbTxEnPort  					GPIOA
#define ampCommCbTxEnPin   					GPIO_Pin_8
#define ampCommCbRxEnPort 					GPIOC
#define ampCommCbRxEnPin						GPIO_Pin_9
//------------------通讯接口--层板接口
#define ampCommLayPort     					UART4
#define ampCommLayTxEnPort 					GPIOA
#define ampCommLayTxEnPin  					GPIO_Pin_12
#define ampCommLayRxEnPort 					GPIOA
#define ampCommLayRxEnPin						GPIO_Pin_11
//------------------通讯接口--读卡器接口
#define ampCommCardPort    					USART3
#define ampCommCardTxEnPort 				GPIOC
#define ampCommCardTxEnPin  				GPIO_Pin_8
#define ampCommCardRxEnPort 				GPIOC
#define ampCommCardRxEnPin					GPIO_Pin_7
#define ampCommCardBaudRate  				9600

//------------------锁接口J10
#define ampLockDrPort    						GPIOB
#define ampLockDrPin     						GPIO_Pin_1
#define ampLockSiPort    						GPIOC
#define ampLockSiPin     						GPIO_Pin_12
#define ampUnLock        						PB1=1
#define ampResLock       						PB1=0
#define ampGetLockSts    						PC12in
//------------------磁吸锁	PB15
#define ampMagneticLockDrPort    		GPIOB
#define ampMagneticLockDrPin     		GPIO_Pin_15
#define ampMagneticLockSiUpPort    	GPIOC
#define ampMagneticLockSiUpPin     	GPIO_Pin_14
#define ampMagneticLockSiDownPort  	GPIOC
#define ampMagneticLockSiDownPin		GPIO_Pin_13
#define ampMagneticOn        				PB15=1
#define ampMagneticOff       				PB15=0
#define ampMagneticGetLockStsUP  		PC14in   	//获取锁的状态，如果为0，表示锁已开
#define ampMagneticGetLockStsDOWN		PC13in 		//获取锁的状态，如果为0，表示锁已开
#define ampMagneticOnTime						2000			//电磁锁吸合时间--ms
//------------------背光接口J11的VCC和EN脚
#define ampBackLightPort 						GPIOB
#define ampBackLightPin  						GPIO_Pin_0
#define ampBackLighSet(Ratio)				api_pwm_oc_set_ratio(TIM3,PWM_OUTChannel3,Ratio)
#define	BackLightOnTime							2000
//------------------透明屏背光接口J11的VCC和EN脚
#define ampLcdLightPort 						GPIOB
#define ampLcdLightPin  						GPIO_Pin_14
#define ampLcdLightSet(Ratio)				api_pwm_oc_set_ratio(TIM1,PWM_OUTChannel2,Ratio)
#define	LcdLightOnTime							2000

//------------------层板电源控制J5、J6、J9共用一个控制电源
#define ampLayPowerPort  						GPIOB
#define ampLayPowerPin   						GPIO_Pin_2
#define ampLayPowerOn    						PB2=1
#define ampLayPowerOff   						PB2=0

//#define ampReSendTime					50		//ms
//#define ampUnlockOuttime   		2000      	//开锁倒计时超时时间
//#define ampMaxResendCount  		5         	//最大发送次数
/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Ly;   //uart4,PA15   		//层板接口
static RS485Def ampRS485Cb;   //usart1,PA8    	//副柜接口
//static RS485Def ampRS485Card; //usart3,PB2    //读卡器接口
static SwitchDef ampSwitchID;
static spi_def stLed;

ampdef ampsys;



//----------------------------------调试数据
//unsigned char rxd[maxFramesize];



/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void API_AMP01V14A1_Configuration(void)
{	
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	
	Hardware_Configuration();

  GPIO_Configuration_OPP50(ampSYSLEDPort,ampSYSLEDPin);

	//SysTick_DeleymS(500);				//SysTick延时nmS
  //PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM设定-20161127版本	占空比1/1000
	
	//IWDG_Configuration(3000);			//独立看门狗配置---参数单位ms
  
  SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
	
//  while(1)
//  {
//  }
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
void API_AMP01V14A1_Server(void)
{ 
	IWDG_Feed();														//独立看门狗喂狗
	
	SysLed_server();
	
	test_cabinet_address_none_process();		//未拨码柜接口发LCD测试程序
	set_time_sync_frame();									//未拨码柜接口发LCD测试程序
	Tim_Server();
  Lock_server();
	Magnetic_Lock_server();		//磁吸锁
  SwitchID_Server();
	status_server();
	
  CommLed_Server();
	BackLight_Server();
	LcdLight_Server();
	
	Pc_Server();
	Cab_Server();
	Lay_Server();
	CardReader_Server();	
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void status_server(void)
{
	static unsigned char i=0;
	static ampLockDef StLock;		//刚上电状态
	//-----------------------------刚上电等待数据稳定
	if(i<100)
	{
		i++;
		StLock	=	ampsys.Lock;
		return;
	}
	if(ampGetLockSts)	//关门状态
	{
		ampsys.Lock.flag.LockStatus	=	1;
	}
	//-----------------------------检查有无状态变化
	if(0!=memcmp((unsigned char*)&StLock,(unsigned char*)&ampsys.Lock,1))	//状态有变化.short类型为两个char长度
	{
		if(StLock.flag.LockStatus	!=	ampsys.Lock.flag.LockStatus)
		{
			set_lock_status_frame();
			StLock.flag.LockStatus	=	ampsys.Lock.flag.LockStatus;
		}
	}
//	ampsys.Sta.lockstd=0;
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void SysLed_server(void)
{
	static unsigned short time = 0;
	if(time++>1000)
	{
		time=0;
		api_gpio_toggle(ampSYSLEDPort,ampSYSLEDPin);		//将GPIO相应管脚输出翻转----V20170605
	}
}
//------------------------------------------------------------------------------


//--------------------------------static-cache_buffer
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static ampCachedef* get_cache_addr(ampPortDef port)
{
	if(NonPort	==	port)
		return 0;
	else if(PcPort	==	port)
		return ampsys.Pc.Buffer;
	else if(CabPort	==	port)
		return ampsys.Cab.Buffer;
	else if(LayPort	==	port)
		return ampsys.Lay.Buffer;
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
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame)
{
	unsigned char 	cab_addr	=	0;	//柜地址
	unsigned char 	lay_addr	=	0;	//层地址	
	unsigned short	i=0;
	unsigned short  lastarry  = 0;
	unsigned short	len	=	0;
	unsigned char*	pBuffer	=	(unsigned char*)frame;
	ampCachedef* Cache	= get_cache_addr(port);

	if(0==Cache)	//返回空地址
		return 0;
	if(0==frame)	//
		return 0;
	
	cab_addr	=	frame->msg.addr.address1;	//柜地址
	lay_addr	=	frame->msg.addr.address2;	//层地址
	
	//-------------------------地址检查
	if((CabPort==port)&&(0x00==cab_addr))	//无柜地址
		return 0;
	if((LayPort==port)&&(0x00==lay_addr))	//无层地址
		return 0;
		
	
	//-------------------------检查数据有无溢出
	len	=	frame->msg.length+5;	//head,len,crcl.crch.end
	if(maxFramesize<len)
		return 0;
	//-------------------------检查缓存中有无相同数据及查找当前最大缓存序号
	for(i=0;i<ampArrySize;i++)
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
	if(lastarry>=ampArrySize)	//从0开始
	{
		//缓存满
		return 0;
	}
	lastarry+=1;
	for(i=0;i<ampArrySize;i++)
  {
    if(0  ==  Cache[i].arry)  //0编号表示此为空缓存
    {
      memcpy(Cache[i].data,pBuffer,len);
      Cache[i].arry = lastarry;             //此缓存在发送队列中的排序
      Cache[i].size = len; 
      return  len;
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
static ampCachedef* get_cache_data(ampPortDef port)
{
	unsigned short	i=0;
	ampCachedef* Cache	= get_cache_addr(port);
	
	if(0==Cache)	//返回空地址
		return 0;
	//-------------------------先发送缓存标号为1号数据
	for(i=0;i<ampArrySize;i++)
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
static unsigned char del_cache_data(ampPortDef port)
{
	unsigned short	i=0;
	ampCachedef* Cache	= get_cache_addr(port);
	
	if(0==Cache)	//返回空地址
		return 0;
	//-------------------------先发送缓存标号为1号数据
	for(i=0;i<ampArrySize;i++)
	{
		if((Cache[i].arry))
		{
			Cache[i].arry-=1;
		}
	}
	return 1;
}
//------------------------------------------------------------------------------


//--------------------------------static-ack_flag
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char get_ack_wait_flag(ampPortDef port)
{
	//0--无需等待应答，1--需等待应答
	if(NonPort	==	port)
		return 0;
	else if(PcPort	==	port)
		return ampsys.Pc.WaitAckFlag;
	else if(CabPort	==	port)
		return ampsys.Cab.WaitAckFlag;
	else if(LayPort	==	port)
		return ampsys.Lay.WaitAckFlag;
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
static void set_ack_wait_flag(ampPortDef port)
{
	//0--无需等待应答，1--需等待应答
	if(NonPort	==	port)
		return;
	else if(PcPort	==	port)
		ampsys.Pc.WaitAckFlag =	1;
	else if(CabPort	==	port)
		ampsys.Cab.WaitAckFlag =	1;
	else if(LayPort	==	port)
		ampsys.Lay.WaitAckFlag =	1;
	else
		return;
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
static void del_ack_wait_flag(ampPortDef port)
{
	//0--无需等待应答，1--需等待应答
	if(NonPort	==	port)
		return;
	else if(PcPort	==	port)
		ampsys.Pc.WaitAckFlag =	0;
	else if(CabPort	==	port)
		ampsys.Cab.WaitAckFlag =	0;
	else if(LayPort	==	port)
		ampsys.Lay.WaitAckFlag =	0;
	else
		return;
}
//------------------------------------------------------------------------------


//--------------------------------static-port_server
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
	pc_receive_data_process();
	pc_send_data_process();
}
//------------------------------------------------------------------------------
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
	cab_receive_data_process();
	//================================================数据发送	
	cab_send_data_process();
}
//------------------------------------------------------------------------------
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
	lay_receive_data_process();
	lay_send_data_process();
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void CardReader_Server(void)
{
	unsigned char uid[16]={0};
	unsigned char len	=	0;
	api_iot5302w_server();
	len	= api_get_iot5302w_uid(uid);
	if(4==len)
	{
		ampphydef  ampframe;
	
		ampframe.head	=	headcode;
		ampframe.msg.length		=	8;
		ampframe.msg.cmd.cmd	=	ampCmdCard;
		ampframe.msg.cmd.rv		=	0;
		ampframe.msg.cmd.dir	=	1;
		ampframe.msg.addr.address1	=	ampsys.sysdata.Cab_Addr;
		ampframe.msg.addr.address2	=	0;
		ampframe.msg.addr.address3	=	0;
		ampframe.msg.data[0]	=	uid[0];
		ampframe.msg.data[1]	=	uid[1];
		ampframe.msg.data[2]	=	uid[2];
		ampframe.msg.data[3]	=	uid[3];
		
		api_set_frame(&ampframe,ampCmdCard,1);
		if(ampsys.sysdata.MB_Flag)	//主板
			set_cache_data(PcPort,&ampframe);
		else
			set_cache_data(CabPort,&ampframe);
		
		//--------------------------LED指示灯
		ampsys.CommLed.flag.rf_rx	=	1;
		ampsys.CommLed.time.rf_rx	=	ampledtime;
	}
}
//------------------------------------------------------------------------------


//--------------------------------static-port_data_process
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void pc_receive_data_process(void)
{  
	unsigned char		cmd	=	ampCmdNone;
	unsigned short 	rxnum=0;
	unsigned char 	rxd[maxFramesize];
	
	ampphydef* frame;
	//============================数据接收
	rxnum	=	api_usart_receive(ampCommPcPort,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	//============================接收处理
	if(0!=rxnum)
	{
		frame	=	api_get_frame(rxd,rxnum);		
		//--------------------------完整帧
		if(0!=frame)
		{
			//------------------------LED指示灯
			ampsys.CommLed.flag.pc_rx	=	1;
			ampsys.CommLed.time.pc_rx	=	ampledtime;
			
			cmd	=	frame->msg.cmd.cmd;
			//------------------------应答帧
			if(ampCmdAck==cmd)	//应答，删除一缓存
			{
				//----------------------应答
				unsigned char wait_ack	=	get_ack_wait_flag(PcPort);
				if(wait_ack)	//等待应答状态:接收到应答后需要删除最小号缓存
				{
					ampsys.Pc.ReSendCount	=	0;
					ampsys.Pc.SendWaitTime	=	5;
					del_cache_data(PcPort);
					del_ack_wait_flag(PcPort);
				}
			}
			//------------------------非应答帧
			else
			{
				//----------------------广播消息:本柜处理和转发到其它柜
				if(0xFF==frame->msg.addr.address1)
				{
					set_cache_data(CabPort,frame);		//发往其它柜
					set_cache_data(LayPort,frame);		//发往本柜层板
					cmd_process(frame);								//本柜相关命令处理
				}
				//----------------------本柜消息:本柜处理
				else if(ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)
				{
					set_cache_data(LayPort,frame);		//发往本柜层板
					cmd_process(frame);								//本柜相关命令处理
					set_ackup_frame(PcPort);					//向上级应答
				}
				//----------------------其它柜消息:转发
				else
				{
					set_cache_data(CabPort,frame);		//发往其它柜
					set_ackup_frame(PcPort);					//向上级应答
				}
			}
		}
		ampsys.BackLight.flag.SetLevel=3;
		ampsys.LcdLight.flag.SetLevel=3;
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void pc_send_data_process(void)
{
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	
	//============================等待发送时间已到
	if(0==ampsys.Pc.SendWaitTime)
	{
		//--------------------------检查有无应答数据要发送：应答数据优先发送
		if(ampsys.Pc.AckFrame.flag)
		{
			unsigned char* addr	=	&ampsys.Pc.AckFrame.head;
			sendnum	=	api_usart_send(ampCommPcPort,addr,ampAckSize);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
			if(sendnum)	//已将数据转移到缓存
			{	
				ampsys.Pc.AckFrame.flag	=	0;
				ampsys.Pc.SendWaitTime	=	2;
				del_ack_wait_flag(PcPort);			
			}
			else
			{
				ampsys.Pc.SendWaitTime	=	3;	//测试
			}
		}		
		//--------------------------发送数据：清除超出发送次数的缓存，上报连接超时；广播数据只发固定再次
		else
		{
			//------------------------上报PC只发一次
			if(ampsys.Pc.ReSendCount>=1)
			{
				ampsys.Pc.ReSendCount	=	0;
				del_cache_data(PcPort);
			}
			//------------------------检查缓存有无需要上传的数据
			Cache	=	get_cache_data(PcPort);
			if(0==Cache)	//无数据
			{
				ampsys.Pc.ReSendCount	=	0;
			}
			//------------------------
			else
			{
				sendnum	=	api_usart_send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
				if(sendnum)	//已将数据转移到缓存
				{	
					//set_ack_wait_flag(PcPort);
					ampsys.Pc.ReSendCount+=1;			
					ampsys.Pc.SendWaitTime	=	ampReSendTime;					
					//--------------------------LED指示灯
					ampsys.CommLed.flag.pc_tx	=	1;
					ampsys.CommLed.time.pc_tx	=	ampledtime;
				}
			}
		}
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void cab_receive_data_process(void)
{  
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
//	unsigned char 	cab_addr	=	0;	//柜地址
	unsigned char		cmd	=	ampCmdNone;
	unsigned char		dir	=	0;
//	unsigned short	Saved_len	=	0;	
	unsigned short 	rxnum=0;
	
	ampphydef* frame;
	unsigned char 	rxd[maxFramesize];
	//============================数据接收
	rxnum	=	api_rs485_receive(&ampRS485Cb,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	//============================接收处理
	if(0!=rxnum)
	{
		frame	=	api_get_frame(rxd,rxnum);
		if(0!=frame)
		{
			//--------------------------LED指示灯
			ampsys.CommLed.flag.cb_rx	=	1;
			ampsys.CommLed.time.cb_rx	=	ampledtime;
			
			cmd	=	frame->msg.cmd.cmd;
			dir	=	frame->msg.cmd.dir;
			//--------------------------检查是否为应答
			if(ampCmdAck==cmd)					//应答，删除一缓存
			{	
				//------------------------应答
				unsigned char wait_ack	=	get_ack_wait_flag(CabPort);
				if(wait_ack)
				{
					if(1==dir)	//上传
					{
						if(ampsys.sysdata.MB_Flag)		//主柜
						{
							ampsys.Cab.ReSendCount	=	0;
							ampsys.Cab.SendWaitTime	=	2;
							del_cache_data(CabPort);
							del_ack_wait_flag(CabPort);
						}
					}
					else			//向下
					{
						if(0==ampsys.sysdata.MB_Flag)		//辅柜
						{
							ampsys.Cab.ReSendCount	=	0;
							ampsys.Cab.SendWaitTime	=	2;
							del_cache_data(CabPort);
							del_ack_wait_flag(CabPort);
						}
					}
				}
			}
			//--------------------------通讯数据
			else
			{
				if(1==dir)	//上传
				{
					//----------------------主柜数据转发到PC
					if(ampsys.sysdata.MB_Flag)	//主柜
					{
						set_cache_data(PcPort,frame);			//发往PC缓存
						set_ackdown_frame(CabPort);
					}
				}
				else
				{
					//----------------------广播
					if(0xFF==frame->msg.addr.address1)
					{
						set_cache_data(LayPort,frame);
						cmd_process(frame);
					}
					//----------------------本柜
					else if(ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)
					{
						set_cache_data(LayPort,frame);
						cmd_process(frame);
						set_ackup_frame(CabPort);
					}
				}
			}		
		}
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void cab_send_data_process(void)
{	
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	ampphydef* frame;
	//============================等待发送时间已到
	if(0==ampsys.Cab.SendWaitTime)
	{
		//----------------------------------------------检查有无应答数据要发送
		if(ampsys.Cab.AckFrame.flag)
		{
			unsigned char* addr	=	&ampsys.Cab.AckFrame.head;
			sendnum	=	api_rs485_send(&ampRS485Cb,addr,ampAckSize);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
			if(sendnum)	//已将数据转移到缓存
			{	
				ampsys.Cab.AckFrame.flag	=	0;
				del_ack_wait_flag(CabPort);	//0--无需等待应答，1--需等待应答
				ampsys.Cab.SendWaitTime		=	10;
			}
		}
		//----------------------------------------------检查有无发送超时
		else
		{
			if(ampsys.Cab.ReSendCount>=ampMaxResendCount)
			{			
				Cache	=	get_cache_data(CabPort);
				if(0!=Cache)	//有数据
				{
					frame	=	(ampphydef*)Cache->data;
					ampsys.Cab.ReSendCount	=	0;			
					if(ampsys.sysdata.MB_Flag)	//主柜
					{
						set_comm_time_out_frame(PcPort,frame);
					}
					else
					{
						set_comm_time_out_frame(CabPort,frame);
					}
					del_cache_data(CabPort);
				}
			}
			//----------------------------------------------发送数据：广播数据只发两次
			Cache	=	get_cache_data(CabPort);
			if(0==Cache)	//无数据
			{
				ampsys.Cab.ReSendCount	=	0;
			}
			else
			{
				//----------------------------------------------
				sendnum	=	api_rs485_send(&ampRS485Cb,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
				if(sendnum)	//已将数据转移到缓存
				{	
					frame	=	(ampphydef*)Cache->data;		
					ampsys.Cab.ReSendCount+=1;			
					ampsys.Cab.SendWaitTime	=	ampReSendTime;
					
					//--------------------------LED指示灯
					ampsys.CommLed.flag.cb_tx	=	1;
					ampsys.CommLed.time.cb_tx	=	ampledtime;
					
					//----------------------------------------------广播数据只发两次，不需要应答
					if(0xFF==frame->msg.addr.address1)	//广播地址
					{
						if(ampsys.Cab.ReSendCount>=2)
						{
							ampsys.Cab.ReSendCount=0;
							del_cache_data(CabPort);
							del_ack_wait_flag(CabPort);			//0--无需等待应答，1--需等待应答
						}
					}
					else
					{
						set_ack_wait_flag(CabPort);			//0--无需等待应答，1--需等待应答
					}
				}
			}
		}
	}		
}
//------------------------------------------------------------------------------
/*******************************************************************************
* 函数名			:	Cmd_Process
* 功能描述		:	柜消息处理：处理针对本柜的消息/命令
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void lay_receive_data_process(void)
{  
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
	unsigned char		cmd	=	ampCmdNone;
	unsigned char		dir	=	0;
	unsigned short 	rxnum=0;
	
	ampphydef* frame;
	unsigned char 	rxd[maxFramesize];
	//============================数据接收
	rxnum	=	api_rs485_receive(&ampRS485Ly,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	//============================接收处理
	if(0!=rxnum)
	{

		frame	=	api_get_frame(rxd,rxnum);
		//--------------------------有效数据
		if(0!=frame)
		{
			//------------------------LED指示灯
			ampsys.CommLed.flag.ly_rx	=	1;
			ampsys.CommLed.time.ly_rx	=	ampledtime;
			
			cmd	=	frame->msg.cmd.cmd;
			dir	=	frame->msg.cmd.dir;
			//------------------------检查数据传输方向，层接口只接收上传
			if(0!=dir)	//下级上传消息
			{
				//----------------------检查是否为应答
				if(ampCmdAck==cmd)			//应答，删除一缓存
				{	
					//--------------------应答
					unsigned char wait_ack	=	get_ack_wait_flag(LayPort);
					if(wait_ack)
					{
						ampsys.Lay.ReSendCount	=	0;
						ampsys.Lay.SendWaitTime	=	2;
						del_cache_data(LayPort);
						del_ack_wait_flag(LayPort);
					}
				}
				//----------------------应答主柜数据转发到PC
				else
				{
					if(ampsys.sysdata.MB_Flag)	//主柜
					{
						set_cache_data(PcPort,frame);			//发往PC缓存						
					}
					else
					{
						set_cache_data(CabPort,frame);
					}
					//--------------------应答
					set_ackdown_frame(LayPort);
				}
			}
		}
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void lay_send_data_process(void)
{	
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	ampphydef* frame;
	//============================等待发送时间已到
	if(0==ampsys.Lay.SendWaitTime)
	{
		//==========================检查有无应答数据要发送
		if(ampsys.Lay.AckFrame.flag)
		{
			unsigned char* addr	=	&ampsys.Lay.AckFrame.head;
			sendnum	=	api_rs485_send(&ampRS485Ly,addr,ampAckSize);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
			if(sendnum)	//已将数据转移到缓存
			{	
				ampsys.Lay.AckFrame.flag	=	0;
				del_ack_wait_flag(LayPort);
				ampsys.Lay.SendWaitTime		=	10;
			}
		}
		//==========================检查有无发送超时
		else if(ampsys.Lay.ReSendCount>=ampMaxResendCount)
		{			
			Cache	=	get_cache_data(LayPort);
			if(0!=Cache)	//有数据
			{
				frame	=	(ampphydef*)Cache->data;
				ampsys.Lay.ReSendCount	=	0;			
				if(ampsys.sysdata.MB_Flag)	//主柜
				{
					set_comm_time_out_frame(PcPort,frame);
				}
				else
				{
					set_comm_time_out_frame(CabPort,frame);
				}
				del_cache_data(LayPort);
			}
		}
		//==========================检查有无待发送数据
		else
		{
			//------------------------发送数据：广播数据只发两次
			Cache	=	get_cache_data(LayPort);
			if(0==Cache)	//无数据
			{
				ampsys.Lay.ReSendCount	=	0;
				ampsys.Lay.SendWaitTime		=	5;
			}
			else
			{
				//----------------------
				sendnum	=	api_rs485_send(&ampRS485Ly,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
				if(sendnum)	//已将数据转移到缓存
				{	
					frame	=	(ampphydef*)Cache->data;		
					ampsys.Lay.ReSendCount+=1;			
					ampsys.Lay.SendWaitTime	=	ampReSendTime;
					
					//----------------------LED指示灯
					ampsys.CommLed.flag.ly_tx	=	1;
					ampsys.CommLed.time.ly_tx	=	ampledtime;
					
					//----------------------广播数据只发两次，不需要应答
					if((0xFF==frame->msg.addr.address2)||(0xFF==frame->msg.addr.address3))	//广播地址
					{
						if(ampsys.Lay.ReSendCount>=2)
						{
							ampsys.Lay.ReSendCount=0;
							del_cache_data(LayPort);
							del_ack_wait_flag(LayPort);			//0--无需等待应答，1--需等待应答
						}
					}
					else
					{
						set_ack_wait_flag(LayPort);				//0--无需等待应答，1--需等待应答
					}
				}
			}
		}
	}	
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void cmd_process(ampphydef* frame)
{	
	if(0!=frame)
	{	
		unsigned char	cab_addr	=	0;	//柜地址
		unsigned char cmd				=	0;
		unsigned char parameter	=	0;
		cmd	=	frame->msg.cmd.cmd;
		cab_addr		=	frame->msg.addr.address1;		//柜地址
		parameter		=	frame->msg.data[0];					//如果是控制类消息时的请求标志
		
		if((0xFF==cab_addr)||(cab_addr==ampsys.sysdata.Cab_Addr))	//地址
		{
			//------------------------------与层板供电相关的命令1
			if((ampCmdLcdConf == cmd)||(ampCmdLcdData==cmd))       //LCD配置命令
			{
				set_layer_power(1);
			}
			
			//------------------------------与层板供电相关的命令2
			if((ampCmdLed==cmd)||(ampCmdLcdPwr==cmd)||(ampCmdPwr==cmd))  //LED电源供电控制指令
			{
				set_layer_power(parameter);
			}
			
			//------------------------------与背光相关的命令
			if((ampCmdLed==cmd)||(ampCmdLcdPwr==cmd)||(ampCmdPwr==cmd)||(ampCmdBKligth==cmd))  //LED电源供电控制指令
			{
				//ampsys.BackLight.flag.Level=3;
				//ampsys.LcdLight.flag.Level=3;
			}

			//------------------------------与开门相关的命令
			if(ampCmdLock ==  cmd)   //锁控制命令
			{
				set_door(parameter);
			}
			//------------------------------查询命令
			else if(ampCmdSta ==  cmd)   //查询状态
			{
				set_lock_status_frame();
			}
			//ampsys.BackLight.flag.Level=3;
			//ampsys.LcdLight.flag.Level=3;
		}
	}
}
//------------------------------------------------------------------------------


//--------------------------------static-hardware_control
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_door(unsigned char flag)
{
	//flag不为0表示有请求
	if(0==flag)
	{
		ampsys.Lock.LockTime		=	0;	//开锁时间2秒
		ampResLock;
	}
	else
	{
		ampsys.Lock.LockTime					=	ampUnlockOuttime;	//开锁时间2秒
		ampUnLock;
	}
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 0-关闭，1-打开
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_layer_power(unsigned char flag)
{
	//flag不为0表示有请求
	if(0==flag)
	{
		ampLayPowerOff;
	}
	else
	{
		ampLayPowerOn;
	}	
}
//------------------------------------------------------------------------------






//--------------------------------static-set_frame
/*******************************************************************************
*函数名			:	getframe
*功能描述		:	获取帧地址，返回帧长度
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_ackup_frame(ampPortDef Port)
{  
	ampphydef*	frame;

	ampAckDef* ack  = 0;
	if(PcPort	==	Port)
		ack	=	(ampAckDef*)&ampsys.Pc.AckFrame;
//	else if(CabPort	==	Port)
//		ack	=	(ampAckDef*)&ampsys.commdata.CbAck;
//	else if(LayPort	==	Port)
//		ack	=	(ampAckDef*)&ampsys.commdata.LyAck;
	else
		return ;	

	frame=(ampphydef*)ack;
	
	ack->head	=	headcode;
	ack->length	=	5;
	ack->cmd.cmd	=	ampCmdAck;
	ack->cmd.rv		=	0;
	ack->cmd.dir	=	1;
	ack->address1	=	ampsys.sysdata.Cab_Addr;		//向上应答为柜地址
	ack->address2	=	0;
	ack->address3	=	0;
	ack->status=0;
	
	api_set_frame(frame,ampCmdAck,1);    //补充消息的CRC和结束符，返回帧长度
	
	ack->flag		=	1;
}
/*******************************************************************************
*函数名			:	getframe
*功能描述		:	获取帧地址，返回帧长度
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_ackdown_frame(ampPortDef Port)
{  
  ampphydef*	frame;

	ampAckDef* ack  = 0;
	if(PcPort	==	Port)
		ack	=	(ampAckDef*)&ampsys.Pc.AckFrame;
//	else if(CabPort	==	Port)
//		ack	=	(ampAckDef*)&ampsys.commdata.CbAck;
//	else if(LayPort	==	Port)
//		ack	=	(ampAckDef*)&ampsys.commdata.LyAck;
	else
		return;	

	frame=(ampphydef*)ack;
	
	frame->head	=	headcode;
	frame->msg.length	=	5;
	frame->msg.cmd.cmd	=	ampCmdAck;
	frame->msg.cmd.rv		=	0;
	frame->msg.cmd.dir	=	0;
	frame->msg.addr.address1	=	0x00;		//向下应答无地址
	frame->msg.addr.address2	=	0;
	frame->msg.addr.address3	=	0;
	frame->msg.data[0]=0;

	api_set_frame(frame,ampCmdAck,0);    //补充消息的CRC和结束符，返回帧长度
	
	ack->flag		=	1;
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_lock_status_frame(void)
{
	ampphydef	frame;
	frame.head	=	headcode;
	frame.msg.length	=	6;		//cmd,addr1,addr2,addr3,data0,data1
	frame.msg.cmd.cmd	=	ampCmdLock;
	frame.msg.cmd.rv	=	0;
	frame.msg.cmd.dir	=	1;
	frame.msg.addr.address1	=	ampsys.sysdata.Cab_Addr;
	frame.msg.addr.address2	=	0;
	frame.msg.addr.address3	=	0;
	frame.msg.data[0]	=	ampStypeLock;
	frame.msg.data[1]	=	ampsys.Lock.flag.LockStatus;
	
	api_set_frame(&frame,ampCmdLock,ampDirUp);		//设置消息帧
	
	if(ampsys.sysdata.MB_Flag)	//主板
		set_cache_data(PcPort,&frame);
	else
		set_cache_data(CabPort,&frame);
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	LockStatusUpdata
*功能描述		:	上报锁状态
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_comm_time_out_frame(ampPortDef Port,ampphydef* frame)
{
//  unsigned char databuffer[32]={0};
//  unsigned short framlength = 0;  //生成的消息长度
//  unsigned short datalength = 2;  //需要上报的数据字节数
  ampphydef  ampframe;
	
	ampframe.head	=	headcode;
	ampframe.msg.length		=	6;
	ampframe.msg.cmd.cmd	=	ampCmdSta;
	ampframe.msg.cmd.rv		=	0;
	ampframe.msg.cmd.dir	=	1;
	ampframe.msg.addr.address1	=	frame->msg.addr.address1;
	ampframe.msg.addr.address2	=	frame->msg.addr.address2;
	ampframe.msg.addr.address3	=	frame->msg.addr.address3;
	ampframe.msg.data[0]	=	ampStypeComm;
	ampframe.msg.data[1]	=	AmpCommTimeOut;
	
  switch(Port)
  {
    case  NonPort   : return ;   //不继续执行
    case  PcPort    : break;    //PC接口发送缓存
    case  CabPort   ://柜接口发送缓存
                      ampframe.msg.addr.address2 = 0;
                      ampframe.msg.addr.address3 = 0;
      break;
    case  LayPort   ://层接口发送缓存
      break;
    case  CardPort  ://读卡器接口发送缓存
      break;
    default :return;      //不继续执行 
  }
	api_set_frame(&ampframe,ampCmdSta,1);
	set_cache_data(Port,&ampframe);
}
//------------------------------------------------------------------------------







//--------------------------------static-Server
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
    id  = api_get_SwitchId_data_left(&ampSwitchID);
		if(id&0x80)
			ampsys.sysdata.MB_Flag	=	1;
		else
			ampsys.sysdata.MB_Flag	=	0;
		
		//------透明屏标志
		if(id&0x40)
			ampsys.sysdata.Display_Flag	=	1;	//透明屏标志://0--普通/无屏，1--主柜透明屏
		else
			ampsys.sysdata.Display_Flag	=	0;	//透明屏标志://0--普通/无屏，1--主柜透明屏
  }
  else
  {
		unsigned char new_id  = 0;
		new_id  = api_get_SwitchId_data_left(&ampSwitchID);
		if(id != new_id)  //重新初始化
		{
			ampsys.sysdata.Cab_Addr	=	new_id&0x3F;	

			if(new_id&0x80)
				ampsys.sysdata.MB_Flag	=	1;
			else
				ampsys.sysdata.MB_Flag	=	0;
			//------透明屏标志
		if(new_id&0x40)
			ampsys.sysdata.Display_Flag	=	1;	//透明屏标志://0--普通/无屏，1--主柜透明屏
		else
			ampsys.sysdata.Display_Flag	=	0;	//透明屏标志://0--普通/无屏，1--主柜透明屏
		
//			power_on_flag = 0;
			id = new_id;
		}
		
  }
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Lock_server(void)
{
	//==========================未拨码:背光闪烁
	if(0==ampsys.sysdata.Cab_Addr)
	{
		if(ampGetLockSts)			//锁未打开
		{
			ampUnLock;
			ampsys.Lock.LockTime=ampUnlockOuttime;
		}
		else
		{
			if(ampsys.Lock.LockTime>0)
			{
				ampsys.Lock.LockTime--;
			}
			else
			{
				ampResLock;
			}
		}
	}
	//==========================正常驱动
	else
	{
		if(ampsys.Lock.LockTime>0)
		{
			if(ampsys.Lock.LockTime<ampUnlockOuttime-100)
			{
				if(ampGetLockSts)			//锁未打开
				{
					ampUnLock;
				}
				else if(ampsys.Lock.LockTime>200)
				{
					ampsys.Lock.LockTime	=	200-1;	//延迟500ms释放锁
				}
			}
			else
			{
				ampUnLock;
			}
		}
		//============================
		else
		{
			ampResLock;
			//--------------------------门状态--关闭
			if(ampGetLockSts)
			{
				if(0==ampsys.Lock.flag.LockStatus)	//原来为开门状态
				{
					set_layer_power(0);				//关层板电源
					set_lock_status_frame();	//上报关门消息
					ampsys.Lock.flag.LockUpdata	=	1;
				}
				else
				{
					ampsys.Lock.flag.LockUpdata	=	0;
				}			
				ampsys.Lock.flag.LockStatus	=	1;
			}
			//--------------------------门状态--打开
			else
			{
				if(0!=ampsys.Lock.flag.LockStatus)	//原来为关门状态
				{
					set_lock_status_frame();
					ampsys.Lock.flag.LockUpdata	=	1;
				}
				else
				{
					ampsys.Lock.flag.LockUpdata	=	0;
				}
				ampsys.Lock.flag.LockStatus	=	0;
			}		
		}
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	Magnetic_Lock_server
*功能描述		:	磁吸锁
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Magnetic_Lock_server(void)
{
	//==========================未拨码:背光闪烁
	if(0==ampsys.sysdata.Cab_Addr)
	{
		ampMagneticOff;
		//ampsys.MagneticLock.LockTime=0;
	}
	//==========================锁状态有变化
	else if(ampsys.Lock.flag.LockUpdata)
	{
		//------------------------开门:释放磁吸锁	
		if(1==ampsys.Lock.flag.LockStatus)
		{
			ampMagneticOn;
		}
		else
		{			
			ampMagneticOff;
		}
	}
	//==========================等待一定时间后释放锁

}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void CommLed_Server(void)
{
	unsigned char displaycode	=	0;
  static unsigned short freshen_led_time=0;
	//==========================
	if(ampsys.CommLed.FlashTime++>100)
	{
		ampsys.CommLed.FlashTime=0;
		//------------------------需要点亮
		if(0==ampsys.CommLed.ClearFlag)
		{
			ampsys.CommLed.ClearFlag=1;
			
			ampsys.CommLed.displaycode.code	=	0;
			//----------------------读卡器
			if(ampsys.CommLed.time.rf_rx>0)
			{
				ampsys.CommLed.time.rf_rx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.rf_rx	=	1;
			}
			if(ampsys.CommLed.time.rf_tx>0)
			{
				ampsys.CommLed.time.rf_tx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.rf_tx	=	1;
			}
			//----------------------层接口
			if(ampsys.CommLed.time.ly_rx>0)
			{
				ampsys.CommLed.time.ly_rx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.ly_rx	=	1;;
			}
			if(ampsys.CommLed.time.ly_tx>0)
			{
				ampsys.CommLed.time.ly_tx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.ly_tx	=	1;
			}
			//----------------------柜接口
			if(ampsys.CommLed.time.cb_rx>0)
			{
				ampsys.CommLed.time.cb_rx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.cb_rx	=	1;
			}
			if(ampsys.CommLed.time.cb_tx>0)
			{
				ampsys.CommLed.time.cb_tx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.cb_tx	=	1;
			}
			//----------------------PC接口
			if(ampsys.CommLed.time.pc_rx>0)
			{
				ampsys.CommLed.time.pc_rx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.pc_rx	=	1;
			}
			if(ampsys.CommLed.time.pc_tx>0)
			{
				ampsys.CommLed.time.pc_tx-=1;
			}
			else
			{
				ampsys.CommLed.displaycode.flag.pc_tx	=	1;
			}			
			displaycode	=	ampsys.CommLed.displaycode.code;
		}
		else
		{
			ampsys.CommLed.ClearFlag=0;
			displaycode	=	0xFF;
		}
		//========================发送数据
		spi_set_nss_low(&stLed);
		SPI_I2S_SendData(stLed.port.SPIx, displaycode);				//发送数据
		spi_set_nss_high(&stLed);
	}
	return ;
  if(freshen_led_time++>100)
  {
		freshen_led_time	=	0;
		
		displaycode	=	ampsys.CommLed.display;
		displaycode	=	displaycode^0xFF;
		
		ampsys.CommLed.display	=	displaycode;
		if(ampsys.CommLed.time.rf_rx>0)
		{
			ampsys.CommLed.time.rf_rx-=1;
		}
		else
		{
			displaycode|=0x80;
		}
		if(ampsys.CommLed.time.rf_tx>0)
		{
			ampsys.CommLed.time.rf_tx-=1;
		}
		else
		{
			displaycode|=0x40;
		}
		//--------------------------------
		if(ampsys.CommLed.time.ly_rx>0)
		{
			ampsys.CommLed.time.ly_rx-=1;
		}
		else
		{
			displaycode|=0x20;
		}
		if(ampsys.CommLed.time.ly_tx>0)
		{
			ampsys.CommLed.time.ly_tx-=1;
		}
		else
		{
			displaycode|=0x10;
		}
		//--------------------------------
		if(ampsys.CommLed.time.cb_rx>0)
		{
			ampsys.CommLed.time.cb_rx-=1;
		}
		else
		{
			displaycode|=0x08;
		}
		if(ampsys.CommLed.time.cb_tx>0)
		{
			ampsys.CommLed.time.cb_tx-=1;
		}
		else
		{
			displaycode|=0x04;
		}
		//--------------------------------
		if(ampsys.CommLed.time.pc_rx>0)
		{
			ampsys.CommLed.time.pc_rx-=1;
		}
		else
		{
			displaycode|=0x02;
		}
		if(ampsys.CommLed.time.pc_tx>0)
		{
			ampsys.CommLed.time.pc_tx-=1;
		}
		else
		{
			displaycode|=0x01;
		}
		
		//____________使能片选
		spi_set_nss_low(&stLed);
		SPI_I2S_SendData(stLed.port.SPIx, displaycode);				//发送数据
//    SPI_ReadWriteByteSPI(&stLed,led_stata);
		//____________取消片选	
		spi_set_nss_high(&stLed);
  }  
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void BackLight_Server(void)
{
	//#define ampBackLightOff		PWM_OUT(TIM3,PWM_OUTChannel3,500,10)	//PWM设定-20161127版本	占空比1/1000
	//#define ampBackLightOn		PWM_OUT(TIM3,PWM_OUTChannel3,500,990)	//PWM设定-20161127版本	占空比1/1000
	//ampsys.Lock.flag.LockStatus
	//ampsys.Lock.flag.LockUpdata
	//==========================未拨码:背光闪烁
	if(0==ampsys.sysdata.Cab_Addr)
	{
		ampsys.BackLight.Time++;
		if(ampsys.BackLight.Time==500)	//亮
		{
			ampBackLighSet(900);
		}
		else if(ampsys.BackLight.Time>=1000)	//暗
		{
			ampsys.BackLight.Time	=	0;
			ampBackLighSet(100);
		}
		ampsys.BackLight.flag.WorkLevel=0;
		ampsys.BackLight.flag.SetLevel =0;
	}
	//==========================门状态改变
	else if(ampsys.Lock.flag.LockUpdata)
	{
		//------------------------开门:保持中等亮度		
		if(0==ampsys.Lock.flag.LockStatus)
		{
			if(2!=ampsys.BackLight.flag.WorkLevel)
			{
				ampBackLighSet(300);
				ampsys.BackLight.flag.WorkLevel	=	2;
				ampsys.BackLight.Time						=	0;
			}			
		}
		//------------------------关门:先高亮后低亮
		else
		{
			if(3!=ampsys.BackLight.flag.WorkLevel)
			{
				ampBackLighSet(999);
				ampsys.BackLight.flag.WorkLevel	=	3;
			}
			ampsys.BackLight.Time							=	BackLightOnTime;
		}
	}
	//==========================控制命令或者计时器
	else
	{
		//------------------------最低亮度/灭灯，计时或者需要灭灯时请求，开门时LCD背光亮度
		if(1==ampsys.BackLight.flag.SetLevel)
		{
			if(1!=ampsys.BackLight.flag.WorkLevel)
			{
				ampBackLighSet(1);
				ampsys.BackLight.flag.WorkLevel	=	1;				
			}
			ampsys.BackLight.Time						=	0;
		}
		//------------------------中等亮度，开门时柜体背光亮度
		else if(2==ampsys.BackLight.flag.SetLevel)
		{
			if(2!=ampsys.BackLight.flag.WorkLevel)
			{
				ampBackLighSet(500);
				ampsys.BackLight.flag.WorkLevel		=	2;				
			}
			ampsys.BackLight.Time							=	BackLightOnTime;
		}
		//------------------------最高亮度，关门时需要亮灯时的亮度
		else if(3==ampsys.BackLight.flag.SetLevel)
		{
			if(3!=ampsys.BackLight.flag.WorkLevel)
			{
				ampBackLighSet(999);
				ampsys.BackLight.flag.WorkLevel		=	3;				
			}
			ampsys.BackLight.Time							=	BackLightOnTime;
		}
		else
		{
			//----------------------倒计时
			if(ampsys.BackLight.Time)
			{
				ampsys.BackLight.Time--;
			}
			else
			{
				//--------------------门已打开
				if(0==ampsys.Lock.flag.LockStatus)
				{
					//------------------中等亮度
					if(2!=ampsys.BackLight.flag.WorkLevel)
					{
						ampBackLighSet(500);
						ampsys.BackLight.flag.WorkLevel		=	2;
					}
				}
				else
				{
					//------------------最低亮度/灭灯，计时或者需要灭灯时请求，开门时LCD背光亮度
					if(1!=ampsys.BackLight.flag.WorkLevel)
					{					
						ampBackLighSet(100);
						ampsys.BackLight.flag.WorkLevel		=	1;
					}
				}
			}
		}
		ampsys.BackLight.flag.SetLevel		=	0;
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void LcdLight_Server(void)
{
	//#define ampLcdLightOff	PWM_OUTN(TIM1,PWM_OUTChannel2,500,1000)	//PWM设定-20161127版本	占空比1/1000
	//#define ampLcdLightOn		PWM_OUTN(TIM1,PWM_OUTChannel2,500,0)		//PWM设定-20161127版本	占空比1/1000
	//ampsys.Lock.flag.LockStatus
	//ampsys.Lock.flag.LockUpdata
	//==========================未拨码:背光闪烁
	if(0==ampsys.sysdata.Cab_Addr)
	{
		ampsys.LcdLight.Time++;
		if(ampsys.LcdLight.Time==500)
		{
			ampLcdLightSet(100);
		}
		else if(ampsys.LcdLight.Time>=1000)
		{
			ampsys.LcdLight.Time	=	0;
			ampLcdLightSet(900);
		}
		ampsys.LcdLight.flag.WorkLevel=0;
		ampsys.LcdLight.flag.SetLevel =0;
	}
	//==========================门状态改变
	else if(ampsys.Lock.flag.LockUpdata)
	{
		//------------------------开门:灭灯
		if(0==ampsys.Lock.flag.LockStatus)
		{
			if(1!=ampsys.LcdLight.flag.WorkLevel)
			{
				ampLcdLightSet(0);
				ampsys.LcdLight.flag.WorkLevel	=	1;
				ampsys.LcdLight.Time						=	0;
			}
		}
		//------------------------关门:先高亮后灭灯
		else
		{
			if(3!=ampsys.LcdLight.flag.WorkLevel)
			{
				ampLcdLightSet(900);
				ampsys.LcdLight.flag.WorkLevel		=	3;
			}			
			ampsys.LcdLight.Time							=	LcdLightOnTime;
		}
	}
	//==========================控制命令或者计时器
	else
	{		
		//------------------------最低亮度/灭灯，计时或者需要灭灯时请求，开门时LCD背光亮度
		if(1==ampsys.LcdLight.flag.SetLevel)
		{
			if(1!=ampsys.LcdLight.flag.WorkLevel)
			{
				ampLcdLightSet(100);
				ampsys.LcdLight.flag.WorkLevel	=	1;
			}
			ampsys.LcdLight.Time						=	0;
		}
		//------------------------中等亮度，开门时柜体背光亮度
		else if(2==ampsys.LcdLight.flag.SetLevel)
		{
			if(2!=ampsys.LcdLight.flag.WorkLevel)
			{
				ampLcdLightSet(500);
				ampsys.LcdLight.flag.WorkLevel		=	2;
			}
			ampsys.LcdLight.Time							=	LcdLightOnTime;
		}
		//------------------------最高亮度，关门时需要亮灯时的亮度
		else if(3==ampsys.LcdLight.flag.SetLevel)
		{
			if(3!=ampsys.LcdLight.flag.WorkLevel)
			{
				ampLcdLightSet(900);
				ampsys.LcdLight.flag.WorkLevel		=	3;
			}
			ampsys.LcdLight.Time							=	LcdLightOnTime;
		}
		//------------------------计时器
		else
		{
			//----------------------倒计时
			if(ampsys.LcdLight.Time)
			{
				ampsys.LcdLight.Time--;
			}
			//----------------------
			else
			{
				//--------------------最低亮度
				if(1!=ampsys.LcdLight.flag.WorkLevel)
				{
					ampLcdLightSet(0);
					//PWM_OUTTim1CH2N_Set_Ratio(1);
					ampsys.LcdLight.flag.WorkLevel		=	1;
				}
			}
		}
		ampsys.LcdLight.flag.SetLevel	=	0;
	}
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void Tim_Server(void)
{
  //----------------PC发送
  if(ampsys.Pc.SendWaitTime>0)
  {
    ampsys.Pc.SendWaitTime--;
  }
  //----------------柜发送
  if(ampsys.Cab.SendWaitTime>0)
  {
    ampsys.Cab.SendWaitTime--;
  }
  //----------------层发送
  if(ampsys.Lay.SendWaitTime>0)
  {
    ampsys.Lay.SendWaitTime--;
  }
  //----------------读卡器发送
  if(ampsys.time.CardSendTime>0)
  {
    ampsys.time.CardSendTime--;
  }
  //----------------锁
  if(ampsys.Lock.LockTime>0)
  {
    ampsys.Lock.LockTime--;
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
//------------------------------------------------------------------------------







//--------------------------------static-configuration
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
//------------------------------------------------------------------------------

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
  //api_usart_dma_configurationNR(ampCommPcPort,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断
	api_usart_configuration_NR(ampCommPcPort,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断
  
  //-----------------------------读卡器接口USART3
  ampIOT5302W.Conf.IOT5302WPort.USARTx  = ampCommCardPort;
	ampIOT5302W.Conf.IOT5302WPort.RS485_TxEn_PORT	= ampCommCardTxEnPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_TxEn_Pin	= ampCommCardTxEnPin;
	ampIOT5302W.Conf.IOT5302WPort.RS485_RxEn_PORT	= ampCommCardRxEnPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_RxEn_Pin	= ampCommCardRxEnPin;
	ampIOT5302W.Conf.IOT5302WPort.RS485_CTL_PORT	=	ampCommCardRxEnPort;
	ampIOT5302W.Conf.IOT5302WPort.RS485_CTL_Pin		=	ampCommCardRxEnPin;
  ampIOT5302W.Conf.USART_BaudRate  = ampCommCardBaudRate;
  api_iot5302w_configuration(&ampIOT5302W);
	
  //-----------------------------层板接口USART2
  ampRS485Ly.USARTx  = ampCommLayPort;
  ampRS485Ly.RS485_TxEn_PORT  = ampCommLayTxEnPort;
  ampRS485Ly.RS485_TxEn_Pin   = ampCommLayTxEnPin;
	ampRS485Ly.RS485_RxEn_PORT	=	ampCommLayRxEnPort;
	ampRS485Ly.RS485_RxEn_Pin		=	ampCommLayRxEnPin;
	ampRS485Ly.RS485_CTL_PORT		=	ampCommLayRxEnPort;
	ampRS485Ly.RS485_CTL_Pin		=	ampCommLayRxEnPin;
  api_rs485_configuration_NR(&ampRS485Ly,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	
  //-----------------------------副柜接口UART4
  ampRS485Cb.USARTx  					=	ampCommCbPort;
	ampRS485Cb.RS485_TxEn_PORT	=	ampCommCbTxEnPort;
	ampRS485Cb.RS485_TxEn_Pin		=	ampCommCbTxEnPin;
	ampRS485Cb.RS485_RxEn_PORT	=	ampCommCbRxEnPort;
	ampRS485Cb.RS485_RxEn_Pin		=	ampCommCbRxEnPin;
	ampRS485Cb.RS485_CTL_PORT		=	ampCommCbRxEnPort;
	ampRS485Cb.RS485_CTL_Pin		=	ampCommCbRxEnPin;
  //api_rs485_dma_configurationIT(&ampRS485Cb,19200,maxFramesize);	//RS485_DMA配置--中断
	api_rs485_configuration_NR(&ampRS485Cb,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
}
//------------------------------------------------------------------------------

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

	api_SwitchId_initialize(&ampSwitchID);						//

  ampsys.sysdata.Cab_Addr  = api_get_SwitchId_data_left(&ampSwitchID)&0x3F;  
  
	//------主柜标志
  if(api_get_SwitchId_data_left(&ampSwitchID)&0x80)
  {
    ampsys.sysdata.MB_Flag=1; //0--副柜，1--主柜
  }
  else
  {
    ampsys.sysdata.MB_Flag=0; //0--副柜，1--主柜
  }
	//------透明屏标志
	if(api_get_SwitchId_data_left(&ampSwitchID)&0x40)
  {
    ampsys.sysdata.Display_Flag=1; //透明屏标志://0--普通/无屏，1--主柜透明屏
  }
  else
  {
    ampsys.sysdata.Display_Flag=0; //透明屏标志://0--普通/无屏，1--主柜透明屏
  }
}
//------------------------------------------------------------------------------

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
  GPIO_Configuration_OPP50(ampBackLightPort,ampBackLightPin);
	GPIO_Configuration_OPP50(ampLcdLightPort,ampLcdLightPin);
	ampsys.BackLight.flag.SetLevel=3;
	ampsys.LcdLight.flag.SetLevel=3;
	//PWM_OUTN(TIM1,PWM_OUTChannel2,500,0);	//亮
	api_pwm_oc_configurationN(TIM1,PWM_OUTChannel2,1000,500);			//LCD背光
	api_pwm_oc_configuration(TIM3,PWM_OUTChannel3,1000,500);			//柜体背光
  //ampBakkLightOff;
}
//------------------------------------------------------------------------------

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
}
//------------------------------------------------------------------------------

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
	//============================机械锁
  GPIO_Configuration_OPP50(ampLockDrPort,ampLockDrPin);
  GPIO_Configuration_IPU(ampLockSiPort,ampLockSiPin);
  ampResLock;
	//============================磁吸锁
  GPIO_Configuration_OPP50(ampMagneticLockDrPort,ampMagneticLockDrPin);
  GPIO_Configuration_IPU(ampMagneticLockSiUpPort,ampMagneticLockSiUpPin);
	GPIO_Configuration_IPU(ampMagneticLockSiDownPort,ampMagneticLockSiDownPin);
  ampMagneticOff;
}
//------------------------------------------------------------------------------

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
  stLed.port.SPIx 			= SPI1;
  stLed.port.nss_port  	= GPIOA;
  stLed.port.nss_pin   	= GPIO_Pin_4;
  stLed.port.clk_port 	= GPIOA;
  stLed.port.clk_pin  	= GPIO_Pin_5;
  stLed.port.miso_port  = GPIOA;
  stLed.port.miso_pin   = GPIO_Pin_6;
  stLed.port.mosi_port  = GPIOA;
  stLed.port.mosi_pin   = GPIO_Pin_7;
  stLed.port.SPI_BaudRatePrescaler_x  = SPI_BaudRatePrescaler_64;
	api_spi_configurationNR(&stLed);		//SPI接口配置
  //SPI_InitializeSPI(&stLed);			//SPI-DMA通讯方式配置
}
//------------------------------------------------------------------------------







//--------------------------------static-test
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void test_cabinet_address_none_process(void)		//未拨码柜接口发LCD测试程序
{
	static unsigned char	backlight_flag=0;
	static unsigned char	num=0;
	static unsigned char	address_none_flag=0;
  static unsigned short time=0;
	
  if(0==ampsys.sysdata.Cab_Addr)   //未拨码
  {    
		unsigned char test[]=
		{	0x7E,0x4C,0x09,0xFF,0xFF,0xFF,		//地址
			0x01,0x10,0xBF,0xD8,0xD6,0xC6,0xB0,0xE5,0xCE,0xB4,0xB2,0xA6,0xC2,0xEB,0xB2,0xE2,0xCA,0xD4,	//名称"控制板未拨码测试"
			0x02,0x09,0x74,0x65,0x73,0x74,0x2D,0xB9,0xE6,0xB8,0xF1,	//规格"test-规格"
			0x03,0x02,0x39,0x39,			//数量
			0x04,0x09,0x74,0x65,0x73,0x74,0x2D,0xB1,0xF0,0xC3,0xFB,	//别名"test-别名"
			0x05,0x09,0x74,0x65,0x73,0x74,0x2D,0xB3,0xA7,0xC9,0xCC,	//厂商"test-厂商"
			0x06,0x0B,0x63,0x6F,0x64,0x65,0x2D,0x74,0x65,0x73,0x74,0x30,0x31,	//编码"code-test01"
			0x07,0x02,0xB8,0xF6,0xBD,0x9E,0x7F};	//单位"个"
		
		if(time++>500)
		{
			time=0;		
			if(num<ampArrySize)
			{
				num+=1;
				test[37]=num/10+0x30;		//转为ASCII
				test[38]=num%10+0x30;
				
				api_set_frame((ampphydef*)test,ampCmdLcdData,0);			
				set_cache_data(LayPort,(ampphydef*)test);    		//往层板发送消息
				
				if(ampsys.sysdata.MB_Flag)
					set_cache_data(CabPort,(ampphydef*)test);    	//往副柜发送消息
			}
			else
			{
				num=0;
			}
			set_layer_power(1);
			set_door(1);
		}
		address_none_flag	=	1;
  }
	else
	{
		if(address_none_flag)		//表示从无拨码变为有拨码
		{
			address_none_flag	=	0;
			
			if(ampGetLockSts)			//门状态--关闭
			{
				set_layer_power(0);	//关层板电源
				ampsys.Lock.flag.LockStatus	=	1;//0--锁为开状态，1--锁住状态
			}
			else
			{
				set_layer_power(1);	//关层板电源
				ampsys.Lock.flag.LockStatus	=	0;//0--锁为开状态，1--锁住状态
			}
		}
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void set_time_sync_frame(void)		//未拨码柜接口发LCD测试程序
{
	static unsigned long	sync_time	=	0;
	
	ampphydef frame;
	
	//-----------------------------同步时间由主板发送
	if(0==ampsys.sysdata.MB_Flag)
		return;
	//-----------------------------同步时间在空闲时发送
	if(get_cache_data(CabPort))
		sync_time=0;
	if(sync_time<600000)	//600000
	{
		sync_time++;
		return ;
	}
	frame.head=headcode;
	frame.msg.length	=	6;
	frame.msg.cmd.cmd				=	ampCmdTimeSync;
	frame.msg.cmd.rv				=	0;
	frame.msg.cmd.dir				=	0;
	frame.msg.addr.address1	=	0xFF;
	frame.msg.addr.address2	=	0xFF;
	frame.msg.addr.address3	=	0xFF;
	frame.msg.data[0]				=	0x00;		//数据暂时无用
	frame.msg.data[1]				=	0x00;		//数据暂时无用
	
	api_set_frame(&frame,ampCmdTimeSync,0);
	set_cache_data(CabPort,&frame);		//其它柜LCD板
	set_cache_data(LayPort,&frame);		//本柜LCD板
	sync_time	=	0;
	
}
#endif
