#ifdef AMP01V11A3

#include "AMP01V11A3.H"

#include	"AMP_Protocol.H"

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
#define ampCommCbTxEnPort  	GPIOA         //锁控制接口，高电平开锁
#define ampCommCbTxEnPin   	GPIO_Pin_8
#define ampCommCbRxEnPort 	GPIOC
#define ampCommCbRxEnPin		GPIO_Pin_9
//------------------通讯接口--层板接口
#define ampCommLayPort     	UART4
#define ampCommLayTxEnPort 	GPIOA         //锁控制接口，高电平开锁
#define ampCommLayTxEnPin  	GPIO_Pin_12
#define ampCommLayRxEnPort 	GPIOA
#define ampCommLayRxEnPin		GPIO_Pin_11
//------------------通讯接口--读卡器接口
#define ampCommCardPort    	USART3
#define ampCommCardTxEnPort 	GPIOC         //锁控制接口，高电平开锁
#define ampCommCardTxEnPin  	GPIO_Pin_8
#define ampCommCardRxEnPort 	GPIOC
#define ampCommCardRxEnPin		GPIO_Pin_7
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

//#define ampReSendTime					50		//ms
//#define ampUnlockOuttime   		2000      	//开锁倒计时超时时间
//#define ampMaxResendCount  		5         	//最大发送次数
/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Ly;   //uart4,PA15   //层板接口
static RS485Def ampRS485Cb;   //usart1,PA8    //副柜接口
//static RS485Def ampRS485Card; //usart3,PB2    //读卡器接口
static SwitchDef ampSwitchID;
static SPIDef stLed;

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
	IWDG_Feed();								//独立看门狗喂狗
	test_cabinet_address_none_process();		//未拨码柜接口发LCD测试程序
	set_time_sync_frame();									//未拨码柜接口发LCD测试程序
	Tim_Server();
  door_server();
  SwitchID_Server();
	status_server();
  Led_Server();
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
	static ampStadef status;		//刚上电状态
	//-----------------------------刚上电等待数据稳定
	if(i<100)
	{
		i++;
		status	=	ampsys.status;
		return;
	}
	if(ampGetLockSts)	//关门状态
	{
		ampsys.status.lock	=	1;
	}
	//-----------------------------检查有无状态变化
	if(0!=memcmp((unsigned char*)&status,(unsigned char*)&ampsys.status,2))	//状态有变化.short类型为两个char长度
	{
		if(status.lock	!=	ampsys.status.lock)
		{
			set_lock_status_frame();
			status.lock	=	ampsys.status.lock;
		}
	}
//	ampsys.Sta.lockstd=0;
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
		return ampsys.commdata.pc;
	else if(CabPort	==	port)
		return ampsys.commdata.cab;
	else if(LayPort	==	port)
		return ampsys.commdata.lay;
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
		return ampsys.AckW.PC;
	else if(CabPort	==	port)
		return ampsys.AckW.CB;
	else if(LayPort	==	port)
		return ampsys.AckW.LY;
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
		ampsys.AckW.PC =	1;
	else if(CabPort	==	port)
		ampsys.AckW.CB =	1;
	else if(LayPort	==	port)
		ampsys.AckW.LY =	1;
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
		ampsys.AckW.PC =	0;
	else if(CabPort	==	port)
		ampsys.AckW.CB =	0;
	else if(LayPort	==	port)
		ampsys.AckW.LY =	0;
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
		ampsys.led.flag.rf_rx	=	1;
		ampsys.led.time.rf_rx	=	ampledtime;
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
	
	rxnum	=	api_usart_dma_receive(ampCommPcPort,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(0==rxnum)
	{
		return;
	}	
	frame	=	api_get_frame(rxd,rxnum);
	//--------------------------LED指示灯
	ampsys.led.flag.pc_rx	=	1;
	ampsys.led.time.pc_rx	=	ampledtime;
	if(0==frame)
		return;
	cmd	=	frame->msg.cmd.cmd;
	//------------------------------------------检查是否为应答
	if(ampCmdAck==cmd)	//应答，删除一缓存
	{
		//------------------------------------------应答
		unsigned char wait_ack	=	get_ack_wait_flag(PcPort);
		if(wait_ack)
		{
			ampsys.ReSend.Pc	=	0;
			ampsys.time.PcSendTime	=	5;
			del_cache_data(PcPort);
			del_ack_wait_flag(PcPort);
		}
		return ;
	}
	//------------------------------数据转发到层板
	if((ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)||(0xFF==frame->msg.addr.address1))
	{
		set_cache_data(LayPort,frame);		//发往本柜层板
	}
	//------------------------------数据转发到其它柜
	if((ampsys.sysdata.Cab_Addr!=frame->msg.addr.address1)||(0xFF==frame->msg.addr.address1))
	{
		set_cache_data(CabPort,frame);		//发往其它柜
	}
	//------------------------------命令处理
	if((ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)||(0xFF==frame->msg.addr.address1))
	{
		cmd_process(frame);		
	}
	set_ackup_frame(PcPort);	
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
static void pc_send_data_process(void)
{
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	
	if(0!=ampsys.time.PcSendTime)
	{
		return;
	}	
	//----------------------------------------------检查有无应答数据要发送
	if(ampsys.commdata.PcAck.size)
	{
		Cache	=	&ampsys.commdata.PcAck;
		sendnum	=	api_usart_dma_send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.commdata.PcAck.size	=	0;
			ampsys.time.PcSendTime	=	2;
			del_ack_wait_flag(PcPort);		//0--无需等待应答，1--需等待应答				
		}
		return;
	}		
	//----------------------------------------------发送数据：清除超出发送次数的缓存，上报连接超时；广播数据只发固定再次
	//----------上报PC只发再次
	if(ampsys.ReSend.Pc>=1)
	{
		ampsys.ReSend.Pc	=	0;
		del_cache_data(PcPort);
	}
	Cache	=	get_cache_data(PcPort);
	if(0==Cache)	//无数据
	{
		ampsys.ReSend.Pc	=	0;
		return;
	}
	sendnum	=	api_usart_dma_send(ampCommPcPort,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
	if(sendnum)	//已将数据转移到缓存
	{	
		set_ack_wait_flag(PcPort);	//0--无需等待应答，1--需等待应答
		ampsys.ReSend.Pc+=1;			
		ampsys.time.PcSendTime	=	ampReSendTime;
		
		//--------------------------LED指示灯
		ampsys.led.flag.pc_tx	=	1;
		ampsys.led.time.pc_tx	=	ampledtime;
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
static void cab_receive_data_process(void)
{  
	//--------------------------------------
	//主板:接收PC数据，广播数据，内部使用和转发
	//接收到总线数据，上传到PC
	unsigned char 	cab_addr	=	0;	//柜地址
	unsigned char		cmd	=	ampCmdNone;
	unsigned char		dir	=	0;
//	unsigned short	Saved_len	=	0;	
	unsigned short 	rxnum=0;
	
	ampphydef* frame;
	unsigned char 	rxd[maxFramesize];
	//================================================数据接收
	rxnum	=	api_rs485_dma_receive(&ampRS485Cb,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(0==rxnum)
	{
		return;
	}	
	frame	=	api_get_frame(rxd,rxnum);
	if(0==frame)
		return;
	//--------------------------LED指示灯
	ampsys.led.flag.cb_rx	=	1;
	ampsys.led.time.cb_rx	=	ampledtime;
	
	cmd	=	frame->msg.cmd.cmd;
	dir	=	frame->msg.cmd.dir;
	//------------------------------------------检查数据传输方向，如果是辅柜，只接收下发数据，如果是主柜，只接收上传数据
	if(ampsys.sysdata.MB_Flag)	//主柜
	{
		if(0==dir)	//下发
			return;
	}
	else
	{
		if(1==dir)	//上传
			return;
	}
	//------------------------------------------检查是否为应答
	if(ampCmdAck==cmd)	//应答，删除一缓存
	{	
		//------------------------------------------应答
		unsigned char wait_ack	=	get_ack_wait_flag(CabPort);
		if(wait_ack)
		{
			ampsys.ReSend.Cab	=	0;
			ampsys.time.CabSendTime	=	2;
			del_cache_data(CabPort);
			del_ack_wait_flag(CabPort);
		}
		return ;
	}
	//------------------------------------------主柜数据转发到PC
	if(ampsys.sysdata.MB_Flag)	//主柜
	{
		set_cache_data(PcPort,frame);			//发往PC缓存
		set_ackdown_frame(CabPort);
		return;
	}
	//------------------------------数据转发到层板
	if((ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)||(0xFF==frame->msg.addr.address1))
	{
		set_cache_data(LayPort,frame);
	}
	//------------------------------命令处理
	if((ampsys.sysdata.Cab_Addr==frame->msg.addr.address1)||(0xFF==frame->msg.addr.address1))
	{
		cmd_process(frame);
	}
	//------------------------------:应答;只对本地址数据应答
	if((0xFF!=cab_addr)&&(ampsys.sysdata.Cab_Addr==frame->msg.addr.address1))	//广播数据:不应答
	{
		//----------------------------此为辅柜
		set_ackup_frame(CabPort);
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
static void cab_send_data_process(void)
{	
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	ampphydef* frame;
	
	if(0!=ampsys.time.CabSendTime)
	{
		return;
	}		
	//----------------------------------------------检查有无应答数据要发送
	if(ampsys.commdata.CbAck.size)
	{
		Cache	=	&ampsys.commdata.CbAck;
		sendnum	=	api_rs485_dma_send(&ampRS485Cb,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.commdata.CbAck.size	=	0;
			del_ack_wait_flag(CabPort);	//0--无需等待应答，1--需等待应答
			ampsys.time.CabSendTime		=	10;
		}
		return;
	}
	//----------------------------------------------检查有无发送超时
	if(ampsys.ReSend.Cab>=ampMaxResendCount)
	{			
		Cache	=	get_cache_data(CabPort);
		if(0!=Cache)	//有数据
		{
			frame	=	(ampphydef*)Cache->data;
			ampsys.ReSend.Cab	=	0;			
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
		ampsys.ReSend.Cab	=	0;
		ampsys.time.CabSendTime		=	5;
		return;
	}	
	//----------------------------------------------
	sendnum	=	api_rs485_dma_send(&ampRS485Cb,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
	if(sendnum)	//已将数据转移到缓存
	{	
		frame	=	(ampphydef*)Cache->data;		
		ampsys.ReSend.Cab+=1;			
		ampsys.time.CabSendTime	=	ampReSendTime;
		
		//--------------------------LED指示灯
		ampsys.led.flag.cb_tx	=	1;
		ampsys.led.time.cb_tx	=	ampledtime;
		
		//----------------------------------------------广播数据只发两次，不需要应答
		if(0xFF==frame->msg.addr.address1)	//广播地址
		{
			if(ampsys.ReSend.Cab>=2)
			{
				ampsys.ReSend.Cab=0;
				del_cache_data(CabPort);
				del_ack_wait_flag(CabPort);			//0--无需等待应答，1--需等待应答
				return;
			}
		}
		else
		{
			set_ack_wait_flag(CabPort);			//0--无需等待应答，1--需等待应答
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
	//================================================数据接收
	rxnum	=	api_rs485_dma_receive(&ampRS485Ly,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(0==rxnum)
	{
		return;
	}
	frame	=	api_get_frame(rxd,rxnum);
	if(0==frame)
		return;
	//--------------------------LED指示灯
	ampsys.led.flag.ly_rx	=	1;
	ampsys.led.time.ly_rx	=	ampledtime;
	
	cmd	=	frame->msg.cmd.cmd;
	dir	=	frame->msg.cmd.dir;
	//------------------------------------------检查数据传输方向，层接口只接收上传
	if(0==dir)	//下发
		return;
	//------------------------------------------检查是否为应答
	if(ampCmdAck==cmd)	//应答，删除一缓存
	{	
		//------------------------------------------应答
		unsigned char wait_ack	=	get_ack_wait_flag(LayPort);
		if(wait_ack)
		{
			ampsys.ReSend.Lay	=	0;
			ampsys.time.LaySendTime	=	2;
			del_cache_data(LayPort);
			del_ack_wait_flag(LayPort);
		}
		return ;
	}
	//------------------------------------------主柜数据转发到PC
	if(ampsys.sysdata.MB_Flag)	//主柜
	{
		set_cache_data(PcPort,frame);			//发往PC缓存
		set_ackdown_frame(LayPort);
		return;
	}
	else
	{
		set_cache_data(CabPort,frame);
		set_ackdown_frame(LayPort);
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
static void lay_send_data_process(void)
{	
	unsigned short sendnum=0;
	ampCachedef*	Cache;
	ampphydef* frame;
	
	if(0!=ampsys.time.LaySendTime)
	{
		return;
	}		
	//----------------------------------------------检查有无应答数据要发送
	if(ampsys.commdata.LyAck.size)
	{
		Cache	=	&ampsys.commdata.CbAck;
		sendnum	=	api_rs485_dma_send(&ampRS485Ly,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
		if(sendnum)	//已将数据转移到缓存
		{	
			ampsys.commdata.LyAck.size	=	0;
			del_ack_wait_flag(LayPort);	//0--无需等待应答，1--需等待应答
			ampsys.time.LaySendTime		=	10;
		}
		return;
	}
	//----------------------------------------------检查有无发送超时
	if(ampsys.ReSend.Lay>=ampMaxResendCount)
	{			
		Cache	=	get_cache_data(LayPort);
		if(0!=Cache)	//有数据
		{
			frame	=	(ampphydef*)Cache->data;
			ampsys.ReSend.Lay	=	0;			
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
	//----------------------------------------------发送数据：广播数据只发两次
	Cache	=	get_cache_data(LayPort);
	if(0==Cache)	//无数据
	{
		ampsys.ReSend.Lay	=	0;
		ampsys.time.LaySendTime		=	5;
		return;
	}	
	//----------------------------------------------
	sendnum	=	api_rs485_dma_send(&ampRS485Ly,Cache->data,Cache->size);		//串口DMA发送程序，如果数据已经传入到DMA，返回Buffer大小，否则返回0
	if(sendnum)	//已将数据转移到缓存
	{	
		frame	=	(ampphydef*)Cache->data;		
		ampsys.ReSend.Lay+=1;			
		ampsys.time.LaySendTime	=	ampReSendTime;
		
		//--------------------------LED指示灯
		ampsys.led.flag.ly_tx	=	1;
		ampsys.led.time.ly_tx	=	ampledtime;
		
		//----------------------------------------------广播数据只发两次，不需要应答
		if((0xFF==frame->msg.addr.address2)||(0xFF==frame->msg.addr.address3))	//广播地址
		{
			if(ampsys.ReSend.Lay>=2)
			{
				ampsys.ReSend.Lay=0;
				del_cache_data(LayPort);
				del_ack_wait_flag(LayPort);			//0--无需等待应答，1--需等待应答
				return;
			}
		}
		else
		{
			set_ack_wait_flag(LayPort);			//0--无需等待应答，1--需等待应答
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
static void cmd_process(ampphydef* frame)
{
	unsigned char	cab_addr	=	0;	//柜地址
	unsigned char cmd				=	0;
	unsigned char parameter	=	0;
	
	if(0==frame)
		return;
	
	cmd	=	frame->msg.cmd.cmd;
	cab_addr		=	frame->msg.addr.address1;		//柜地址
	parameter		=	frame->msg.data[0];				//如果是控制类消息时的请求标志
	
	if((0xFF!=cab_addr)&&(cab_addr!=ampsys.sysdata.Cab_Addr))	//地址
		return;
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
		set_backlight(parameter);
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
		return;
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
		ampsys.request.unlock	=	0;
		ampsys.time.LockTime	=	0;	//开锁时间2秒
		ampResLock;
	}
	else
	{
		ampsys.request.unlock	=	1;
		ampsys.time.LockTime	=	ampUnlockOuttime;	//开锁时间2秒
		ampUnLock;
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
static void set_backlight(unsigned char flag)
{
	//flag不为0表示有请求
	if(0==flag)
	{
		ampBakkLightOff;
	}
	else
	{
		ampBakkLightOn;
	}	
}
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

	ampCachedef*	Cache;
	if(PcPort	==	Port)
		Cache	=	&ampsys.commdata.PcAck;
	else if(CabPort	==	Port)
		Cache	=	&ampsys.commdata.CbAck;
	else if(LayPort	==	Port)
		Cache	=	&ampsys.commdata.LyAck;
	else
		return ;	

	frame=(ampphydef*)Cache->data;
	
	frame->head	=	headcode;
	frame->msg.length	=	5;
	frame->msg.cmd.cmd	=	ampCmdAck;
	frame->msg.cmd.rv		=	0;
	frame->msg.cmd.dir	=	1;
	frame->msg.addr.address1	=	ampsys.sysdata.Cab_Addr;		//向上应答为柜地址
	frame->msg.addr.address2	=	0;
	frame->msg.addr.address3	=	0;
	frame->msg.data[0]=0;
	
	Cache->arry	=	1;
	Cache->size	=	api_set_frame(frame,ampCmdAck,1);    //补充消息的CRC和结束符，返回帧长度
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

	ampCachedef*	Cache;
	if(PcPort	==	Port)
		Cache	=	&ampsys.commdata.PcAck;
	else if(CabPort	==	Port)
		Cache	=	&ampsys.commdata.CbAck;
	else if(LayPort	==	Port)
		Cache	=	&ampsys.commdata.LyAck;
	else
		return;	

	frame=(ampphydef*)Cache->data;
	
	frame->head	=	headcode;
	frame->msg.length	=	5;
	frame->msg.cmd.cmd	=	ampCmdAck;
	frame->msg.cmd.rv		=	0;
	frame->msg.cmd.dir	=	0;
	frame->msg.addr.address1	=	0x00;		//向下应答无地址
	frame->msg.addr.address2	=	0;
	frame->msg.addr.address3	=	0;
	frame->msg.data[0]=0;
	
	Cache->arry	=	1;
	Cache->size	=	api_set_frame(frame,ampCmdAck,0);    //补充消息的CRC和结束符，返回帧长度
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
	frame.msg.data[1]	=	ampsys.status.lock;
	
	api_set_frame(&frame,ampCmdLock,ampDirUp);		//设置消息帧
	
	if(ampsys.sysdata.MB_Flag)	//主板
		set_cache_data(PcPort,&frame);
	else
		set_cache_data(CabPort,&frame);
}
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
//			power_on_flag = 0;
			id = new_id;
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
static void door_server(void)
{
	if(ampsys.time.LockTime>0)
	{
		if(ampsys.time.LockTime<ampUnlockOuttime-100)
		{
			if(ampGetLockSts)			//锁未打开
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
		if(0!=ampsys.request.unlock)		//请求开锁
		{
			if(ampGetLockSts)		//门状态--未打开
			{
				ampsys.status.lock			=	1;
				ampsys.status.unlockerr	=	1;//开锁失败								
			}
			else	//开锁失败
			{
				set_backlight(1);	//亮背光
				ampsys.status.lock	=	0;
				ampsys.status.unlockerr	=	0;//开锁失败
			}
			ampResLock;
			ampsys.request.unlock	=	0;	//清除开锁请求
			set_lock_status_frame();
		}
		else
		{
			if(ampGetLockSts)			//门状态--关闭
			{
				if(0==ampsys.status.lock)		//关闭门
				{
					set_backlight(0);		//关背光
					set_layer_power(0);	//关层板电源
					ampsys.status.lock	=	1;//0--锁为开状态，1--锁住状态
				}
			}
			else		//手动开门
			{
				if(1==ampsys.status.lock)	//原来为关闭状态
				{
					set_backlight(1);	//亮背光
					ampsys.status.lock	=	0;//0--锁为开状态，1--锁住状态
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
static void Led_Server(void)
{
	unsigned char displaycode	=	0;
  static unsigned short freshen_led_time=0;
  if(freshen_led_time++>100)
  {
		freshen_led_time	=	0;
		
		displaycode	=	ampsys.led.display;
		displaycode	=	displaycode^0xFF;
		
		ampsys.led.display	=	displaycode;
		if(ampsys.led.time.rf_rx>0)
		{
			ampsys.led.time.rf_rx-=1;
		}
		else
		{
			displaycode|=0x80;
		}
		if(ampsys.led.time.rf_tx>0)
		{
			ampsys.led.time.rf_tx-=1;
		}
		else
		{
			displaycode|=0x40;
		}
		//--------------------------------
		if(ampsys.led.time.ly_rx>0)
		{
			ampsys.led.time.ly_rx-=1;
		}
		else
		{
			displaycode|=0x20;
		}
		if(ampsys.led.time.ly_tx>0)
		{
			ampsys.led.time.ly_tx-=1;
		}
		else
		{
			displaycode|=0x10;
		}
		//--------------------------------
		if(ampsys.led.time.cb_rx>0)
		{
			ampsys.led.time.cb_rx-=1;
		}
		else
		{
			displaycode|=0x08;
		}
		if(ampsys.led.time.cb_tx>0)
		{
			ampsys.led.time.cb_tx-=1;
		}
		else
		{
			displaycode|=0x04;
		}
		//--------------------------------
		if(ampsys.led.time.pc_rx>0)
		{
			ampsys.led.time.pc_rx-=1;
		}
		else
		{
			displaycode|=0x02;
		}
		if(ampsys.led.time.pc_tx>0)
		{
			ampsys.led.time.pc_tx-=1;
		}
		else
		{
			displaycode|=0x01;
		}
		
		//____________使能片选
		SPI_CS_LOW(&stLed);
		SPI_I2S_SendData(stLed.Port.SPIx, displaycode);				//发送数据
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
static void Tim_Server(void)
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
  api_usart_dma_configurationNR(ampCommPcPort,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断
  
  //-----------------------------读卡器接口USART3
  ampIOT5302W.Conf.IOT5302WPort.USARTx  = ampCommCardPort;
	ampIOT5302W.Conf.IOT5302WPort.RS485_TxEn_PORT  = ampCommCardTxEnPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_TxEn_Pin   = ampCommCardTxEnPin;
	ampIOT5302W.Conf.IOT5302WPort.RS485_RxEn_PORT  = ampCommCardRxEnPort;
  ampIOT5302W.Conf.IOT5302WPort.RS485_RxEn_Pin   = ampCommCardRxEnPin;
  ampIOT5302W.Conf.USART_BaudRate  = ampCommCardBaudRate;
  api_iot5302w_configuration(&ampIOT5302W);
	
  //-----------------------------层板接口USART2
  ampRS485Ly.USARTx  = ampCommLayPort;
  ampRS485Ly.RS485_TxEn_PORT  = ampCommLayTxEnPort;
  ampRS485Ly.RS485_TxEn_Pin   = ampCommLayTxEnPin;
	ampRS485Ly.RS485_RxEn_PORT	=	ampCommLayRxEnPort;
	ampRS485Ly.RS485_RxEn_Pin		=	ampCommLayRxEnPin;
  api_rs485_dma_configurationNR(&ampRS485Ly,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	
  //-----------------------------副柜接口UART4
  ampRS485Cb.USARTx  					=	ampCommCbPort;
	ampRS485Cb.RS485_TxEn_PORT	=	ampCommCbTxEnPort;
	ampRS485Cb.RS485_TxEn_Pin		=	ampCommCbTxEnPin;
	ampRS485Cb.RS485_RxEn_PORT	=	ampCommCbRxEnPort;
	ampRS485Cb.RS485_RxEn_Pin		=	ampCommCbRxEnPin;
  //api_rs485_dma_configurationIT(&ampRS485Cb,19200,maxFramesize);	//RS485_DMA配置--中断
	api_rs485_dma_configurationNR(&ampRS485Cb,19200,maxFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
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

	api_SwitchId_initialize(&ampSwitchID);						//

  ampsys.sysdata.Cab_Addr  = api_get_SwitchId_data_left(&ampSwitchID)&0x3F;  
  
  if(api_get_SwitchId_data_left(&ampSwitchID)&0x80)
  {
    ampsys.sysdata.MB_Flag=1; //0--副柜，1--主柜
  }
  else
  {
    ampsys.sysdata.MB_Flag=0; //0--副柜，1--主柜
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
static void BackLight_Configuration(void)
{
  GPIO_Configuration_OPP50(ampBackLightPort,ampBackLightPin);
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
  GPIO_Configuration_OPP50(ampLockDrPort,ampLockDrPin);
  GPIO_Configuration_IPU(ampLockSiPort,ampLockSiPin);
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
			
			//---------------------背光闪烁
			if(0!=backlight_flag)
			{
				set_backlight(1);
				backlight_flag	=	0;
			}
			else
			{
				set_backlight(0);
				backlight_flag	=	1;
			}
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
				set_backlight(0);		//关背光
				ampsys.status.lock	=	1;//0--锁为开状态，1--锁住状态
			}
			else
			{
				set_layer_power(1);	//关层板电源
				set_backlight(1);		//关背光---门打开时需要打开背光
				ampsys.status.lock	=	0;//0--锁为开状态，1--锁住状态
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
