#include "AMP01_HW.H"


//#include "AMP_LAY.H"
//#include "AMP_CABV11A2.H"

//#include	"AMP_PHY.H"

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

#define AmpArrySize    	30       		//每个发送端口待发送缓存排序个数
#define AmpFramesize   	256       		//每个发送端口待发送缓存排序个数
/* Private variables ---------------------------------------------------------*/

static RS485Def ampRS485Ly;   //uart4,PA15   //层板接口
static RS485Def ampRS485Cb;   //usart1,PA8    //副柜接口
//static RS485Def ampRS485Card; //usart3,PB2    //读卡器接口
static SwitchDef ampSwitchID;
static SPIDef stLed;



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
//-----------------------------层板电源控制
static void LayPower_Configuration(void);
//-----------------------------锁相关程序
static void Lock_Configuration(void);
static void Lock_Server(void);
static void Set_Lock_Open(void);
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
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void AMP_HW_Configuration(void)
{	
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	
	Hardware_Configuration();

  GPIO_Configuration_OPP50(ampSYSLEDPort,ampSYSLEDPin);

	//SysTick_DeleymS(500);				//SysTick延时nmS
  PWM_OUT(TIM2,PWM_OUTChannel1,2,500);	//PWM设定-20161127版本	占空比1/1000
	
  IWDG_Configuration(2000);			//独立看门狗配置---参数单位ms
  
  SysTick_Configuration(1000);	//系统嘀嗒时钟配置72MHz,单位为uS
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
void AMP_HW_Server(void)
{ 
	IWDG_Feed();								//独立看门狗喂狗
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
static void Pc_Server(void)
{
	unsigned short rxnum=0;
	unsigned char rxd[256];
	//================================================数据接收
	rxnum	=	api_usart_dma_receive(ampCommPcPort,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{

	}
	//================================================数据发送
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
	unsigned short 	rxnum=0;
	unsigned char 	rxd[256];
	//================================================数据接收
	rxnum	=	api_rs485_dam_receive(&ampRS485Cb,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{
	}
	//================================================数据发送
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
	unsigned short rxnum=0;
	unsigned char rxd[256];
	//================================================数据接收
	rxnum	=	api_rs485_dam_receive(&ampRS485Ly,rxd);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数
	if(rxnum)
	{
	
	}
	//================================================数据发送
//	if(0==ampsys.time.LaySendTime)
//	{
//	}
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
  api_usart_dma_configurationNR(ampCommPcPort,19200,AmpFramesize);	//USART_DMA配置--查询方式，不开中断
  
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
  api_rs485_dma_configurationNR(&ampRS485Ly,19200,AmpFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
	GPIO_Configuration_OOD50(GPIOA,GPIO_Pin_11);			//将GPIO相应管脚配置为OD(开漏)输出模式，最大速度50MHz----V20170605
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
  //-----------------------------副柜接口UART4
  ampRS485Cb.USARTx  = ampCommCbPort;
  ampRS485Cb.RS485_CTL_PORT  = ampCommCbCTLPort;
  ampRS485Cb.RS485_CTL_Pin   = ampCommCbCTLPin;
  api_rs485_dma_configurationNR(&ampRS485Cb,19200,AmpFramesize);	//USART_DMA配置--查询方式，不开中断,配置完默认为接收状态
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
	SwitchDef	switchid;
  switchid.NumOfSW	=	8;
  
  switchid.SW1_PORT	=	GPIOB;
  switchid.SW1_Pin	=	GPIO_Pin_9;
  
  switchid.SW2_PORT	=	GPIOB;
  switchid.SW2_Pin	=	GPIO_Pin_8;
  
  switchid.SW3_PORT	=	GPIOB;
  switchid.SW3_Pin	=	GPIO_Pin_7;
  
  switchid.SW4_PORT	=	GPIOB;
  switchid.SW4_Pin	=	GPIO_Pin_6;
  
  switchid.SW5_PORT	=	GPIOB;
  switchid.SW5_Pin	=	GPIO_Pin_5;
  
  switchid.SW6_PORT	=	GPIOB;
  switchid.SW6_Pin	=	GPIO_Pin_4;
  
  switchid.SW7_PORT	=	GPIOB;
  switchid.SW7_Pin	=	GPIO_Pin_3;
  
  switchid.SW8_PORT	=	GPIOD;
  switchid.SW8_Pin	=	GPIO_Pin_2;

	SwitchIdInitialize(&switchid);						//

//  ampsys.sysdata.Cab_Addr  = SWITCHID_ReadLeft(&switchid)&0x3F;  
//  
//  if(SWITCHID_ReadLeft(&switchid)&0x80)
//  {
//    ampsys.sysdata.MB_Flag=1; //0--副柜，1--主柜
//  }
//  else
//  {
//    ampsys.sysdata.MB_Flag=0; //0--副柜，1--主柜
//  }
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
  GPIO_Configuration_IPU(ampLockDrPort,ampLockDrPin);
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

