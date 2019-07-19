

#ifdef SDQ_TEST
#include "SDQ_TEST.H"



#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_USART.H"


#include "BQ26100Slave.H"
#include 	"TOOL.H"

#include "STM32F10x_BitBand.H"

#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间



		
#define ComPortIn		USART2
#define ComPortOut	USART3

//-------------------带KEY
#define U1SDQPort   	GPIOA  
#define U1SDQPin    	GPIO_Pin_7

#define U4SDQPort   	GPIOA  
#define U4SDQPin    	GPIO_Pin_5

//-------------------飞达接口
#define FDSDQPort   	GPIOA  
#define FDSDQPin    	GPIO_Pin_6

//-------------------从机接入检测
#define FeederCheckPort   	GPIOB  
#define FeederCheckPin    	GPIO_Pin_2

#define GetSlavePortCheckInLevel	PB2in

//-------------------作为从机模拟接入控制
#define FeederConnectPort   	GPIOB  
#define FeederConnectPin    	GPIO_Pin_9

#define SetFeederConnect   		(PB9=1)  
#define SetFeederDisConnect  	(PB9=0)

//-------------------串口连接使能
#define TxEnConnectPort   	GPIOB  
#define TxEnConnectPin    	GPIO_Pin_0
#define SetTxEnable					(PB0=1)
#define SetTxDisable				(PB0=0)

//-------------------串口连接使能
#define RxEnConnectPort   	GPIOB  
#define RxEnConnectPin   		GPIO_Pin_1
#define SetRxEnable					(PB1=1)
#define SetRxDisable				(PB1=0)

//-------------------LED
#define SYSLEDPort   			GPIOA  
#define SYSLEDPin   			GPIO_Pin_0
#define SysLedOn					(PA0=0)
#define SysLedOff					(PA0=1)


bq26100slave_def SDQ_SLAVE;

unsigned short bq_test_serial=0;

unsigned char bq_rxd[20]={0};

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SDQ_TEST_Configuration(void)
{
  SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	

	//PWM_OUT(TIM2,PWM_OUTChannel1,1000000,500);	//PWM设定-20161127版本	占空比1/1000
	GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_0);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	SDQ_TEST_SYS_Configuration();
	
	SysTick_Configuration(1000);
	while(1)
	{
		api_bq26100slave_server();		//SDQ从机设备配置
	}
//	while(1)
//	{
//		if(1000==TIM3->CR1)
//		{
//			TIM3->CR1=0;
//			GPIO_Toggle(GPIOA,GPIO_Pin_0);		//将GPIO相应管脚输出翻转----V20170605
//		}
//	}
}

/*******************************************************************************
* 函数名		:
* 功能描述	:
* 输入		:
* 输出		:
* 返回 		:
*******************************************************************************/
void SDQ_TEST_Server(void)
{
	static unsigned short time = 0;
	static unsigned short time2 = 0;
	static unsigned short len_bac = 0;
	unsigned short len_temp = 0;
	
	//------------测试校验
	AuthTest();
	//api_bq26100slave_server();		//SDQ从机设备配置
	//------------
	if(GetSlavePortCheckInLevel)	//高信号，未接入
	{
		SetFeederDisConnect;
		SysLedOff;
		api_bq26100slave_reset_error_status();
		return ;
	}
	
	
	
	if(api_bq26100slave_get_error_status())		//无错误返回0，有错误返回1
	{
		time++;
		if(time>=50)
		{
			time	=	0;
			SetFeederDisConnect;
			SysLedOff;
			api_bq26100slave_reset_error_status();			
		}
	}
	else
	{
		time++;
		if(time==1000)
		{
			//time	=	0;
			SysLedOn;
			SetFeederConnect;
			api_bq26100slave_default();
		}
		else if(time>7000)
		{
			time=0;
			SetFeederDisConnect;
			SysLedOff;
		}
	}
	return;
	//--------------------防止死机
	len_temp	=api_bq26100slave_get_receivelen();	//返回接收长度
	if(0!=len_temp)		//返回接收长度
	{
		if(len_temp!=len_bac)
		{
			time2	=	0;
			len_bac	=	len_temp;
		}
		else
		{
			if(time2++>100)
			{
				time2	=	0;
				api_bq26100slave_default();
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
static void SDQ_TEST_SYS_Configuration(void)
{
	//===================GPIO配置
	GPIO_Configuration_OPP50(FeederConnectPort,FeederConnectPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	GPIO_Configuration_OPP50(TxEnConnectPort,TxEnConnectPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	SetTxEnable;
	
	GPIO_Configuration_OPP50(RxEnConnectPort,RxEnConnectPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	SetRxEnable;	
	
	GPIO_Configuration_IPU(FeederCheckPort,FeederCheckPin);				//将GPIO相应管脚配置为上拉输入模式----V20170605
	
	api_bq26100slave_configuration(&SDQ_SLAVE);												//SDQ从机设备配置
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
static void AuthTest(void)
{
	static unsigned char usart_cof_flag=0;
	static unsigned short time=0;
	static unsigned short serial=0;
	static unsigned short num=0;
	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
	unsigned char GetDigest[3]={0x4B,0x00,0xB4};
	unsigned char rxbuff[128]={0};
	unsigned char rxlen=0;
	
	
	//bq26100BufferSize //bq26100_sample_data[i][0]
	if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		api_usart_dma_configurationNR(ComPortOut,625000,128);
	}
	else
	{
		time++;
		if(time==2)
		{
			SysLedOn;
			MessgeFramTest(Message,0);
			api_usart_dma_send(ComPortOut,Message,23);
		}
		else if(time==5)
		{
			api_usart_dma_send(ComPortOut,GetDigest,3);
		}
		else if(time>100)
		{
			time=0;
			MessgeFramTest(rxbuff,3);
		}
		
		rxlen	=	api_usart_dma_receive(ComPortOut,rxbuff);
		if(rxlen)
		{		
			SysLedOff;
			if((0x4B==rxbuff[0])&&(0x16==rxbuff[1]))
				MessgeFramTest(rxbuff,1);
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
static void MessgeFramTest(unsigned char* message,unsigned char flag)
{
	
	//#include "BQ26100DATA.H"
	
	//bq26100BufferSize //bq26100_sample_data[i][0]
	unsigned char i = 0;
	unsigned char temp = 0;
	unsigned char* address = 0;
	if(0==flag)
	{
		address = api_bq26100slave_get_message_address(bq_test_serial);
		if(address)
		{
			for(i=0;i<20;i++)
			{
				
				message[i+2]=address[19-i];
			}
			message[22]=CheckSum(message,22);
		}
		
	}
	else if(1==flag)
	{
		for(i=0;i<20;i++)
		{
			bq_rxd[i]	=	message[23-i];
			
		}
		address = api_bq26100slave_get_message_address(bq_test_serial);
		if(address)
		{
			if(0!=memcmp(bq_rxd,address,20))
			{
				for(i=0;i<10;i++)
				{
					SysTick_DeleyuS(500);				//SysTick延时nuS
					if(i%2==0)
						SysLedOn;
					else
						SysLedOff;
				}				
			}
		}
	}
	else
	{
		bq_test_serial++;
		if(bq_test_serial>=api_bq26100slave_get_sample_data_size())
			bq_test_serial=0;
	}
}

//------------------------------------------------------------------------------


#endif
