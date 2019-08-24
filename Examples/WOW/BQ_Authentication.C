

#ifdef BQ_Authentication
#include "BQ_Authentication.H"



#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_TIM.H"
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



/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void BQ_Authentication_Configuration(void)
{
  SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	

	//api_pwm_oc_configuration(TIM2,PWM_OUTChannel1,1000000,500);	//PWM设定-20161127版本	占空比1/1000
	GPIO_Configuration_OPP50(GPIOA,GPIO_Pin_0);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	BQ_Authentication_SYS_Configuration();
	
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
//------------------------------------------------------------------------------


/*******************************************************************************
* 函数名		:
* 功能描述	:
* 输入		:
* 输出		:
* 返回 		:
*******************************************************************************/
void BQ_Authentication_Server(void)
{
	//api_bq26100slave_server();		//SDQ从机设备配置
	//------------
//	if(GetSlavePortCheckInLevel)	//高信号，未接入
//	{
//		SetFeederDisConnect;
//		SysLedOff;
//		//api_bq26100slave_reset_error_status();
//		return ;
//	}
	
	//------------离线测试校验：测试模式--定时发送待验证信息
	//BQ_Authentication_Test_Model_WorkAsSMT();
	//------------在线测试校验:定时连接SMT，让SMT发送待验证消息
	//BQ_Authentication_Test_Model_OnLine();
	//------------在线应用模式：校验不通过自动重启
	//BQ_Authentication_Auto_Reset_OnLine();
	//------------正常运行模式：
	//BQ_Authentication_Running_Model_OnLine();	//正常运行模式
	
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
static void BQ_Authentication_SYS_Configuration(void)
{
	//===================GPIO配置
	GPIO_Configuration_OPP50(FeederConnectPort,FeederConnectPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	GPIO_Configuration_OPP50(TxEnConnectPort,TxEnConnectPin);					//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	SetTxEnable;
	
	GPIO_Configuration_OPP50(RxEnConnectPort,RxEnConnectPin);					//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	SetRxEnable;	
	
	GPIO_Configuration_IPU(FeederCheckPort,FeederCheckPin);						//将GPIO相应管脚配置为上拉输入模式----V20170605
	
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
static void BQ_Authentication_Test_Model_WorkAsSMT(void)
{
	static unsigned char usart_cof_flag=0;
	static unsigned short time=0;
	static unsigned short bq_test_serial=0;		//认证消息序号
//	static unsigned short serial=0;
//	static unsigned short num=0;
	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
	unsigned char GetDigest[3]={0x4B,0x00,0xB4};		//让FD返回校验消息
	unsigned char rxbuff[128]={0};
	unsigned char rxlen=0;
	unsigned char i = 0;
	unsigned char* address = 0;
	
	
	if(GetSlavePortCheckInLevel)	//高信号，未接入
	{
		SetFeederDisConnect;
		SysLedOff;
		//api_bq26100slave_reset_error_status();
	}
	//=======================配置串口
	else if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		api_usart_configuration_NR(ComPortOut,625000,128);
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------获取待认证消息，发送认证消息
		time++;
		if(time==2)
		{
			//-------------------获取待认证消息
			//bq_test_serial=399;
			address = api_bq26100slave_get_sample_message_address(bq_test_serial);
			if(address)
			{
				//-----------------重新排序:逆向
				for(i=0;i<20;i++)
				{
					//---------------前两字节是0x4C,0x14
					Message[i+2]=address[19-i];
				}
				Message[0]=0x4C;
				Message[1]=0x14;
				//-----------------计算校验和
				Message[22]=CheckSum(Message,22);				
				//-----------------将待认证消息发给飞达
				api_usart_send(ComPortOut,Message,23);		//模拟主机将消息发送给FD验证
			}
			else
			{
				time	=	200;
			}
			SysLedOff;
		}
		//---------------------向飞达发送返回摘要的命令
		else if(time==160)
		{
			api_usart_send(ComPortOut,GetDigest,3);		//让FD返回校验消息
		}
		//---------------------序号循环自增
		else if(time>200)
		{
			time=0;
			bq_test_serial++;
			if(bq_test_serial>=api_bq26100slave_get_sample_data_size())
				bq_test_serial=0;
		}
		
		
		//---------------------获取飞达返回的数据
		rxlen	=	api_usart_receive(ComPortOut,rxbuff);	
		if(rxlen)
		{			
			//-------------------返回消息摘要:完整消息长度25字节
			if((0x4B==rxbuff[0])&&(0x16==rxbuff[1])&&(25==rxlen))
			{
				//-----------------提取摘要
				for(i=0;i<20;i++)
				{
					Message[i]	=	rxbuff[23-i];			
				}
				//-----------------获取当前待认证的消息对应的摘要地址
				address = api_bq26100slave_get_sample_digest_address(bq_test_serial);
				if(address)
				{
					//---------------对比数据：不一致
					if(0!=memcmp(Message,address,20))
					{
						SysLedOff;
					}
					//---------------对比一致
					else
					{
						SysLedOn;
					}
				}
				else
				{
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
static void BQ_Authentication_Test_Model_OnLine(void)
{
	static unsigned char usart_cof_flag=0;
	static unsigned long led_time=0;
	static unsigned long connect_time=0;
	static unsigned char led_flag=0;
//	static unsigned short bq_test_serial=0;		//认证消息序号
//	static unsigned short serial=0;
//	static unsigned short num=0;
//	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
//	unsigned char GetDigest[3]={0x4B,0x00,0xB4};		//让FD返回校验消息
//	unsigned char rxbuff[128]={0};
//	unsigned char rxlen=0;
//	unsigned char i = 0;
//	unsigned char* address = 0;
	
	
	if(GetSlavePortCheckInLevel)	//高信号，未接入
	{
		SetFeederDisConnect;

		led_time++;
		if(50==led_time)
		{			
			led_time	=	0;				
			if(led_flag)
			{
				SysLedOff;
				led_flag=0;
			}
			else
			{
				SysLedOn;
				led_flag=1;
			}
		}		
		//api_bq26100slave_reset_error_status();
	}
	//=======================配置串口
	else if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
//		//---------------------连接控制接口
//		GPIO_Configuration_OPP50(FeederConnectPort,FeederConnectPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
//		//---------------------TXD跳线
//		GPIO_Configuration_OPP50(TxEnConnectPort,TxEnConnectPin);					//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
//		SetTxEnable;
//		//---------------------RXD跳线
//		GPIO_Configuration_OPP50(RxEnConnectPort,RxEnConnectPin);					//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
//		SetRxEnable;	
//		//---------------------飞达接入检测
//		GPIO_Configuration_IPU(FeederCheckPort,FeederCheckPin);						//将GPIO相应管脚配置为上拉输入模式----V20170605		
//		//---------------------关闭发送--USART3-TX
//		GPIO_Configuration_INF(GPIOB,	GPIO_Pin_10);												//将GPIO相应管脚配置为浮空输入模式----V20170605
//		//---------------------关闭接收--USART3-RX
//		GPIO_Configuration_INF(GPIOB,	GPIO_Pin_11);												//将GPIO相应管脚配置为浮空输入模式----V20170605
//		//---------------------关闭发送--USART2-TX
//		GPIO_Configuration_INF(GPIOA,	GPIO_Pin_2);												//将GPIO相应管脚配置为浮空输入模式----V20170605
//		//---------------------关闭接收--USART2-RX
//		GPIO_Configuration_INF(GPIOA,	GPIO_Pin_3);												//将GPIO相应管脚配置为浮空输入模式----V20170605
//		//api_usart_configuration_NR(ComPortOut,625000,128);
//		api_bq26100slave_configuration(&SDQ_SLAVE);												//SDQ从机设备配置
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------飞达定时连接和断开:完整的认证和电机初始化需要3.5秒
		connect_time++;
		if(1000==connect_time)
		{
			SetFeederConnect;
			api_bq26100slave_all_data_default();
		}
		else if(connect_time>=10000)		//10秒断开
		{
			connect_time	=	0;
			SetFeederDisConnect;
		}		
		//---------------------认证无误
		if(0==api_bq26100slave_get_error_status())		//无错误返回0，有错误返回1
		{			
			led_time++;
			if(200==led_time)
			{
				led_time	=	0;				
				if(led_flag)
				{
					SysLedOff;
					led_flag=0;
				}
				else
				{
					SysLedOn;
					led_flag=1;
				}
			}
		}
		//---------------------认证出错:断开后重新连接
		else
		{
			SysLedOff;
		}
	}
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	BQ_Authentication_Auto_Reset_OnLine
*功能描述		:	认证失败自动重启
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void BQ_Authentication_Auto_Reset_OnLine(void)
{
	static unsigned char usart_cof_flag=0;
	static unsigned short time=0;
	static unsigned short bq_test_serial=0;		//认证消息序号
//	static unsigned short serial=0;
//	static unsigned short num=0;
	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
	unsigned char GetDigest[3]={0x4B,0x00,0xB4};		//让FD返回校验消息
	unsigned char rxbuff[128]={0};
	unsigned char rxlen=0;
	unsigned char i = 0;
	unsigned char* address = 0;
	
	
	if(GetSlavePortCheckInLevel)	//高信号，未接入
	{
		SetFeederDisConnect;
		SysLedOff;
		//api_bq26100slave_reset_error_status();
	}
	//=======================配置串口
	else if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		//---------------------配置串口
		api_usart_configuration_NR(ComPortOut,625000,128);
		//---------------------关闭发送--USART3-TX
		GPIO_Configuration_INF(GPIOB,	GPIO_Pin_10);			//将GPIO相应管脚配置为浮空输入模式----V20170605
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------检查认证有无异常
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
		//---------------------截取飞达返回的数据：如梦飞达有上报摘要，表示已完成一次认证
		//---------------------检查样品摘要，如果样品摘要全为0x00，表示没有找到对应的摘要，需要重新认证
		//---------------------将串口截取的摘要与样品摘要对比，如果不一致，表示SDQ读取时序异常，需要重新认证
		rxlen	=	api_usart_receive(ComPortOut,rxbuff);	
		if(rxlen)
		{			
			//-------------------返回消息摘要:完整消息长度25字节
			if((0x4B==rxbuff[0])&&(0x16==rxbuff[1])&&(25==rxlen))
			{
				//-----------------提取摘要
				for(i=0;i<20;i++)
				{
					Message[i]	=	rxbuff[23-i];			
				}
				//-----------------获取当前待认证的消息对应的摘要地址
				address = api_bq26100slave_get_sample_digest_address(bq_test_serial);
				if(address)
				{
					//---------------对比数据：不一致
					if(0!=memcmp(Message,address,20))
					{
					}
					//---------------对比一致
					else
					{
						//-------------点亮LED指示灯
						SysLedOn;
					}
				}
				else
				{
				}
			}
		}
		//---------------------获取待认证消息，发送认证消息
		time++;
		if(time==2)
		{
			//-------------------获取待认证消息
			address = api_bq26100slave_get_sample_message_address(bq_test_serial);
			if(address)
			{
				//-----------------重新排序:逆向
				for(i=0;i<20;i++)
				{
					//---------------前两字节是0x4C,0x14
					Message[i+2]=address[19-i];
				}
				//-----------------计算校验和
				Message[22]=CheckSum(Message,22);				
				//-----------------将待认证消息发给飞达
				api_usart_send(ComPortOut,Message,23);		//模拟主机将消息发送给FD验证				
			}
			else
			{
				time	=	0;
				bq_test_serial++;
			}
			SysLedOff;
		}
		//---------------------向飞达发送返回摘要的命令
		else if(time==5)
		{
			api_usart_send(ComPortOut,GetDigest,3);		//让FD返回校验消息
		}
		//---------------------序号循环自增
		else if(time>100)
		{
			time=0;
			bq_test_serial++;
			if(bq_test_serial>=api_bq26100slave_get_sample_data_size())
				bq_test_serial=0;
		}		
	}	
}
//------------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	BQ_Authentication_Running_Model_OnLine
*功能描述		:	正常运行模式--模拟BQ模式
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void BQ_Authentication_Running_Model_OnLine(void)
{
	static unsigned char usart_cof_flag=0;
	static unsigned short time=0;
//	static unsigned short bq_test_serial=0;		//认证消息序号
//	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
//	unsigned char GetDigest[3]={0x4B,0x00,0xB4};		//让FD返回校验消息
//	unsigned char rxbuff[128]={0};
//	unsigned char rxlen=0;
//	unsigned char i = 0;
//	unsigned char* address = 0;
//	unsigned short len_temp = 0;
//	static unsigned short len_bac = 0;
	

	//=======================配置串口
	if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		//---------------------配置串口
		//api_usart_configuration_NR(ComPortOut,625000,128);
		//---------------------关闭发送--USART3-TX
		//GPIO_Configuration_INF(GPIOB,	GPIO_Pin_10);			//将GPIO相应管脚配置为浮空输入模式----V20170605
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------认证无误
		if(0==api_bq26100slave_get_error_status())		//无错误返回0，有错误返回1
		{
			SetFeederConnect;
			time++;
			if(time>=200)
			{
				static unsigned char led_flag=0;
				time	=	0;				
				if(led_flag)
				{
					SysLedOff;
					led_flag=0;
				}
				else
				{
					SysLedOn;
					led_flag=1;
				}
			}
		}
		//---------------------认证出错:断开后重新连接
		else
		{
			time++;
			if(time<1000)
			{
				SysLedOff;
				SetFeederDisConnect;
			}
			else
			{
				time	=	0;
				SysLedOn;
				SetFeederConnect;
				api_bq26100slave_all_data_default();
			}
		}
	}
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------


#endif
