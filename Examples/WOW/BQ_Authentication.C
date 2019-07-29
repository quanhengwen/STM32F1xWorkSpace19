

#ifdef BQ_Authentication
#include "BQ_Authentication.H"



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

	//PWM_OUT(TIM2,PWM_OUTChannel1,1000000,500);	//PWM设定-20161127版本	占空比1/1000
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
	static unsigned short time = 0;
	static unsigned short time2 = 0;
	static unsigned short len_bac = 0;
	unsigned short len_temp = 0;
	
	
	//api_bq26100slave_server();		//SDQ从机设备配置
	//------------
	if(GetSlavePortCheckInLevel)	//高信号，未接入
	{
		SetFeederDisConnect;
		SysLedOff;
		api_bq26100slave_reset_error_status();
		return ;
	}
	
	//------------离线测试校验：测试模式--定时发送待验证信息
	//BQ_Authentication_Test_Model_OffLine();
	//------------在线应用模式：校验不通过自动重启
	//BQ_Authentication_Auto_Reset_OnLine();
	
	
	
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
static void BQ_Authentication_Test_Model_OffLine(void)
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
	
	
	//bq26100BufferSize //bq26100_sample_data[i][0]
	//=======================配置串口
	if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		api_usart_dma_configurationNR(ComPortOut,625000,128);
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
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
				api_usart_dma_send(ComPortOut,Message,23);		//模拟主机将消息发送给FD验证
				//-----------------点亮LED指示灯
				
			}
			else
			{
				time	=	0;
				bq_test_serial++;
			}
			SysLedOff;
		}
		//---------------------向飞达发送返回摘要的命令
		else if(time==160)
		{
			api_usart_dma_send(ComPortOut,GetDigest,3);		//让FD返回校验消息
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
		rxlen	=	api_usart_dma_receive(ComPortOut,rxbuff);	
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
						//while(1);
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
	
	
	//bq26100BufferSize //bq26100_sample_data[i][0]
	//=======================配置串口
	if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		//---------------------配置串口
		api_usart_dma_configurationNR(ComPortOut,625000,128);
		//---------------------关闭发送--USART3-TX
		GPIO_Configuration_INF(GPIOB,	GPIO_Pin_10);			//将GPIO相应管脚配置为浮空输入模式----V20170605
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------截取飞达返回的数据：如梦飞达有上报摘要，表示已完成一次认证
		//---------------------检查样品摘要，如果样品摘要全为0x00，表示没有找到对应的摘要，需要重新认证
		//---------------------将串口截取的摘要与样品摘要对比，如果不一致，表示SDQ读取时序异常，需要重新认证
		rxlen	=	api_usart_dma_receive(ComPortOut,rxbuff);	
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
				api_usart_dma_send(ComPortOut,Message,23);		//模拟主机将消息发送给FD验证				
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
			api_usart_dma_send(ComPortOut,GetDigest,3);		//让FD返回校验消息
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
	static unsigned short bq_test_serial=0;		//认证消息序号
//	static unsigned short serial=0;
//	static unsigned short num=0;
	unsigned char Message[23]={0x4C,0x14,0xC5,0x14,0xE1,0x78,0x2D,0x56,0xF2,0x07,0x7B,0xB8,0x21,0xB8,0xEC,0x19,0x25,0x78,0x89,0xE9,0xFE,0x57,0x7C};
	unsigned char GetDigest[3]={0x4B,0x00,0xB4};		//让FD返回校验消息
	unsigned char rxbuff[128]={0};
	unsigned char rxlen=0;
	unsigned char i = 0;
	unsigned char* address = 0;
	
	
	//bq26100BufferSize //bq26100_sample_data[i][0]
	//=======================配置串口
	if(usart_cof_flag==0)
	{
		usart_cof_flag	=	1;
		//---------------------配置串口
		api_usart_dma_configurationNR(ComPortOut,625000,128);
		//---------------------关闭发送--USART3-TX
		GPIO_Configuration_INF(GPIOB,	GPIO_Pin_10);			//将GPIO相应管脚配置为浮空输入模式----V20170605
	}
	//=======================认证消息，对比摘要，及认证消息循环
	else
	{
		//---------------------截取飞达返回的数据：如梦飞达有上报摘要，表示已完成一次认证
		//---------------------检查样品摘要，如果样品摘要全为0x00，表示没有找到对应的摘要，需要重新认证
		//---------------------将串口截取的摘要与样品摘要对比，如果不一致，表示SDQ读取时序异常，需要重新认证
		rxlen	=	api_usart_dma_receive(ComPortOut,rxbuff);	
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
				api_usart_dma_send(ComPortOut,Message,23);		//模拟主机将消息发送给FD验证				
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
			api_usart_dma_send(ComPortOut,GetDigest,3);		//让FD返回校验消息
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



//------------------------------------------------------------------------------


#endif
