#include "SDQ.H"	

#include "stm32f10x_exti.h"

#include "STM32_TIM.H"
#include "STM32_EXTI.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

#include "STM32F10x_BitBand.H"

#include 	"CRC.H"

#include "BQ26100DATA.H"

#include	"stdio.h"			//用于printf
#include	"string.h"		//用于printf
#include	"stdarg.h"		//用于获取不确定个数的参数
#include	"stdlib.h"		//malloc动态申请内存空间

#define	SDQPort	GPIOA
#define	SDQPin	GPIO_Pin_6

#define	SetSDQPinLevel	PA6
#define	GetSDQPinLevel	PA6in

#define	SetSDQPinOut		{	GPIOA->CRL&=0xF0FFFFFF;\
													GPIOA->CRL|=0x07000000;\
													/*SetSDQPinLevel=0;*/}		//配置为OD输出模式并拉低
													
#define	SetSDQPinIntrr	{	GPIOA->CRL&=0xF0FFFFFF;\
												GPIOA->CRL|=0x08000000;\
												SetSDQPinLevel=1;}			//配置为上拉输入模式


sdq_def* SDQ_SAMPLE=0;
RCC_ClocksTypeDef bqRCC_ClocksStatus;				//时钟状态---时钟值


/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_sdq_slave_configuration(sdq_def *sSDQ)		//SDQ从机设备配置
{
	SDQ_SAMPLE	=	sSDQ;
	EXTI_Configuration_ITF(SDQPort, SDQPin);		//外部边沿触发中断配置,抢占1，响应1--20171213	
	
	RCC_GetClocksFreq(&bqRCC_ClocksStatus);		//获取时钟参数
	SDQ_SAMPLE->time.timeper=bqRCC_ClocksStatus.SYSCLK_Frequency/8000000;
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
void api_sdq_slave_server(void)		//SDQ从机设备服务
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
static void api_sdq_process(void)
{
	//unsigned long fitime=0;
	//----滤波
	SDQ_SAMPLE->time.time1=SysTick_ReLoad();			//设置SYSTICK初始值
	SDQ_SAMPLE->data.intrrcout++;									//中断次数计数
	if(0==GetSDQPinLevel)													//低电平
	{
		api_sdq_receive_process();
		api_sdq_data_process();
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
static void api_sdq_data_process(void)
{
	//跳过ID一字节，命令一字节，地址两字节
	if(SDQ_SAMPLE->data.receivelen>4)
	{
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
unsigned char api_sdq_receive_process(void)
{
	unsigned char bit=0;
	
	//-------------------应答状态
	if((sdq_master_readCRC==SDQ_SAMPLE->status)|(sdq_master_read==SDQ_SAMPLE->status)|(sdq_master_read_digest==SDQ_SAMPLE->status))
	{
		api_sdq_send_byte();		
		return 0;
	}
	//-------------------位接收		
	bit=api_sdq_get_bit();
	if(0xFF==bit)
	{
		api_sdq_set_start();
		return 0;
	}
	if(0x80==bit)
	{
		SDQ_SAMPLE->data.receivelen=0;
		return 0;
	}
	//-------------------字节接收
	if(1==api_sdq_get_byte(bit))	//接收完一字节
	{
		
		if(SDQ_SAMPLE->data.receivelen<255)
		{
			SDQ_SAMPLE->data.receive[SDQ_SAMPLE->data.receivelen]=SDQ_SAMPLE->data.temp;
			SDQ_SAMPLE->data.receivelen+=1;
		}
		SDQ_SAMPLE->data.bitlen	=	0;
		SDQ_SAMPLE->data.temp		=	0;
		
		//-------------------获取ROM命令
		if(1==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->RomCmd	=	(sdq_rom_cmd)SDQ_SAMPLE->data.receive[0];
			return 0;
		}
		//-------------------获取RAM命令
		if(2==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->MemCmd	=	(sdq_mem_cmd)SDQ_SAMPLE->data.receive[1];
			return 0;
		}
		
		
		//------------------主机读取摘要
		if(sdq_mem_read_digest==SDQ_SAMPLE->MemCmd)
		{
			if(4==SDQ_SAMPLE->data.receivelen)
			{
				unsigned short i = 0;
				for(i=0;i<2500;i++)
				{
					if(0==memcmp(SDQ_SAMPLE->data.message,bq26100_sample_data[i][0],20))
					{
						memcpy(SDQ_SAMPLE->data.digest,bq26100_sample_data[i][1],20);
						
						SDQ_SAMPLE->status	=	sdq_master_readCRC;
						SDQ_SAMPLE->data.digestlen	=	0;
						SDQ_SAMPLE->data.ackbitlen	=	0;
						SDQ_SAMPLE->data.ackdata	=		CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],SDQ_SAMPLE->data.receivelen-1);						
						break;
					}
				}
			}
			return 0;
		}
		//------------------主机写消息
		if(sdq_mem_write_message==SDQ_SAMPLE->MemCmd)
		{
			//------------------4为ROM命令，RAM命令和两字节地址长度
			if(5<=SDQ_SAMPLE->data.receivelen)
			{
				if(20>SDQ_SAMPLE->data.messgaelen)
				{
					
					SDQ_SAMPLE->data.message[SDQ_SAMPLE->data.messgaelen]=SDQ_SAMPLE->data.receive[SDQ_SAMPLE->data.receivelen-1];
					SDQ_SAMPLE->data.messgaelen++;
					
					//每接收一字节后需要应答一次CRC
					SDQ_SAMPLE->status	=	sdq_master_readCRC;
					SDQ_SAMPLE->data.ackbitlen	=	0;
					SDQ_SAMPLE->data.ackdata		=	CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],SDQ_SAMPLE->data.receivelen-1);
				}
			}
			return 0;
		}		
	}
	return 0;
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
unsigned char api_sdq_set_bit(unsigned char bit)
{	
	SetSDQPinOut;
	SetSDQPinLevel	=	bit;
	SysTick_DeleyuS(50);				//SysTick延时nuS
	SetSDQPinIntrr;
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
unsigned char api_sdq_send_byte(void)
{	
	api_sdq_set_bit(SDQ_SAMPLE->data.ackdata&0x01);
	
	SDQ_SAMPLE->data.ackdata		>>=	1;
	SDQ_SAMPLE->data.ackbitlen	+=	1;
	if(8<=SDQ_SAMPLE->data.ackbitlen)
	{
		//---------------------写消息
		if(sdq_mem_write_message==SDQ_SAMPLE->MemCmd)		//写消息命令：写一字节，读一次CRC和原字节
		{
			if(sdq_master_readCRC==SDQ_SAMPLE->status)
			{
				SDQ_SAMPLE->data.ackbitlen	=	0;
				SDQ_SAMPLE->data.ackdata		=	SDQ_SAMPLE->data.message[SDQ_SAMPLE->data.messgaelen-1];
				SDQ_SAMPLE->status	=	sdq_master_read;
			}
			else if(sdq_master_read==SDQ_SAMPLE->status)
			{
				SDQ_SAMPLE->data.ackbitlen	=	0;
				SDQ_SAMPLE->data.ackdata		=	0x00;
				SDQ_SAMPLE->status	=	sdq_master_write;					
			}					
		}
		//---------------------主机读取摘要
		else if(sdq_mem_read_digest==SDQ_SAMPLE->MemCmd)
		{	
			//------------开始发送摘要
			if(20>SDQ_SAMPLE->data.digestlen)
			{	
				SDQ_SAMPLE->status	=	sdq_master_read_digest;				
				SDQ_SAMPLE->data.ackdata	=	SDQ_SAMPLE->data.digest[SDQ_SAMPLE->data.digestlen];
				SDQ_SAMPLE->data.ackbitlen	=	0;
				SDQ_SAMPLE->data.digestlen+=1;
			}
			//------------发送CRC
			else if(20==SDQ_SAMPLE->data.digestlen)
			{				
				SDQ_SAMPLE->data.ackdata	=	CRC8_8541_lsb(SDQ_SAMPLE->data.digest,20);
				SDQ_SAMPLE->status	=	sdq_master_readCRC;
				SDQ_SAMPLE->data.ackbitlen	=	0;
				SDQ_SAMPLE->data.digestlen+=1;				
			}
			//------------发送完成
			else
			{
				SDQ_SAMPLE->data.digestlen	=	0;
				SDQ_SAMPLE->status	=	sdq_ilde;
				SDQ_SAMPLE->data.digestlen	=	0;
				SDQ_SAMPLE->MemCmd	=	sdq_mem_idle;
			}
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
unsigned char api_sdq_get_bit(void)
{
	unsigned char flag=0;
	
	SDQ_SAMPLE->time.count=0;
	SDQ_SAMPLE->time.time_sys=0;	
	while(0==GetSDQPinLevel)
	{
		if(SysTick_GetVal()<2)
		{
			if(0==flag)
			{
				flag=1;
				SDQ_SAMPLE->time.time_sys+=1;
			}
		}
		else
		{
			flag=0;
		}
	}
	SDQ_SAMPLE->time.time2=SysTick_GetVal();							//获取当前SysTick计数值
	
	//------开始前时间少于500，去掉一次计数
	//------计算时长:9000为SYSTICK设置为1us时的重装载值
//	if(SDQ_SAMPLE->time.time2<10)
//	{
//		if(SDQ_SAMPLE->time.time_sys>0)
//			SDQ_SAMPLE->time.time_sys-=1;
//	}
	
	SDQ_SAMPLE->time.time_sys=SDQ_SAMPLE->time.time_sys*9000+SDQ_SAMPLE->time.time1-SDQ_SAMPLE->time.time2;

	//------转换为us
	SDQ_SAMPLE->time.count=SDQ_SAMPLE->time.time_sys/SDQ_SAMPLE->time.timeper;
	
	//------
	//------启动条件：主机拉低信号480us-960us
	if(SDQ_SAMPLE->time.count>400)
	{	
		return 0xFF;
	}
	//------主机写0:拉低信号60us-120us
	else if(SDQ_SAMPLE->time.count>50)
	{
		return 0;
	}
	//------主机写1:拉低信号大于1us，总时间60us
	else if((SDQ_SAMPLE->time.count<15))
	{
		return 1;
	}
	return 0x80;
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
unsigned char api_sdq_get_byte(unsigned char bit)
{
	if(0==bit)
	{
		SDQ_SAMPLE->data.temp>>=1;
		SDQ_SAMPLE->data.temp&=0x7F;		
		SDQ_SAMPLE->data.bitlen+=1;
	}
	//------------数据接收或者是读取时序
	else if(1==bit)
	{
		SDQ_SAMPLE->data.temp>>=1;
		SDQ_SAMPLE->data.temp|=0x80;		
		SDQ_SAMPLE->data.bitlen+=1;
	}
	if((SDQ_SAMPLE->data.bitlen>=8))
	{
		return 1;		//接收完一字节
	}
	return 0;			//未接收完
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
unsigned char api_sdq_set_start(void)
{	
	//------启动条件：主机拉低信号480us-960us

	//----------------status
	SDQ_SAMPLE->status	=	sdq_start;
	
	//----------------time
	SDQ_SAMPLE->time.high	=	0;
	SDQ_SAMPLE->time.low	=	0;
	
	
	//----------------data
	//SDQ_SAMPLE->data.intrrcout=0;
	SDQ_SAMPLE->data.startcout++;
	SDQ_SAMPLE->data.bitlen	=	0;
	SDQ_SAMPLE->data.messgaelen=0;
	SDQ_SAMPLE->data.receivelen=0;
	SDQ_SAMPLE->data.temp	=	0;
	
	//----------------RomCmd
	SDQ_SAMPLE->RomCmd	=	sdq_rom_idle;
	
	//----------------MemCmd
	SDQ_SAMPLE->MemCmd	=	sdq_mem_idle;
	
	
	//------等待15us-60us后从机拉低总线60us-240us作为对主机的应答
	SetSDQPinOut;
	SetSDQPinLevel=1;
	SysTick_DeleyuS(20);				//SysTick延时nuS
	SetSDQPinLevel=0;
	SysTick_DeleyuS(100);				//SysTick延时nuS
	SetSDQPinIntrr;
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
//unsigned char api_sdq_get_level(void)
//{
//	return SDQ_SAMPLE->port.SDQ_Port->IDR & (SDQ_SAMPLE->port.SDQ_Pin);
//	//return GPIO_ReadInputDataBit(SDQ_SAMPLE->port.SDQ_Port, SDQ_SAMPLE->port.SDQ_Pin);
//}
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
void EXTI9_5_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line6);
	//=======================高电平:上升沿
	if(0==GetSDQPinLevel)	//低电平
	{		
		api_sdq_process();
	}
	//=======================低电平:下降沿
	else
	{
		//----------启动总线
		//if(SDQ_SAMPLE->time.count>1000)
		//{
			//SDQ_SAMPLE->status=sdq_start;
		//}
	}
	EXTI_ClearITPendingBit(EXTI_Line6);
	//EXTI_ClearITPendingBit(EXTI_Line6);
}



