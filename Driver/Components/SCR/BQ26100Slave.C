#include "BQ26100Slave.H"	

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

#define	BQ26100slavePort					GPIOA
#define	BQ26100slavePin						GPIO_Pin_6

#define	SetBQ26100slavePinLevel		PA6
#define	GetBQ26100slavePinLevel		PA6in

#define	SetBQ26100slavePinOut		{	/*SetBQ26100slavePinLevel=0;*/\
																	GPIOA->CRL&=0xF3FFFFFF;\
																	GPIOA->CRL|=0x03000000;\
																	SetBQ26100slavePinLevel=0;}		//配置为OD输出模式并拉低
													
#define	SetBQ26100slavePinIntrr	{	GPIOA->CRL&=0xF0FFFFFF;\
																	GPIOA->CRL|=0x08000000;\
																	/*SetBQ26100slavePinLevel=1;*/}			//配置为上拉输入模式


bq26100slave_def* SDQ_SAMPLE=0;
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
void api_bq26100slave_configuration(bq26100slave_def *sSDQ)		//SDQ从机设备配置
{
	SDQ_SAMPLE	=	sSDQ;
	EXTI_Configuration_ITF(BQ26100slavePort, BQ26100slavePin);		//外部边沿触发中断配置,抢占1，响应1--20171213	
	
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
void api_bq26100slave_server(void)		//SDQ从机设备服务
{
	static unsigned short time = 0;
	//====================有错误
	if(api_bq26100slave_get_error_status())		//无错误返回0，有错误返回1
	{
		if(time++>1000)
		{
			api_bq26100slave_reset_error_status();
		}
	}
	else
	{
		time = 0;
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
unsigned char api_bq26100slave_process(void)
{
	//unsigned long fitime=0;
	//----滤波
	SDQ_SAMPLE->time.time1=SysTick_ReLoad();			//设置SYSTICK初始值
	SDQ_SAMPLE->data.intrrcout++;									//中断次数计数
	if(SDQ_SAMPLE->data.receivelen<4)
	{
		api_bq26100slave_Initialize();
	}
	else
	{
		//SetBQ26100slavePinLevel=0;
		api_bq26100slave_command_process();
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
unsigned char api_bq26100slave_Initialize(void)
{
	unsigned char byted = api_bq26100slave_receive_byte();						//返回0--未读完一字节，返回1--读完一字节，返回0xFF--复位信号
	//------------------------每次命令先传rom、ram、地址低位、地址高位
	//---------------------等待接收完一字节
	if(1==byted)	//未接收完一字节
	{
		//-------------------获取ROM命令
		if(1==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->RomCmd	=	(sdq_rom_cmd)SDQ_SAMPLE->data.receive[0];
		}
		//-------------------获取RAM命令
		else if(2==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->MemCmd	=	(sdq_mem_cmd)SDQ_SAMPLE->data.receive[1];
			//api_bq26100slave_MenCmd();
		}
		//-------------------获取地址低字节
		else if(3==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->data.addressL	=	(sdq_mem_cmd)SDQ_SAMPLE->data.receive[2];
		}
		//-------------------获取地址高字节
		else if(4==SDQ_SAMPLE->data.receivelen)
		{
			SDQ_SAMPLE->data.addressH	=	(sdq_mem_cmd)SDQ_SAMPLE->data.receive[3];
			api_bq26100slave_MenCmd();
		}
		return 1;
	}
	//---------------------复位信号
	else if(0xFF == byted)
	{
		api_bq26100slave_start();
		api_bq26100slave_default();
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
unsigned char api_bq26100slave_command_process(void)
{
	unsigned char time = 0;
	//========================数据接收或者读取:发送CRC，message原值、status、digest
	//------------------主机写消息
	if(sdq_mem_write_message==SDQ_SAMPLE->MemCmd)
	{
		api_bq26100slave_write_message();
	}
	//------------------主机读摘要
	else if((sdq_mem_read_digest==SDQ_SAMPLE->MemCmd)&&(20==SDQ_SAMPLE->data.Message.MessageLen))
	{
		api_bq26100slave_read_digest();
	}
	//------------------主机读控制寄存器
	else if((sdq_mem_read_control==SDQ_SAMPLE->MemCmd)&&(20==SDQ_SAMPLE->data.Message.MessageLen))
	{
		api_bq26100slave_read_control();
	}
	//------------------主机写控制寄存器
	else if((sdq_mem_write_control==SDQ_SAMPLE->MemCmd)&&(20==SDQ_SAMPLE->data.Message.MessageLen))
	{
		api_bq26100slave_write_control();
	}
	while(0==GetBQ26100slavePinLevel)	//低电平
	{
		SysTick_DeleyuS(1);				//SysTick延时nuS
		if(time++>200)
			api_bq26100slave_start();
	}
	return 1;
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
unsigned char api_bq26100slave_MenCmd(void)
{
	//------------------主机写消息
	if(sdq_mem_write_message==SDQ_SAMPLE->MemCmd)
	{
		//---------------Message
		SDQ_SAMPLE->data.Message.MessageLen	=	0;
		//---------------
		SDQ_SAMPLE->status					=	sdq_master_write;
		//---------------
		SDQ_SAMPLE->data.CtlFlag		=	0;
		//---------------
		SDQ_SAMPLE->data.Digest.Serial	=	0;
		
		
		//---------------Digest
		SDQ_SAMPLE->data.Digest.Flag.Auth	=	0;
		//---------------Ctrl
		SDQ_SAMPLE->data.Ctrl.Read.Times	=	0;
		
		SDQ_SAMPLE->data.Ctrl.Flag.Start	=	0;
	}
	//------------------主机读摘要
	else if(sdq_mem_read_digest==SDQ_SAMPLE->MemCmd)
	{		
		SDQ_SAMPLE->status							=	sdq_master_readCRC;
		SDQ_SAMPLE->data.Digest.Flag.Start	=	0;
		SDQ_SAMPLE->data.Digest.Serial	=	0;
	}
	//------------------主机读控制寄存器
	else if(sdq_mem_read_control==SDQ_SAMPLE->MemCmd)
	{
		SDQ_SAMPLE->data.Ctrl.Read.Serial	=	0;
		SDQ_SAMPLE->status					=	sdq_master_readCRC;
	}
	//------------------主机写控制寄存器
	else if(sdq_mem_write_control==SDQ_SAMPLE->MemCmd)
	{		
		SDQ_SAMPLE->data.Ctrl.Write.Serial	=	0;
		SDQ_SAMPLE->data.Ctrl.Flag.Start		=	0;
		
		//---------------Digest
		SDQ_SAMPLE->data.Digest.Flag.Auth	=	1;
		
		SDQ_SAMPLE->status									=	sdq_master_write;
	}
	
	//SDQ_SAMPLE->data.SendBitLen	=	0;
	return 1;
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
unsigned char api_bq26100slave_read_digest(void)
{
	//====================第一步:主机读取RAM命令和两字节地址校验
	if(sdq_master_readCRC	==	SDQ_SAMPLE->status)
	{
		if(0==SDQ_SAMPLE->data.Digest.Flag.Start)
		{
			//---------------------RAM命令和两字节地址校验
			SDQ_SAMPLE->data.SendByte	=		CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],3);
			SDQ_SAMPLE->data.Digest.Flag.Start	=	1;
		}
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{				
			//SDQ_SAMPLE->data.SendByte	=	SDQ_SAMPLE->data.message[SDQ_SAMPLE->data.messgaelen-1];
			SDQ_SAMPLE->status	=	sdq_master_read;
			SDQ_SAMPLE->data.Digest.Flag.Start	=	0;
		}
		return 1;
	}
	//====================第二步:返回消息摘要
	else if(sdq_master_read	==	SDQ_SAMPLE->status)
	{
		//----------------未启动传输
		if(0==SDQ_SAMPLE->data.Digest.Flag.Start)		//未设置消息
		{	
			//--------------第一次返回摘要为原message消息----未设置控制位
			if(0==SDQ_SAMPLE->data.Digest.Flag.Auth)		//未启动转换:发回message
			{
				memcpy(SDQ_SAMPLE->data.Digest.Buffer,SDQ_SAMPLE->data.Message.Buffer,20);
			}
			//--------------第二次返回摘要为转换后的摘要消息--已设置控制位，如果未匹配到摘要则做异常处理
			else
			{
				unsigned short i = 0;
				for(i=0;i<bq26100BufferSize;i++)
				{
					if(0==memcmp(SDQ_SAMPLE->data.Message.Buffer,bq26100_sample_data[i][0],20))
					{
						memcpy(SDQ_SAMPLE->data.Digest.Buffer,bq26100_sample_data[i][1],20);
						break;
					}
				}
				//------------------------未匹配到数据
				if(i>=bq26100BufferSize)
				{
					api_bq26100slave_digest_match_error();
					return 0;
				}
			}
			//--------------启动发送
			SDQ_SAMPLE->data.Digest.Flag.Start	=	1;
			SDQ_SAMPLE->data.Digest.Serial			=	0;
			SDQ_SAMPLE->data.SendByte		=	SDQ_SAMPLE->data.Digest.Buffer[SDQ_SAMPLE->data.Digest.Serial];				
		}
		//----------------已启动传输
		//===========================发送数据
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{				
			//SDQ_SAMPLE->data.SendByte	=	SDQ_SAMPLE->data.message[SDQ_SAMPLE->data.messgaelen-1];
			SDQ_SAMPLE->data.Digest.Serial	+=	1;		//已发送数量计数
			if(20>SDQ_SAMPLE->data.Digest.Serial)
			{
				SDQ_SAMPLE->data.SendByte		=	SDQ_SAMPLE->data.Digest.Buffer[SDQ_SAMPLE->data.Digest.Serial];
			}
			//--------------发送所有的Digest数据校验
			else if(20==SDQ_SAMPLE->data.Digest.Serial)
			{
				SDQ_SAMPLE->data.SendByte		=	CRC8_8541_lsb(SDQ_SAMPLE->data.Digest.Buffer,20);
			}
			else	//发送完摘要和CRC
			{
				SDQ_SAMPLE->status							=	sdq_ilde;
				SDQ_SAMPLE->data.receivelen	=	0;
				return 1;
			}					
		}
		SDQ_SAMPLE->status					=	sdq_master_read;
		return 1;
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
unsigned char api_bq26100slave_write_message(void)
{
	//====================第一步:主机写数据
	if(sdq_master_write	==	SDQ_SAMPLE->status)
	{
		if(1==api_bq26100slave_receive_byte())	//未接收完一字节
		{
			if(SDQ_SAMPLE->data.Message.MessageLen<20)	//message合法字节数据只能为20
			{
				SDQ_SAMPLE->data.Message.Buffer[SDQ_SAMPLE->data.Message.MessageLen]	=	SDQ_SAMPLE->data.RcvByte;
				SDQ_SAMPLE->data.Message.MessageLen		+=	1;
				
				//----------------主机发送完第一字节message后需要读取"RAM命令和两字节地址长度和数据"校验
				if(1==SDQ_SAMPLE->data.Message.MessageLen)
				{					
					SDQ_SAMPLE->data.SendByte		=	CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],SDQ_SAMPLE->data.receivelen-1);
				}
				//----------------主机发完第二条message后的校验方式:当前地址与当前数据异或出结果再求对应结果的CRC
				else
				{
					unsigned char temp=(SDQ_SAMPLE->data.Message.MessageLen-1)^SDQ_SAMPLE->data.Message.Buffer[SDQ_SAMPLE->data.Message.MessageLen-1];
					SDQ_SAMPLE->data.SendByte		=	CRC8_8541_lsb(&temp,1);
				}
				//----------------主机每发送完一字节数据后需要读取CRC
				SDQ_SAMPLE->status	=	sdq_master_readCRC;				
				return 1;
			}
			//协议失败:发送数据个数超过20个字节
			else
			{
			}
			return 0;		//协议失败:发送数据个数超过20个字节
		}
		return 1;
	}
	//====================主机读CRC和数据时，启动bit只能为1
	//====================第二步:主机读CRC，主机读完CRC后读回原字节数据
	else if(sdq_master_readCRC	==	SDQ_SAMPLE->status)
	{
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{
			SDQ_SAMPLE->data.SendByte	=	SDQ_SAMPLE->data.Message.Buffer[SDQ_SAMPLE->data.Message.MessageLen-1];
			SDQ_SAMPLE->status	=	sdq_master_read;
			return 1;
		}
	}
	//====================第三步:主机读回数据,读回数据后主机继续发送下一字节数据，直到主机发送完20个字节
	else if(sdq_master_read	==	SDQ_SAMPLE->status)
	{
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{
			if(20	<=	SDQ_SAMPLE->data.Message.MessageLen)
			{
				SDQ_SAMPLE->status	=	sdq_ilde;						//空闲:主机发送完所有的message
				SDQ_SAMPLE->data.receivelen	=	0;
			}
			else
			{
				SDQ_SAMPLE->status	=	sdq_master_write;		//主机继续发送下一字节数据
			}
		}
		return 1;
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
unsigned char api_bq26100slave_read_control(void)
{
	//----------------第一步:上传RAM,AddressL,AddressH校验
	//----------------第二步:上传00地址状态值
	//----------------第三步:上传01地址状态值
	//----------------第四步:上传(RAM,AddressL,AddressH)&(RAM,AddressL,AddressH校验结果)&(00地址内容)&(01地址内容)的CRC校验
	//====================第一步:主机读取RAM命令和两字节地址校验
	if(sdq_master_readCRC	==	SDQ_SAMPLE->status)
	{
		if(0==SDQ_SAMPLE->data.Ctrl.Flag.Start)
		{
			//---------------------RAM命令和两字节地址校验
			if(0	==	SDQ_SAMPLE->data.Ctrl.Read.Times)
			{				
				SDQ_SAMPLE->data.Ctrl.Read.CrcData	=	CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],3);				
				SDQ_SAMPLE->data.Ctrl.Read.Ctrl0		=	0x04;
				SDQ_SAMPLE->data.Ctrl.Read.Ctrl1		=	0xA1;
			}
			else if(1	==	SDQ_SAMPLE->data.Ctrl.Read.Times)
			{				
				SDQ_SAMPLE->data.Ctrl.Read.CrcData	=	CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],3);
				SDQ_SAMPLE->data.Ctrl.Read.Ctrl0		=	0x06;
				SDQ_SAMPLE->data.Ctrl.Read.Ctrl1		=	0xA1;
			}
			SDQ_SAMPLE->data.SendByte						=	SDQ_SAMPLE->data.Ctrl.Read.CrcData;
			SDQ_SAMPLE->data.Ctrl.Flag.Start		=	1;
		}
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{	
			if(0 == SDQ_SAMPLE->data.Ctrl.Read.Serial)		//未传输数据
			{
				SDQ_SAMPLE->data.SendByte	=	SDQ_SAMPLE->data.Ctrl.Read.Ctrl0;
				SDQ_SAMPLE->status	=	sdq_master_read;
			}
			else if(2 == SDQ_SAMPLE->data.Ctrl.Read.Serial)	//发送完最后一个CRC
			{
				SDQ_SAMPLE->status	=	sdq_ilde;
				SDQ_SAMPLE->data.Ctrl.Flag.Start	=	0;
				SDQ_SAMPLE->data.Ctrl.Read.Times	+=1;
				SDQ_SAMPLE->data.receivelen	=	0;
			}
		}
		return 1;
	}
	//----------------主机读数据
	else if(sdq_master_read	==	SDQ_SAMPLE->status)
	{
		if(1==api_bq26100slave_send_byte())			//返回1表示发送完一字节
		{	
			SDQ_SAMPLE->data.Ctrl.Read.Serial	+=1;
			if(1 == SDQ_SAMPLE->data.Ctrl.Read.Serial)
			{
				SDQ_SAMPLE->data.SendByte	=	SDQ_SAMPLE->data.Ctrl.Read.Ctrl1;
				SDQ_SAMPLE->status	=	sdq_master_read;
			}
			else if(2 == SDQ_SAMPLE->data.Ctrl.Read.Serial)
			{
				unsigned char buffer[6]={0};
				memcpy(buffer,&SDQ_SAMPLE->data.receive[1],3);
				buffer[3]	=	SDQ_SAMPLE->data.Ctrl.Read.CrcData;
				buffer[4]	=	SDQ_SAMPLE->data.Ctrl.Read.Ctrl0;
				buffer[5]	=	SDQ_SAMPLE->data.Ctrl.Read.Ctrl1;
				SDQ_SAMPLE->data.SendByte	=	CRC8_8541_lsb(buffer,6);
				SDQ_SAMPLE->status	=	sdq_master_readCRC;
			}
		}
		return 1;
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
unsigned char api_bq26100slave_write_control(void)
{
	//====================第一步:主机写数据
	if(sdq_master_write	==	SDQ_SAMPLE->status)
	{
		//------------------第一步:接收第一个数据
		if(1==api_bq26100slave_receive_byte())	//未接收完一字节
		{
			SDQ_SAMPLE->data.Ctrl.Write.Serial+=1;
			//------------------主机发完第一个状态寄存器：下一步返回CRC
			if(1 == SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				//----------------主机发送第一个字节
				SDQ_SAMPLE->data.Ctrl.Write.Ctrl0R	=	SDQ_SAMPLE->data.RcvByte;				
				//----------------从机需要返回的状态值
				SDQ_SAMPLE->data.Ctrl.Write.Ctrl0T	=	0x06;
				//----------------主机发送完第一字节控制寄存器值后需要读取"RAM命令和两字节地址长度和数据"校验
				SDQ_SAMPLE->data.SendByte		=	CRC8_8541_lsb(&SDQ_SAMPLE->data.receive[1],SDQ_SAMPLE->data.receivelen-1);
				//----------------下一条操作为向主机返回CRC
				SDQ_SAMPLE->status	=	sdq_master_readCRC;
			}
			//------------------主机发完第二个状态寄存器：下一步返回CRC
			else if(2 == SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				unsigned char temp	=	0;
				SDQ_SAMPLE->data.Ctrl.Write.Ctrl1	=	SDQ_SAMPLE->data.RcvByte;
				//----------------当前地址与当前数据异或出结果再求对应结果的CRC
				temp	=	0x01^SDQ_SAMPLE->data.Ctrl.Write.Ctrl1;
				//----------------获取CRC
				SDQ_SAMPLE->data.SendByte		=	CRC8_8541_lsb(&temp,1);
				//----------------下一条操作为向主机返回CRC
				SDQ_SAMPLE->status	=	sdq_master_readCRC;
			}
			return 1;
		}
	}
	//====================向主机返回CRC
	//--------------------第一步返回CRC：原状态为向主机发送CRC，下一步为向主机发送控制寄存器状态
	else if(sdq_master_readCRC==SDQ_SAMPLE->status)
	{
		//----------------------发送CRC
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{
			//------------------主机读完第一个CRC：下一步读取第一个状态寄存器值
			if(1 ==	SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				//----------------控制寄存器0的值
				SDQ_SAMPLE->data.SendByte		=	SDQ_SAMPLE->data.Ctrl.Write.Ctrl0T;
				//----------------等待主机读取数据
				SDQ_SAMPLE->status	=	sdq_master_read;
			}
			//------------------主机读完第二个状态值的CRC：下一步读取第二个状态寄存器
			else if(2 ==	SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				//----------------控制寄存器0的值
				SDQ_SAMPLE->data.SendByte		=	SDQ_SAMPLE->data.Ctrl.Write.Ctrl1;
				//----------------等待主机读取数据
				SDQ_SAMPLE->status	=	sdq_master_read;
			}
			//------------------
			else
			{
				SDQ_SAMPLE->status	=	sdq_ilde;		//主机继续发送下一字节数据
				//SDQ_SAMPLE->data.receivelen	=	0;
			}
			return 1;
		}
		return 1;
	}
	//====================第三步:返回寄存器值
	else if(sdq_master_read==SDQ_SAMPLE->status)
	{
		//----------------------发送数据
		if(1==api_bq26100slave_send_byte())	//返回1表示发送完一字节
		{
			//------------------主机读完第一个状态寄存器：下一步主机发送第二个状态寄存器值
			if(1 ==	SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				//----------------等待主机发送寄存器1数据
				SDQ_SAMPLE->status	=	sdq_master_write;
			}
			//------------------主机读完第二个状态寄存器：完成整个写控制寄存器操作
			else if(2 ==	SDQ_SAMPLE->data.Ctrl.Write.Serial)
			{
				//----------------等待主机发送寄存器1数据：完成
				SDQ_SAMPLE->status	=	sdq_ilde;
				SDQ_SAMPLE->data.receivelen	=	0;
			}
		}
		return 1;
	}
	return 1;
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
unsigned char api_bq26100slave_digest_match_error(void)
{
	SDQ_SAMPLE->status	=	sdq_error;
	//api_bq26100slave_default();
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
unsigned char api_bq26100slave_get_error_status(void)
{
	//==================无错误:返回0
	if(sdq_error !=	SDQ_SAMPLE->status)
	{
		return 0;
	}
	//==================有错误:返回1
	else
	{
		return 1;
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
unsigned char api_bq26100slave_reset_error_status(void)
{
	SDQ_SAMPLE->status	=	sdq_ilde;
	return 1;
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
unsigned short api_bq26100slave_get_receivelen(void)
{
	return SDQ_SAMPLE->data.receivelen;
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
unsigned char api_bq26100slave_send_bit(unsigned char bit)
{	
	SetBQ26100slavePinOut;
	SetBQ26100slavePinLevel	=	bit;
	SysTick_DeleyuS(30);				//SysTick延时nuS
	SetBQ26100slavePinIntrr;
	return 1;
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
unsigned char api_bq26100slave_receive_bit(void)
{
	unsigned char flag=0;
	unsigned char bit = 0x80;
	//-----------------主机写数据或者空闲状态时
	if((sdq_start==SDQ_SAMPLE->status)||(sdq_ilde==SDQ_SAMPLE->status)||(sdq_master_write==SDQ_SAMPLE->status))
	{
		SDQ_SAMPLE->time.count=0;
		SDQ_SAMPLE->time.time_sys=0;	
		while(0==GetBQ26100slavePinLevel)
		{
			if(SysTick_GetVal()<2)
			{
				if(0==flag)
				{
					flag=1;
					SDQ_SAMPLE->time.time_sys+=1;
					break;
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
		
		SDQ_SAMPLE->time.time_sys=SDQ_SAMPLE->time.time_sys*9000+SDQ_SAMPLE->time.time1-SDQ_SAMPLE->time.time2;

		//------转换为us
		SDQ_SAMPLE->time.count=SDQ_SAMPLE->time.time_sys/SDQ_SAMPLE->time.timeper;
		
		//------
		//------启动条件：主机拉低信号480us-960us
		if(SDQ_SAMPLE->time.count>350)
		{	
			bit	= 0xFF;
		}
		//------主机写0:拉低信号60us-120us
		else if(SDQ_SAMPLE->time.count>50)
		{
			bit	= 0;
		}
		//------主机写1或者启动bit位读:拉低信号大于1us，总时间60us
		else if((SDQ_SAMPLE->time.count<15))
		{
			bit	= 1;
		}
	}
	return bit;		//其它未识别时间
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
unsigned char api_bq26100slave_send_byte(void)
{	
	api_bq26100slave_send_bit(SDQ_SAMPLE->data.SendByte&0x01);
	
	//=======================右移一位:低位先传
	SDQ_SAMPLE->data.SendByte		>>=	1;
	//=======================传输计数
	SDQ_SAMPLE->data.SendBitLen	+=	1;
	
	if(8<=SDQ_SAMPLE->data.SendBitLen)
	{
		SDQ_SAMPLE->data.SendBitLen	=	0;
		return 1;		//发送完一字节数据
	}	
	return 0;		//未发送完
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
unsigned char api_bq26100slave_receive_byte(void)
{
	unsigned char bit=api_bq26100slave_receive_bit();
	//--------------------右移1位:低位先传输	
	SDQ_SAMPLE->data.RcvByte>>=1;
	if(0==bit)
	{		
		SDQ_SAMPLE->data.RcvByte&=0x7F;		
	}
	else if(1==bit)
	{
		SDQ_SAMPLE->data.RcvByte|=0x80;		
	}
	else if(0xFF == bit)
	{
		return 0xFF;
	}	
	//--------------------字节接收计数
	SDQ_SAMPLE->data.RcvBitlen+=1;
	//--------------------判断数据有无接收完一字节
	if(8<=SDQ_SAMPLE->data.RcvBitlen)
	{
		SDQ_SAMPLE->data.RcvBitlen	=	0;
		//========================将接收到的数据保存	
		if(SDQ_SAMPLE->data.receivelen<sdq_receive_buffer_size)
		{
			SDQ_SAMPLE->data.receive[SDQ_SAMPLE->data.receivelen]=SDQ_SAMPLE->data.RcvByte;
			SDQ_SAMPLE->data.receivelen+=1;
			return 1;		//接收完一字节
		}
		//------------------------缓存满
		else
		{
			return 0;
		}		
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
unsigned char api_bq26100slave_start(void)
{	
	//------启动条件：主机拉低信号480us-960us
	//------等待15us-60us后从机拉低总线60us-240us作为对主机的应答
	SetBQ26100slavePinOut;
	SetBQ26100slavePinLevel=1;
	SysTick_DeleyuS(20);				//SysTick延时nuS
	SetBQ26100slavePinLevel=0;
	SysTick_DeleyuS(120);				//SysTick延时nuS
	SetBQ26100slavePinIntrr;
	
	return 1;
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
unsigned char  api_bq26100slave_default(void)
{
//	//----------------time
//	SDQ_SAMPLE->time.high	=	0;
//	SDQ_SAMPLE->time.low	=	0;	
	
	//----------------data
	//SDQ_SAMPLE->data.intrrcout=0;
	SDQ_SAMPLE->data.startcout++;
	SDQ_SAMPLE->data.receivelen=0;
	SDQ_SAMPLE->data.addressL	=	0;
	SDQ_SAMPLE->data.addressH	=	0;
	//SDQ_SAMPLE->data.SendBitLen	=	0;
	SDQ_SAMPLE->data.SendByte	=	0;
	SDQ_SAMPLE->data.RcvBitlen	=	0;
	SDQ_SAMPLE->data.RcvByte	=	0;

	
	//---------------Message
	//SDQ_SAMPLE->data.Message.MessageLen=0;
	
	//---------------Digest
	SDQ_SAMPLE->data.Digest.Serial=0;	
	
	//----------------RomCmd
	SDQ_SAMPLE->RomCmd	=	sdq_rom_idle;
	
	//----------------MemCmd
	SDQ_SAMPLE->MemCmd	=	sdq_mem_idle;
	
	//----------------status
	SDQ_SAMPLE->status	=	sdq_start;
	return 1;
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
void EXTI9_5_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line6);
	//=======================高电平:上升沿
	if(0==GetBQ26100slavePinLevel)	//低电平
	{	
//		if((SDQ_SAMPLE->status==sdq_master_read)||(SDQ_SAMPLE->status==sdq_master_readCRC))
//			SetBQ26100slavePinOut;
		api_bq26100slave_process();
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



