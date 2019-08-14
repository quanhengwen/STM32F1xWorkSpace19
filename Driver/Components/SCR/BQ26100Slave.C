#include "BQ26100Slave.H"	

#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"

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

#define	SetBQ26100slavePinOutL		{	/*SetBQ26100slavePinLevel=0;*/\
																	GPIOA->CRL&=0xF7FFFFFF;\
																	GPIOA->CRL|=0x07000000;\
																	SetBQ26100slavePinLevel=0;}		//配置为OD输出模式并拉低
#define	SetBQ26100slavePinOutH		{	/*SetBQ26100slavePinLevel=0;*/\
																	GPIOA->CRL&=0xF7FFFFFF;\
																	GPIOA->CRL|=0x07000000;\
																	SetBQ26100slavePinLevel=1;}		//配置为OD输出模式并拉低
													
#define	SetBQ26100slavePinIntrr	{	GPIOA->CRL&=0xF0FFFFFF;\
																	GPIOA->CRL|=0x08000000;\
																	SetBQ26100slavePinLevel=1;}			//配置为上拉输入模式

//unsigned char bqrecord[64]={0};
//unsigned char all_data_reset_flag=0;

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
	
	
	RCC_GetClocksFreq(&bqRCC_ClocksStatus);		//获取时钟参数
	SDQ_SAMPLE->time.timeper=bqRCC_ClocksStatus.SYSCLK_Frequency/8000000;
	
	EXTI_Configuration_ITF(BQ26100slavePort, BQ26100slavePin);		//外部边沿触发中断配置,抢占1，响应1--20171213	
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
	//static unsigned short time = 0;
	//====================查询摘要
	bq26100slave_set_digest();	
	//====================超时
	if(0!=SDQ_SAMPLE->PublicData.receivelen)
	{
		if(SDQ_SAMPLE->time.time_out++>0x200)
		{
			api_bq26100slave_all_data_default();
		}
	}

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
unsigned char* api_bq26100slave_get_sample_message_address(unsigned short serial)
{
	unsigned char* address = 0;
	if(serial<bq26100BufferSize)
		address = (unsigned char*)bq26100_sample_data[serial][0];
	else
		address = 0;
	return address;
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
unsigned char* api_bq26100slave_get_sample_digest_address(unsigned short serial)
{
	unsigned char* address = 0;
	if(serial<bq26100BufferSize)
		address = (unsigned char*)bq26100_sample_data[serial][1];
	else
		address = 0;
	return address;
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
unsigned short api_bq26100slave_get_sample_data_size(void)
{
	return bq26100BufferSize;
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
unsigned char bq26100slave_process(void)
{
	//unsigned long fitime=0;
	unsigned char	result	=	0;
	//=========================滤波
	SDQ_SAMPLE->time.time1=SysTick_ReLoad();			//设置SYSTICK初始值
	SDQ_SAMPLE->PublicData.intrrcout++;									//中断次数计数
	
	//=========================发送或者接收数据
	if((sdq_master_ilde	==SDQ_SAMPLE->MasterStatus)
		||(sdq_master_write	==SDQ_SAMPLE->MasterStatus))
	{
		result	=	bq26100slave_receive_byte();			//返回0--未读完一字节，返回1--读完一字节，返回0xFF--复位信号
	}
	else if(sdq_master_read	==	SDQ_SAMPLE->MasterStatus)
	{
		result	=	bq26100slave_send_byte();					//返回0--未发送完一字节，返回1--发送完一字节
	}
	//=========================读取或者发送完一字节
	if(1 == result)
	{
		//=========================BQ初始化：接收ROM命令、RAM命令和地址过程
		if(0 == SDQ_SAMPLE->InitData.InitFlag)
		{
			bq26100slave_Initialize();
		}
		//=========================根据初始化后的RAM命令执行相应的操作
		else
		{
			bq26100slave_command_process();
		}
	}
	//=========================重启时序
	else if(0xFF == result)
	{
		bq26100slave_start();
	}
	else
	{
		unsigned short time = 0;
		//========================检查有无重启消息
		while(0==GetBQ26100slavePinLevel)	//低电平
		{
			SysTick_DeleyuS(1);				//SysTick延时nuS
			if(time++>100)
			{
				bq26100slave_start();
			}
		}
	}
	if(0!=SDQ_SAMPLE->PublicData.receivelen)
	{
		if((!is_bq26100_rom_cmd(SDQ_SAMPLE->InitData.RomCmd))||(!is_bq26100_ram_cmd(SDQ_SAMPLE->InitData.MemCmd)))
		{
			api_bq26100slave_all_data_default();
		}
	}
	return result;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	BQ初始化：接收ROM命令、RAM命令和地址过程
							每次命令先传rom、ram、地址低位、地址高位
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char	bq26100slave_Initialize(void)
{
	//-----------------------获取ROM命令
	if(1==SDQ_SAMPLE->PublicData.receivelen)
	{
		SDQ_SAMPLE->InitData.RomCmd	=	(sdq_rom_cmd)SDQ_SAMPLE->PublicData.receive[0];		
	}
	//-----------------------获取RAM命令
	else if(2==SDQ_SAMPLE->PublicData.receivelen)
	{
		SDQ_SAMPLE->InitData.MemCmd	=	(sdq_mem_cmd)SDQ_SAMPLE->PublicData.receive[1];		
	}
	//-----------------------获取地址低字节
	else if(3==SDQ_SAMPLE->PublicData.receivelen)
	{
		SDQ_SAMPLE->InitData.addressL	=	(sdq_mem_cmd)SDQ_SAMPLE->PublicData.receive[2];
	}
	//-----------------------获取地址高字节
	else if(4==SDQ_SAMPLE->PublicData.receivelen)
	{
		SDQ_SAMPLE->InitData.addressH	=	(sdq_mem_cmd)SDQ_SAMPLE->PublicData.receive[3];
		bq26100slave_MenCmd_InitializeData();		//根据命令对相应的数据进行初始化
		SDQ_SAMPLE->InitData.InitFlag	=	1;			//初始化完成
	}
	return 1;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	bq26100slave_command_process
*功能描述		:	根据初始化后的RAM命令执行相应的操作：数据接收或者读取:发送CRC，message原值、status、digest
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char bq26100slave_command_process(void)
{	
	//========================主机写消息
	if(sdq_mem_write_message==SDQ_SAMPLE->InitData.MemCmd)
	{
		bq26100slave_write_message();
	}
	//========================主机读摘要
	else if(sdq_mem_read_digest==SDQ_SAMPLE->InitData.MemCmd)
	{
		if(20==SDQ_SAMPLE->MemData.Message.MessageLen)
			bq26100slave_read_digest();
	}
	//========================主机读控制寄存器
	else if((sdq_mem_read_control==SDQ_SAMPLE->InitData.MemCmd)&&(20==SDQ_SAMPLE->MemData.Message.MessageLen))
	{
		bq26100slave_read_control();
	}
	//========================主机写控制寄存器
	else if((sdq_mem_write_control==SDQ_SAMPLE->InitData.MemCmd)&&(20==SDQ_SAMPLE->MemData.Message.MessageLen))
	{
		bq26100slave_write_control();
	}	
	return 1;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	bq26100slave_MenCmd_InitializeData
*功能描述		:	根据命令对相应的数据进行初始化
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char	bq26100slave_MenCmd_InitializeData(void)
{
	//=========================主机写消息
	if(sdq_mem_write_message==SDQ_SAMPLE->InitData.MemCmd)
	{
		//---------------Message
		SDQ_SAMPLE->MemData.Message.MessageLen	=	0;
		
		//---------------Digest
		SDQ_SAMPLE->MemData.Digest.Flag.Auth	=	0;
		
		//---------------Ctrl
		SDQ_SAMPLE->MemData.Ctrl.Read.Times	=	0;		
		
		SDQ_SAMPLE->MasterStatus	=	sdq_master_write;
	}
	//=========================主机读摘要
	else if(sdq_mem_read_digest==SDQ_SAMPLE->InitData.MemCmd)
	{		
		SDQ_SAMPLE->MemData.Digest.Serial			=	0;
		
		SDQ_SAMPLE->MasterStatus	=	sdq_master_read;
		
		SDQ_SAMPLE->MemData.Digest.DigestTransType	=	sdq_Trans_ilde;
		bq26100slave_read_digest();
	}
	//=========================主机读控制寄存器
	else if(sdq_mem_read_control==SDQ_SAMPLE->InitData.MemCmd)
	{
		SDQ_SAMPLE->MemData.Ctrl.Read.Serial		=	0;
		SDQ_SAMPLE->MasterStatus									=	sdq_master_read;
		
		SDQ_SAMPLE->MemData.Ctrl.CtrlTransType		=	sdq_Trans_ilde;
		bq26100slave_read_control();
	}
	//=========================主机写控制寄存器
	else if(sdq_mem_write_control==SDQ_SAMPLE->InitData.MemCmd)
	{		
		SDQ_SAMPLE->MemData.Ctrl.Write.Serial	=	0;
		
		//---------------Digest
		SDQ_SAMPLE->MemData.Digest.Flag.Auth			=	1;
		SDQ_SAMPLE->MemData.Digest.Flag.Digested	=	0;
		
		SDQ_SAMPLE->MasterStatus										=	sdq_master_write;
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
unsigned char bq26100slave_write_message(void)
{
	//----第一步:主机写数据0
	//----第二步:主机读取"RAM命令和两字节地址长度和数据"校验
	//----第三步:主机读回数据0
	//-----------主机写剩下19字节消息
	//----第1步:主机写数据n
	//----第2步:主机读数据n与对应地址异或后对应的CRC5
	//----第3步:主机读回数据n
	
	//====================主机写数据，写完数据需要返回主机CRC
	if(sdq_master_write	==	SDQ_SAMPLE->MasterStatus)
	{
		if(SDQ_SAMPLE->MemData.Message.MessageLen<20)	//message合法字节数据只能为20
		{
			SDQ_SAMPLE->MemData.Message.MessageBuffer[SDQ_SAMPLE->MemData.Message.MessageLen]	=	SDQ_SAMPLE->TransData.RcvByte;
			SDQ_SAMPLE->MemData.Message.MessageLen		+=	1;
			
			//----------------主机发送完第一字节message后需要读取"RAM命令和两字节地址长度和数据"校验
			if(1==SDQ_SAMPLE->MemData.Message.MessageLen)
			{					
				SDQ_SAMPLE->TransData.SendByte		=	CRC8_8541_lsb(&SDQ_SAMPLE->PublicData.receive[1],SDQ_SAMPLE->PublicData.receivelen-1);
			}
			//----------------主机发完第二条message后的校验方式:当前地址与当前数据异或出结果再求对应结果的CRC
			else
			{
				unsigned char temp=(SDQ_SAMPLE->MemData.Message.MessageLen-1)^SDQ_SAMPLE->MemData.Message.MessageBuffer[SDQ_SAMPLE->MemData.Message.MessageLen-1];
				SDQ_SAMPLE->TransData.SendByte		=	CRC8_8541_lsb(&temp,1);
			}
			//----------------主机每发送完一字节数据后需要读取CRC
			SDQ_SAMPLE->MasterStatus	=	sdq_master_read;
			SDQ_SAMPLE->MemData.Message.MessageTransType	=	sdq_Trans_CRC;
		}
	}
	//====================主机读数据:读CRC和原数据
	else if(sdq_master_read	==	SDQ_SAMPLE->MasterStatus)
	{
		//------------------传输完的是CRC:下一步传输最后接收的一字节数据
		if(sdq_Trans_CRC == SDQ_SAMPLE->MemData.Message.MessageTransType)
		{
			SDQ_SAMPLE->TransData.SendByte								=	SDQ_SAMPLE->MemData.Message.MessageBuffer[SDQ_SAMPLE->MemData.Message.MessageLen-1];
			SDQ_SAMPLE->MemData.Message.MessageTransType	=	sdq_Trans_Data;
		}
		//------------------传输完的是数据:下一步是接收主机数据或者传输完成检查
		else if(sdq_Trans_Data == SDQ_SAMPLE->MemData.Message.MessageTransType)
		{
			if(20	<=	SDQ_SAMPLE->MemData.Message.MessageLen)
			{
				SDQ_SAMPLE->MasterStatus					=	sdq_master_ilde;						//空闲:主机发送完所有的message
			}
			else
			{
				SDQ_SAMPLE->MasterStatus	=	sdq_master_write;		//主机继续发送下一字节数据
			}
		}
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
unsigned char bq26100slave_read_control(void)
{
	//----------------第一步:主机读RAM,AddressL,AddressH校验
	//----------------第二步:主机读00地址状态值
	//----------------第三步:主机读01地址状态值
	//----------------第四步:主机读(RAM,AddressL,AddressH)&(RAM,AddressL,AddressH校验结果)&(00地址内容)&(01地址内容)的CRC校验
	//====================第一步:主机写数据
	if(sdq_master_write	==	SDQ_SAMPLE->MasterStatus)
	{
	}
	//====================主机读数据
	else if(sdq_master_read	==	SDQ_SAMPLE->MasterStatus)
	{
		//---------------------第一步:主机读取RAM命令和两字节地址校验
		if(0==SDQ_SAMPLE->MemData.Ctrl.Read.Serial)
		{
			SDQ_SAMPLE->MemData.Ctrl.Read.CrcData	=	CRC8_8541_lsb(&SDQ_SAMPLE->PublicData.receive[1],3);		//第一次校验为RAM命令和两字节地址校验
			SDQ_SAMPLE->TransData.SendByte	=	SDQ_SAMPLE->MemData.Ctrl.Read.CrcData;
		}
		//---------------------第二步:读取状态地址0的值
		else if(1==SDQ_SAMPLE->MemData.Ctrl.Read.Serial)
		{
			//-------------------第一次读取状态地址0的值为0x04
			if(0	==	SDQ_SAMPLE->MemData.Ctrl.Read.Times)
			{
				SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl0		=	0x04;	//0x04;
				SDQ_SAMPLE->TransData.SendByte				=	SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl0;
			}
			//-------------------第二次读取状态地址0的值为0x06
			else
			{
				SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl0		=	0x06;
				SDQ_SAMPLE->TransData.SendByte				=	SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl0;
			}
		}
		//---------------------第三步:读取状态地址1的值--两次读取都为0xA1
		else if(2==SDQ_SAMPLE->MemData.Ctrl.Read.Serial)
		{
			SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl1		=	0xA1;
			SDQ_SAMPLE->TransData.SendByte				=	SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl1;
		}
		//---------------------第四步:校验为(RAM,AddressL,AddressH)&(RAM,AddressL,AddressH校验结果)&(00地址内容)&(01地址内容)的CRC校验
		else if(3==SDQ_SAMPLE->MemData.Ctrl.Read.Serial)
		{
			unsigned char buffer[6]={0};
			memcpy(buffer,&SDQ_SAMPLE->PublicData.receive[1],3);
			buffer[3]	=	SDQ_SAMPLE->MemData.Ctrl.Read.CrcData;
			buffer[4]	=	SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl0;
			buffer[5]	=	SDQ_SAMPLE->MemData.Ctrl.Read.Ctrl1;
			SDQ_SAMPLE->TransData.SendByte	=	CRC8_8541_lsb(buffer,6);
		}
		//---------------------完成读取
		else
		{
			if(0==SDQ_SAMPLE->MemData.Ctrl.Read.Times)
			{
				SDQ_SAMPLE->MemData.Ctrl.Read.Times	=1;
			}
			else
			{
				SDQ_SAMPLE->MemData.Ctrl.Read.Times	=0;
			}			
			SDQ_SAMPLE->MemData.Ctrl.Read.Serial=0;
			SDQ_SAMPLE->MasterStatus	=	sdq_master_ilde;
			return 1;
		}
		SDQ_SAMPLE->MemData.Ctrl.Read.Serial+=1;
		SDQ_SAMPLE->MasterStatus	=	sdq_master_read;
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
unsigned char bq26100slave_write_control(void)
{	
	//----第一步:主机写状态寄存器0
	//----第二步:主机读取"RAM命令和两字节地址长度和数据"校验
	//----第三步:主机读取状态寄存器0
	//----第四步:主机写状态寄存器1
	//----第五步:主机读取状态寄存器地址1与写入的状态寄存器值异或后对应的CRC5
	//----第六步:主机读取状态寄存器1的值
	
	//====================主机写控制寄存器：写完寄存器后主机先读取CRC
	if(sdq_master_write	==	SDQ_SAMPLE->MasterStatus)
	{
		//------------------主机写入状态寄存器0数据
		if(0==SDQ_SAMPLE->MemData.Ctrl.Write.Serial)
		{
			SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl0R	=	SDQ_SAMPLE->TransData.RcvByte;			//主机发送的状态寄存器0的值
			SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl0T	=	0x06;																//主机待读取的状态寄存器0的值
			SDQ_SAMPLE->TransData.SendByte	=	CRC8_8541_lsb(&SDQ_SAMPLE->PublicData.receive[1],SDQ_SAMPLE->PublicData.receivelen-1);	//主机发送完第一字节控制寄存器值后需要读取"RAM命令和两字节地址长度和数据"校验
		}
		//------------------主机写入状态寄存器1数据
		else if(1==SDQ_SAMPLE->MemData.Ctrl.Write.Serial)
		{
			unsigned char temp	=	0;
			SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl1	=	SDQ_SAMPLE->TransData.RcvByte;		//主机发送的状态寄存器1的值
			temp	=	0x01^SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl1;						//当前地址与当前数据异或出结果再求对应结果的CRC
			SDQ_SAMPLE->TransData.SendByte		=	CRC8_8541_lsb(&temp,1);		//CRC
		}		
		SDQ_SAMPLE->MemData.Ctrl.Write.Serial+=1;									//主机写完数据后转为读取模式
		SDQ_SAMPLE->MasterStatus	=	sdq_master_read;							//写完寄存器后主机先读取CRC
		SDQ_SAMPLE->MemData.Ctrl.CtrlTransType	=	sdq_Trans_CRC;	//写完寄存器后主机先读取CRC
	}
	//====================主机读数据CRC和数据
	else if(sdq_master_read	==	SDQ_SAMPLE->MasterStatus)
	{
		//------------------主机读取完CRC
		if(sdq_Trans_CRC	==	SDQ_SAMPLE->MemData.Ctrl.CtrlTransType)
		{
			if(1==SDQ_SAMPLE->MemData.Ctrl.Write.Serial)
			{
				SDQ_SAMPLE->TransData.SendByte	=	SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl0T;	//读取完第一个CRC,然后读取第一个状态寄存器
			}
			//----------------读取完第二个CRC
			else
			{
				SDQ_SAMPLE->TransData.SendByte	=	SDQ_SAMPLE->MemData.Ctrl.Write.Ctrl1;		//主机读完CRC后读取状态寄存器值		
			}
			SDQ_SAMPLE->MemData.Ctrl.CtrlTransType	=	sdq_Trans_Data;
		}
		//------------------主机读取完状态寄存器值
		else
		{
			if(1==SDQ_SAMPLE->MemData.Ctrl.Write.Serial)
			{
				SDQ_SAMPLE->MasterStatus	=	sdq_master_write;		//主机读完第一个状态寄存器后主机准备写状态寄存器2的值
			}
			else
			{
				SDQ_SAMPLE->MemData.Ctrl.Write.Serial	=	0;
				SDQ_SAMPLE->MasterStatus	=	sdq_master_ilde;		//主机读完第二个状态寄存器后完成写寄存器过程
			}
		}
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
unsigned char	bq26100slave_read_digest(void)
{
	//----第一步:主机读RAM,AddressL,AddressH校验
	//----第二步:主机读取"RAM命令和两字节地址长度和数据"校验
	//----第三步:主机读取状态寄存器0
	//----第四步:主机写状态寄存器1
	//----第五步:主机读取状态寄存器地址1与写入的状态寄存器值异或后对应的CRC5
	//----第六步:主机读取状态寄存器1的值
	
	//====================第一步:主机写数据
	if(sdq_master_write	==	SDQ_SAMPLE->MasterStatus)
	{
	}
	//====================主机读数据
	else if(sdq_master_read	==	SDQ_SAMPLE->MasterStatus)
	{
		//------------------第一步:主机读取RAM命令和两字节地址校验
		if(sdq_Trans_ilde == SDQ_SAMPLE->MemData.Digest.DigestTransType)
		{
			//----------------RAM命令和两字节地址校验
			SDQ_SAMPLE->TransData.SendByte	=		CRC8_8541_lsb(&SDQ_SAMPLE->PublicData.receive[1],3);
			SDQ_SAMPLE->MemData.Digest.DigestTransType	=	sdq_Trans_CRC;
		}
		//====================传输第一字节摘要
		else if(sdq_Trans_CRC == SDQ_SAMPLE->MemData.Digest.DigestTransType)
		{
			//--------------第一次返回摘要为原message消息----未设置控制位
			if(0==SDQ_SAMPLE->MemData.Digest.Flag.Auth)		//未启动转换:发回message
			{
				memcpy(SDQ_SAMPLE->MemData.Digest.DigestBuffer,SDQ_SAMPLE->MemData.Message.MessageBuffer,20);
			}
			//--------------第二次返回摘要为转换后的摘要消息--已设置控制位，如果未匹配到摘要则做异常处理
			//--------------启动发送
			SDQ_SAMPLE->MemData.Digest.Serial			=	0;			
			SDQ_SAMPLE->TransData.SendByte		=	SDQ_SAMPLE->MemData.Digest.DigestBuffer[SDQ_SAMPLE->MemData.Digest.Serial];
			
			SDQ_SAMPLE->MemData.Digest.DigestTransType	=	sdq_Trans_Data;
		}
		//====================传输剩下19字节摘要
		else if(sdq_Trans_Data == SDQ_SAMPLE->MemData.Digest.DigestTransType)
		{
			SDQ_SAMPLE->MemData.Digest.Serial	+=	1;		//已发送数量计数
			if(20>SDQ_SAMPLE->MemData.Digest.Serial)
			{
				SDQ_SAMPLE->TransData.SendByte		=	SDQ_SAMPLE->MemData.Digest.DigestBuffer[SDQ_SAMPLE->MemData.Digest.Serial];
			}
			//--------------发送所有的Digest数据校验
			else if(20==SDQ_SAMPLE->MemData.Digest.Serial)
			{
				SDQ_SAMPLE->TransData.SendByte		=	CRC8_8541_lsb(SDQ_SAMPLE->MemData.Digest.DigestBuffer,20);
			}
			else	//发送完摘要和CRC
			{
				SDQ_SAMPLE->MemData.Digest.Serial	=	0;
				SDQ_SAMPLE->PublicData.receivelen	=	0;
				SDQ_SAMPLE->MasterStatus					=	sdq_master_ilde;
				return 1;
			}
			SDQ_SAMPLE->MemData.Digest.DigestTransType	=	sdq_Trans_Data;
		}
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
	SDQ_SAMPLE->MemData.Digest.Flag.match_error	=	1;
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
	return SDQ_SAMPLE->MemData.Digest.Flag.match_error;
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
	SDQ_SAMPLE->MemData.Digest.Flag.match_error	=	0;
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
	return SDQ_SAMPLE->PublicData.receivelen;
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
unsigned char bq26100slave_send_bit(unsigned char bit)
{	
	SetBQ26100slavePinOutL;
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
*返回值			:	0:0,1:1,0x80:未知，0xFF重启
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char bq26100slave_receive_bit(void)
{
	unsigned char flag=0;
	unsigned char bit = 0x80;
	//-----------------主机写数据或者空闲状态时
	if((sdq_master_ilde==SDQ_SAMPLE->MasterStatus)||(sdq_master_write==SDQ_SAMPLE->MasterStatus))
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
unsigned char bq26100slave_send_byte(void)
{	
	//=======================按位发送
	bq26100slave_send_bit(SDQ_SAMPLE->TransData.SendByte&0x01);	
	//=======================
	if(0==SDQ_SAMPLE->TransData.SendBitLen)
		RecordData(SDQ_SAMPLE->TransData.SendByte);
	//=======================右移一位:低位先传
	SDQ_SAMPLE->TransData.SendByte		>>=	1;
	//=======================传输计数
	SDQ_SAMPLE->TransData.SendBitLen	+=	1;
	//=======================
	if(8<=SDQ_SAMPLE->TransData.SendBitLen)
	{
		SDQ_SAMPLE->TransData.SendBitLen	=	0;
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
unsigned char bq26100slave_receive_byte(void)
{
	unsigned char bit=bq26100slave_receive_bit();
	//--------------------右移1位:低位先传输	
	SDQ_SAMPLE->TransData.RcvByte>>=1;
	if(0==bit)
	{		
		SDQ_SAMPLE->TransData.RcvByte&=0x7F;		
	}
	else if(1==bit)
	{
		SDQ_SAMPLE->TransData.RcvByte|=0x80;		
	}
	else if(0xFF == bit)
	{
		return 0xFF;
	}	
	//--------------------字节接收计数
	SDQ_SAMPLE->TransData.RcvBitlen+=1;
	//--------------------判断数据有无接收完一字节
	if(8<=SDQ_SAMPLE->TransData.RcvBitlen)
	{
		SDQ_SAMPLE->TransData.RcvBitlen	=	0;
		//========================将接收到的数据保存	
		if(SDQ_SAMPLE->PublicData.receivelen<sdq_receive_buffer_size)
		{
			SDQ_SAMPLE->PublicData.receive[SDQ_SAMPLE->PublicData.receivelen]=SDQ_SAMPLE->TransData.RcvByte;
			SDQ_SAMPLE->PublicData.receivelen+=1;
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
unsigned char bq26100slave_start(void)
{	
	//------启动条件：主机拉低信号480us-960us,然后主机释放总线
	//------等待15us-60us后从机拉低总线60us-240us作为对主机的应答
	unsigned short time = 0;
	//===========================等待主机释放总线
	while((0==GetBQ26100slavePinLevel)&&(time<480))	//低电平
	{
		SysTick_DeleyuS(1);					//SysTick延时nuS
		time++;
	}
	//===========================等待15us-60us后从机拉低总线60us-240us作为对主机的应答
	SetBQ26100slavePinOutH;
	SetBQ26100slavePinLevel=1;
	SysTick_DeleyuS(45);					//SysTick延时nuS
	SetBQ26100slavePinLevel=0;
	SysTick_DeleyuS(160);					//SysTick延时nuS
	SetBQ26100slavePinIntrr;
	//===========================数据初始化
	bq26100slave_start_data_init();
	return 1;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	bq26100slave_start_data_init
*功能描述		:	检测到重启时序时的数据初始化
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char	bq26100slave_start_data_init(void)
{
	unsigned char all_data_reset_flag=0;
	//====================
	if(sdq_master_ilde	!=	SDQ_SAMPLE->MasterStatus)
	{
		all_data_reset_flag=1;
	}
	//====================TransData
	else if((0!=SDQ_SAMPLE->TransData.RcvBitlen)&&(0!=SDQ_SAMPLE->TransData.SendBitLen))
	{
		all_data_reset_flag=2;
	}
	//====================MemData.Message
	else if((0!=SDQ_SAMPLE->MemData.Message.MessageLen)&&(20!=SDQ_SAMPLE->MemData.Message.MessageLen))
	{
		all_data_reset_flag=3;
	}
	//====================MemData.Digest
	else if(0!=SDQ_SAMPLE->MemData.Digest.Serial)
	{
		all_data_reset_flag=4;
	}
	//====================MemData.Ctrl
	else if((0!=SDQ_SAMPLE->MemData.Ctrl.Read.Serial)&&(0!=SDQ_SAMPLE->MemData.Ctrl.Write.Serial))
	{
		all_data_reset_flag=5;
	}
	if(all_data_reset_flag)
	{
		all_data_reset_flag=0;
		api_bq26100slave_all_data_default();
	}
	else
	{
		all_data_reset_flag=0;
		SDQ_SAMPLE->InitData.InitFlag	=	0;			//初始化完成
		SDQ_SAMPLE->PublicData.receivelen	=	0;
		SDQ_SAMPLE->MasterStatus	=	sdq_master_write;
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
unsigned char	api_bq26100slave_all_data_default(void)
{
	//====================
	SDQ_SAMPLE->MasterStatus	=	sdq_master_write;
	
	//====================TransData
	SDQ_SAMPLE->TransData.RcvBitlen		=	0;
	SDQ_SAMPLE->TransData.SendBitLen	=	0;
	
	//====================PublicData
	SDQ_SAMPLE->PublicData.receivelen	=	0;
	SDQ_SAMPLE->PublicData.intrrcout	=	0;
	
	//====================InitData
	SDQ_SAMPLE->InitData.InitFlag	=	0;
	
	//====================MemData.Message
	SDQ_SAMPLE->MemData.Message.MessageLen	=	0;
	
	//====================MemData.Digest
	SDQ_SAMPLE->MemData.Digest.Serial					=	0;
	SDQ_SAMPLE->MemData.Digest.Flag.Auth			=	0;
	SDQ_SAMPLE->MemData.Digest.Flag.Digested	=	0;
	//====================MemData.Ctrl
	SDQ_SAMPLE->MemData.Ctrl.Read.Serial	=	0;
	SDQ_SAMPLE->MemData.Ctrl.Write.Serial	=	0;
	SDQ_SAMPLE->MemData.Ctrl.Read.Times		=	0;

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
void RecordData(unsigned char bytedata)
{
	//static unsigned short serial=0;
	
//	if(1==crflag)
//	{
//		bqrecord[serial]=bytedata;
//		
//		if(serial++>=20)
//			serial=0;
//	}
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
void	bq26100slave_set_digest(void)
{
	if((1==SDQ_SAMPLE->MemData.Digest.Flag.Auth)		//未启动转换:发回message
		&&(0==SDQ_SAMPLE->MemData.Digest.Flag.Digested))
	{
		unsigned short i = 0;
		for(i=0;i<bq26100BufferSize;i++)
		{
			if(0==memcmp(SDQ_SAMPLE->MemData.Message.MessageBuffer,bq26100_sample_data[i][0],20))
			{
				memcpy(SDQ_SAMPLE->MemData.Digest.DigestBuffer,bq26100_sample_data[i][1],20);
				break;
			}
		}
		//------------------------未匹配到数据
		api_bq26100slave_reset_error_status();
		if(i>=bq26100BufferSize)
		{
			api_bq26100slave_digest_match_error();
			memset(SDQ_SAMPLE->MemData.Digest.DigestBuffer,0x00,20);
		}
		SDQ_SAMPLE->MemData.Digest.Flag.Digested = 1;
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
void EXTI9_5_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line6);
	//=======================高电平:上升沿
	if(0==GetBQ26100slavePinLevel)	//低电平
	{	
		if(SDQ_SAMPLE->MasterStatus==sdq_master_read)
			SetBQ26100slavePinOutL;
		bq26100slave_process();
		SDQ_SAMPLE->time.time_out	=	0;
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



