#include "BQ26100Master.H"	

//#include "BQ26100DATA.H"

#include "stdlib.h"
#include "string.h"

#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

#include 	"CRC.H"

#include "usb_data.h"

#include "STM32F10x_BitBand.H"

#define bq26100Model1		0
#define bq26100Model2		1
#define bq26100V2Slave	0


#if bq26100Model1
	#define	BQ26100MasterPort		GPIOA
	#define	BQ26100MasterPin		GPIO_Pin_3
#elif bq26100Model2
	#if bq26100V2Slave
		//-------------------飞达接口
		#define BQ26100MasterPort   				GPIOA  
		#define BQ26100MasterPin    				GPIO_Pin_6
		
		#define	SetBQ26100MasterPinLevel		PA6
		#define	GetBQ26100MasterPinLevel		PA6in

		#define	SetBQ26100MasterPinOut			{	GPIOA->CRL&=0xF0FFFFFF;\
																					GPIOA->CRL|=0x07000000;\
																					SetBQ26100MasterPinLevel=0;}		//配置为OD输出模式并拉低
															
		#define	SetBQ26100MasterPinInUp			{	GPIOA->CRL&=0xF0FFFFFF;\
																					GPIOA->CRL|=0x08000000;\
																					SetBQ26100MasterPinLevel=0;}			//配置为上拉输入模式

	#else
		//-------------------带KEY
		#define BQ26100MasterPort   			GPIOA  
		#define BQ26100MasterPin    			GPIO_Pin_7
		
		#define	SetBQ26100MasterPinLevel	PA7
		#define	GetBQ26100MasterPinLevel	PA7in

		#define	SetBQ26100MasterPinOut		{	GPIOA->CRL&=0x0FFFFFFF;\
																				GPIOA->CRL|=0x70000000;\
																				SetBQ26100MasterPinLevel=1;}		//配置为OD输出模式并拉低
													
		#define	SetBQ26100MasterPinInUp		{	GPIOA->CRL&=0x0FFFFFFF;\
																				GPIOA->CRL|=0x80000000;\
																				SetBQ26100MasterPinLevel=1;}			//配置为上拉输入模式

	#endif
		
		
		#define FDSDQPort   	GPIOA  
		#define FDSDQPin    	GPIO_Pin_6
		
		#define U4SDQPort   	GPIOA  
		#define U4SDQPin    	GPIO_Pin_5
		
		//-------------------飞达接口
		#define FDSDQPort   	GPIOA  
		#define FDSDQPin    	GPIO_Pin_6
#endif
//------------------------------------------------------------------------------


/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void api_bq26100Master_configuration(void)
{
	GPIO_Configuration_OPP50	(BQ26100MasterPort,	BQ26100MasterPin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
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
void api_bq26100Master_server(void)
{

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
unsigned char api_bq26100Master_get_digest(const unsigned char* message,unsigned char* digest)
{
//	#include "usb_data.h"
	//-------------流程：
	//1,生成随机消息
	//2,将数据与key一起计算出结果
	//3,将消息发往bq26100
	//4,设置发送bq26100控制命令
	//5,读取bq26100的验证结果
	//6,对比第二步计算出的结果与从bq26100读取的数据,如果相同，验证通过
	unsigned char Value;
	int i;
	static unsigned char flag=0;
  sdq_result result;

	//--------------------发送消息
	result	=	api_bq26100Master_send_message(message);
	if(result	==	sdq_error)
		return 0;
	//--------------------读取发送的message数据
	result	=	api_bq26100Master_read_digest(digest);
	if(result	==	sdq_error)
		return 0;
	//--------------------
	result	=	api_bq26100Master_read_control();
	if(result	==	sdq_error)
		return 0;
	//--------------------	
	result	=	api_bq26100Master_write_control();
	if(result	==	sdq_error)
		return 0;
	//--------------------
	result	=	api_bq26100Master_read_control();
	if(result	==	sdq_error)
		return 0;
	//--------------------读取bq26100生成的SHA1/HMAC并存储在digest
	result	=	api_bq26100Master_read_digest(digest);
	
	return 1;
}
//---------------------------------------------------------------------------



/*******************************************************************************
*函数名			:	bq26100_step1_send_message
*功能描述		:	Send 160-bit Message to bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100Master_send_message(const unsigned char* message)
{
	unsigned long i = 0;
	unsigned char Value	=	0;
	unsigned char CRC8	=	0;
	unsigned char temp	= 0;
	sdq_result result;
	// Send 160-bit Message to bq26100
	//----------------reset
	result	=	api_bq26100Master_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//----------------Write ROM Command:SkipID
	bq26100Master_write_byte(sdq_rom_skipID);
	//----------------Write RAM Command:write_message
	bq26100Master_write_byte(sdq_mem_write_message);
	//----------------Write Adrress Low byte
	bq26100Master_write_byte(0x00);
	//----------------Write Adrress high byte
	bq26100Master_write_byte(0x00);
	
	//----------------Write first byte of message, Message[0]
	bq26100Master_write_byte(message[0x00]);
	//----------------Read CRC of Message Command, Address and Message[0]
	//								CRC are not being calculated by the microcontroller, the results given by
	Value = bq26100Master_read_byte();
	//----------------Read Data to Verify Write
	Value = bq26100Master_read_byte();
	//----------------Write the remaining bytes of the message
	for (i=1; i<=19; i++)
	{
		//--------------Write Data
		bq26100Master_write_byte(message[i]);
		//--------------Read CRC of Address and Message
		//							CRC are not being calculated by the microcontroller, the results given by bq26100 are being ignored
		Value	=	bq26100Master_read_byte();
		//temp	=	(i^message[i]);
		//CRC8	=	CRC8_8541_lsb(&temp,1);
		//if(Value	!=	CRC8)
			//break;
		//--------------Read Data to Verify Write
		Value	=	bq26100Master_read_byte();
		//if(Value	!=	message[i])
			//break;
	}
	return sdq_success;
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	bq26100_step2_write_control
*功能描述		:	Write Control to set AUTH bit to initiate Authentication by bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100Master_write_control(void)
{
	unsigned char Value	=	0;
	unsigned char i = 0;
	sdq_result result;
	// Write Control to set AUTH bit to initiate Authentication by bq26100
	//----------------reset
	result	=	api_bq26100Master_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//----------------Write ROM Command:SkipID
	bq26100Master_write_byte(sdq_rom_skipID);
	//----------------Write RAM Command:Write Control
	bq26100Master_write_byte(sdq_mem_write_control);
	//----------------Write Adrress Low byte
	bq26100Master_write_byte(0x00);
	//----------------Write Adrress high byte
	bq26100Master_write_byte(0x00);
	
	//----------------Write Control Register 0
	bq26100Master_write_byte(0x05);
	//----------------Read CRC of RAM Command, Address and Register 0
	Value = bq26100Master_read_byte();	
	//----------------Read New Value Of Control Register 0
	Value = bq26100Master_read_byte();
	
	//----------------Write Control Register 1
	bq26100Master_write_byte(0xA1);
	//----------------Read CRC of	Address and Register 1
	Value = bq26100Master_read_byte();	
	//----------------Read New Value Of Control Register 1
	Value = bq26100Master_read_byte();

return sdq_success;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	bq26100_step3_read_control
*功能描述		:	Write Control to set AUTH bit to initiate Authentication by bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100Master_read_control(void)
{
	unsigned char Value	=	0;
	unsigned char i=0;
	sdq_result result;
	// Write Control to set AUTH bit to initiate Authentication by bq26100
	//----------------reset
	result	=	api_bq26100Master_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//----------------Write ROM Command:SkipID
	bq26100Master_write_byte(sdq_rom_skipID);
	//----------------Write RAM Command:Read Control
	bq26100Master_write_byte(sdq_mem_read_control);
	//----------------Write Adrress Low byte
	bq26100Master_write_byte(0x00);
	//----------------Write Adrress high byte
	bq26100Master_write_byte(0x00);	
	//----------------Read CRC of RAM Command and Address
	Value = bq26100Master_read_byte();	
	//----------------Read	Value Of Control Register 0
	Value = bq26100Master_read_byte();	
	//----------------Read	Value Of Control Register 1
	Value = bq26100Master_read_byte();	
	//----------------Read CRC From RAM Command To Value Of Control Register 1
	Value = bq26100Master_read_byte();
	
	return sdq_success;
	
}
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	bq26100_step4_read_digest
*功能描述		:	Read Digest
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100Master_read_digest(unsigned char* digest)
{
	unsigned char i = 0;
	unsigned char Value	=	0;
	sdq_result result;
	//Read Digest
	//----------------reset
	result	=	api_bq26100Master_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//----------------Write ROM Command:SkipID
	bq26100Master_write_byte(sdq_rom_skipID);
	//----------------Write RAM Command:Read Digest
	bq26100Master_write_byte(sdq_mem_read_digest);
	//----------------Write Adrress Low byte
	bq26100Master_write_byte(0x00);
	//----------------Write Adrress high byte
	bq26100Master_write_byte(0x00);
	//----------------Read CRC of RAM Command and Address
	Value = bq26100Master_read_byte();
	//----------------Read 20 byte Digest Data
	for (i=0; i<20; i++)
	{
		digest[i] = bq26100Master_read_byte();
	}
	//----------------Read CRC of all Digest Data transmitted
	Value = bq26100Master_read_byte();

	return sdq_success;
}
//------------------------------------------------------------------------------

















/*******************************************************************************
* 函数名			:	sdq_write_bit
* 功能描述		:	1-wire 一位（1bit）写操作-写0&写1：写操作整体时序需要60-120us，写完后需要最少1us等待时间。
							写操作时从设备采样在总线拉低后15-60us进行采样，写时序需要在总线拉低后15us内设置总线是写1还是写0（高电平写1，低电平写0）。

							向1-Wire总线写1bit至少需要60μs，同时还要保证两次连续的写操作有1μs 以上的间隔。
							若待写位wbit为0则主机拉低总线60μs然后释放，写0操作完成。
							若待写位wbit为1，则主机拉低总线并在1～15μs内释放，然后等待60μs，写1操作完成。
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void bq26100Master_write_bit(unsigned char bit)		//1-wire 一位（1bit）写操作-写0&写1
{
	//总时间按80us
	//----------------拉低总线
	SetBQ26100MasterPinLevel=0;
	//----------------至少维持了1us,表示写时序(包括写0时序或写1时序)开始
	SysTick_DeleyuS(6);
	//----------------写入位状态
	SetBQ26100MasterPinLevel	=	bit&0x01;
	//----------------等待从机采样完成
	SysTick_DeleyuS(80);
	//----------------拉高总线
	SetBQ26100MasterPinLevel=1;
	//----------------至少需要1us恢复时间
	SysTick_DeleyuS(4);
	return ;
}
//------------------------------------------------------------------------------


/*******************************************************************************
* 函数名			:	sdq_read_bit
* 功能描述		:	从Dallas读取一个位	- 读0&读1
							读时序最少需要拉低总线1us，然后释放总线，从设备会在15-45us操作总线，主机进行采样。
							两次之间最少需要1us的时间间隔。
							从1-Wire总线读取1bit同样至少需要60μs，同时也要保证两次连续的读操作间隔1μs以上
* 输入			: void
* 返回值			: 1/0
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static unsigned char bq26100Master_read_bit(void)
{
	//总时间按80us
	unsigned char bit=0;
	//----------------设为输出模式
	SetBQ26100MasterPinOut;
	//----------------拉低总线:启动读
	SetBQ26100MasterPinLevel	=	0;
	//----------------至少1us时间
	SysTick_DeleyuS(2);
	//----------------释放总线：设置为输入模式
	SetBQ26100MasterPinInUp;
	//----------------等待从机响应
	SysTick_DeleyuS(15);
	//----------------读取总线状态
	if(GetBQ26100MasterPinLevel)
		bit = 1;
	else
		bit = 0;
	//----------------从1-Wire总线读取1bit至少需要60us
	SysTick_DeleyuS(45);
	return bit;
}
//------------------------------------------------------------------------------

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	写一个字节到Dallas---从低位开始写入
* 输入			: dat：要写入的字节,
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void bq26100Master_write_byte(unsigned char dat)
{             
	unsigned char j;
	unsigned char testb;
	//----------------设为输出模式
	SetBQ26100MasterPinOut;
	//----------------写入8个位状态
	for (j=1; j<=8; j++)
	{
		testb = dat&0x01;
		dat = dat>>1;
		//----------------写入位信息（1bit）
		bq26100Master_write_bit(testb);
	}
}
//------------------------------------------------------------------------------

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	从Dallas读取一个字节---从低位开始读取
* 输入			: void
* 返回值			: 读到的数据
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static unsigned char	bq26100Master_read_byte(void)
{        
	unsigned char i,j,dat=0;
	//----------------设为输入模式
	SetBQ26100MasterPinInUp;
	//----------------读取8个位状态:先读取最低位
	for (i=0; i<8; i++)
	{
		//----------------位读取
		dat>>=1;
		if(bq26100Master_read_bit())
		{
			dat|= 0x80;		//Bit == 1
		}
		else
		{
			dat&= 0x7F;		//Bit == 0
		}
	}	
	return dat;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	复位Dallas--拉低总线(低速480uS,高速48uS),然后释放总线，等待从机拉高总线
* 输入			: void
* 返回值			: 返回0:复位不成功/无IC
							返回1:复位成功
*******************************************************************************/
sdq_result api_bq26100Master_rest(void)		//复位Dallas,返回结果
{
	unsigned short retry=0;
	//----------------------复位时间：拉低信号大于480us
	SetBQ26100MasterPinLevel	=	1;
	//----------------------设置为输出模式
	SetBQ26100MasterPinOut;
	//----------------------启动前总线电平为1
	SetBQ26100MasterPinLevel	=	1;
	//----------------------
	SysTick_DeleyuS(10);		//拉低750us（大于480uS)
	//----------------------拉低总线信号
	SetBQ26100MasterPinLevel	=	0;
	//----------------------拉低750us（大于480uS)
	SysTick_DeleyuS(480);
	//----------------释放总线：设置为输入模式
	SetBQ26100MasterPinInUp;
	//----------------------
	SetBQ26100MasterPinLevel	=	1;
	//----------------------检测响应需要在15uS后
	SysTick_DeleyuS(15);
	//----------------------检测响应：总时间需要大于480us
	while(GetBQ26100MasterPinLevel	&& (retry < 260))			//响应检测时间不超过240uS
	{
		retry++;
		SysTick_DeleyuS(1);
	}
	if(retry >= 250)	//超时
		return sdq_error;
	else
		retry=0;
	//----------------------设置为输出模式
	SetBQ26100MasterPinOut;
	//----------------------
	SetBQ26100MasterPinLevel	=	1;
	//----------------------增加延时--为了满足总的检测时间480uS
	SysTick_DeleyuS(400);
	return sdq_success;
}
//------------------------------------------------------------------------------

























