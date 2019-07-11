#include "BQ26100.H"	

//#include "BQ26100DATA.H"

#include "stdlib.h"
#include "string.h"

#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

#include 	"CRC.H"

#include "usb_data.h"

#define bq26100V1	0
#define bq26100V2	1
#define bq26100V2Slave	1

#if bq26100V1
	#define	SDQ_Port		GPIOA
	#define	SDQ_Pin			GPIO_Pin_3
#elif bq26100V2
	#if bq26100V2Slave
		//-------------------飞达接口
		#define SDQ_Port   	GPIOA  
		#define SDQ_Pin    	GPIO_Pin_6
	#else
		//-------------------带KEY
		#define SDQ_Port   	GPIOA  
		#define SDQ_Pin    	GPIO_Pin_7
	#endif
		
		
		#define FDSDQPort   	GPIOA  
		#define FDSDQPin    	GPIO_Pin_6
		
		#define U4SDQPort   	GPIOA  
		#define U4SDQPin    	GPIO_Pin_5
		
		//-------------------飞达接口
		#define FDSDQPort   	GPIOA  
		#define FDSDQPin    	GPIO_Pin_6
#endif

#define	SDQ_SetOut		GPIO_Configuration_OPP50(SDQ_Port,	SDQ_Pin)			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
#define	SDQ_SetIn			GPIO_Configuration_IPD(SDQ_Port,		SDQ_Pin)			//将GPIO相应管脚配置为下拉输入模式----V20170605

#define	SDQ_H					SDQ_Port->BSRR	= SDQ_Pin			//CPLD_WR	=	1;
#define	SDQ_L					SDQ_Port->BRR		= SDQ_Pin			//CPLD_WR	=	0;

#define	SDQ_Read			(SDQ_Port->IDR	& SDQ_Pin)			//CPLD_WR	=	0;


#define bq26100_Family_Code 0x09		//默认09


// Global Variables
unsigned char Message[20];			//These are the 20 bytes for the random message	sent to bq26100
unsigned char Key[16];				//These are the 16 bytes for the secret key, should match with contents of valid bq26100 
unsigned char Digest[20];			//These are the 20 bytes for the SHA1 response of the bq26100

// Global Variables for SHA1
unsigned long Ws[80];					//Global Work schedule variable--计算时需要用到的缓冲区
unsigned long A;	//缓冲区，计算前初始化值为0x67452301
unsigned long B;	//缓冲区，计算前初始化值为0xEFCDAB89
unsigned long C;	//缓冲区，计算前初始化值为0x98BADCFE
unsigned long D;	//缓冲区，计算前初始化值为0x10325476
unsigned long E;	//缓冲区，计算前初始化值为0xC3D2E1F0
unsigned long H[5];	//计算出的消息摘要缓存，消息摘要是一个160位的字符串
unsigned long Random[5];		//The 16 bytes of random message for the bq26100 are contained here
					//for microcontroller to use in SHA1/HMAC
unsigned long Digest_32[5];	//The result of the SHA1/HMAC obtained by the microcontroller is contained here


unsigned long key_key32[4] = {0x00,0x01,0x02,0x03};
const unsigned long key_message[5] = { 0xc82ca3ca ,0x10dec726 ,0x8e070a7c ,0xf0d1fe82 ,0x20aad3b8 };		//The 16 bytes of random message for the bq26100 are contained here
const unsigned long key_digest[5] = { 0x18459979,0xca46a610,0xb18d173e,0x0db5d113 ,0x99951241 };	//The result of the SHA1/HMAC obtained by the microcontroller is contained here

 
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void api_bq26100_configuration(void)
{
	GPIO_Configuration_OPP50	(SDQ_Port,	SDQ_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
}
/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读48字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_read_id(unsigned char* pbuffer)
{
	unsigned char i = 0;
	unsigned char Family_Code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	bq26100_rest();	//复位Dallas,返回结果
	if(sdq_error	==	result)
		return result;
	//------------写入命令
	sdq_write_byte(sdq_rom_readID);
	//------------确认器件
	Family_Code	=	sdq_read_byte();	
	if(bq26100_Family_Code	!= Family_Code)
		return sdq_error;
	//------------读取48位ID
	for(i=0;i<6;i++)
		pbuffer[i]=sdq_read_byte();
	//------------读一字节CRC
	sdq_read_byte();
	return sdq_success;
}

/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_skip_id(void)
{
	unsigned char i = 0;
	unsigned char Family_Code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	bq26100_rest();	//复位Dallas,返回结果
	if(sdq_error	==	result)
		return result;
	//------------写入命令
	sdq_write_byte(sdq_rom_skipID);
	return sdq_success;
}
/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_read_memory(unsigned char* pbuffer)
{
	unsigned short i = 0;
	unsigned char Family_Code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	api_bq26100_skip_id();
	if(sdq_error	==	result)
		return result;
	sdq_write_byte(sdq_mem_read_memory);
	//------------写入地址
	sdq_write_byte(0x00);
	sdq_write_byte(0x00);
	sdq_write_byte(0x9A);
//	//------------确认器件
	Family_Code	=	sdq_read_byte();	
//	if(bq26100_Family_Code	!= Family_Code)
//		return sdq_error;
//	//------------读取64位ID
	for(i=0;i<0x7F;i++)
	sdq_read_byte();
		pbuffer[i]=sdq_read_byte();
	return sdq_success;
}
/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_write_memory(unsigned char* pbuffer)
{
	unsigned short i = 0;
	unsigned char Family_Code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	api_bq26100_skip_id();
	if(sdq_error	==	result)
		return result;
	sdq_write_byte(sdq_mem_write_memory);
	//------------写入地址
	sdq_write_byte(0x00);
	sdq_write_byte(0x00);
	sdq_write_byte(0x9A);
//	//------------确认器件
	Family_Code	=	sdq_read_byte();	
//	if(bq26100_Family_Code	!= Family_Code)
//		return sdq_error;
//	//------------读取64位ID
	for(i=0;i<0x7F;i++)
	sdq_read_byte();
		pbuffer[i]=sdq_read_byte();
	return sdq_success;
}
/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_read_page(unsigned char* pbuffer)
{
	unsigned short i = 0;
	unsigned char read_code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	api_bq26100_skip_id();
	if(sdq_error	==	result)
		return result;
	sdq_write_byte(sdq_mem_read_page);
	//------------写入地址
	sdq_write_byte(0x00);
	sdq_write_byte(0x00);
	//------------Master RX:
	read_code	=	sdq_read_byte();	
//	//------------读取64位ID
	for(i=0;i<0x7F;i++)
		sdq_read_byte();
		//pbuffer[i]=sdq_read_byte();
	return sdq_success;
}
//------------------------------------------------------------------------------





/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_read_status(unsigned char* pbuffer)
{
	unsigned short i = 0;
	unsigned char read_code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	api_bq26100_skip_id();
	if(sdq_error	==	result)
		return result;
	sdq_write_byte(sdq_mem_read_status);
	//------------写入地址
	sdq_write_byte(0x03);
	sdq_write_byte(0x00);
	//------------Master RX:
	read_code	=	sdq_read_byte();	
//	//------------读取64位ID
	for(i=0;i<0x7F;i++)
		sdq_read_byte();
		//pbuffer[i]=sdq_read_byte();
	return sdq_success;
}
/*******************************************************************************
*函数名			:	api_read_id
*功能描述		:	读64字节器件电子标签
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result api_bq26100_write_status(unsigned char* pbuffer)
{
	unsigned short i = 0;
	unsigned char read_code	=	0;
	sdq_result	result = sdq_error;
	//------------启动总线
	result	=	api_bq26100_skip_id();
	if(sdq_error	==	result)
		return result;
	sdq_write_byte(sdq_mem_read_page);
	//------------写入地址
	sdq_write_byte(0x00);
	sdq_write_byte(0x00);
	//------------Master RX:
	read_code	=	sdq_read_byte();	
//	//------------读取64位ID
	for(i=0;i<0x7F;i++)
		sdq_read_byte();
		//pbuffer[i]=sdq_read_byte();
	return sdq_success;
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
sdq_result bq26100_rom_fun(sdq_rom_cmd rom_cmd)
{
	sdq_result	result	=	sdq_error;
	if(sdq_rom_readID	==	rom_cmd)
	{
	}
	else if(sdq_rom_matchID	==	rom_cmd)
	{
	}
	else if(sdq_rom_skipID	==	rom_cmd)
	{
	}
	else if(sdq_rom_searchID	==	rom_cmd)
	{
	}
	else
	{
		result	=	sdq_error;
	}
	return result;
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
sdq_result bq26100_mem_fun(sdq_mem_cmd mem_cmd)
{
	sdq_result	result	=	sdq_error;
	if(sdq_mem_read_memory	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_memory	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_page	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_digest	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_message	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_status	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_status	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_control	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_control	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_eeprom	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_eeprom	==	mem_cmd)
	{
	}	
	else if(sdq_mem_profile	==	mem_cmd)
	{
	}
	else if(sdq_mem_read_page4	==	mem_cmd)
	{
	}
	else if(sdq_mem_write_page4	==	mem_cmd)
	{
	}
	else
	{
		result	=	sdq_error;
	}
	return result;
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
void sdq_write_bit(unsigned char bit)		//1-wire 一位（1bit）写操作-写0&写1
{
	//总时间按80us
	// =============Write 1
	if (bit&0x01)		
	{
		SDQ_L;					//拉低总线
		SysTick_DeleyuS(10);		//至少维持了1us,表示写时序(包括写0时序或写1时序)开始 
		SDQ_H;					//拉高总线
		SysTick_DeleyuS(80);	//等待从机采样完成
	}
	// =============Write 0
	else						
	{
		SDQ_L;					//拉低总线
		SysTick_DeleyuS(85);	//保持60us，等待从机采样
		SDQ_H;					//释放总线
		SysTick_DeleyuS(5);
	}
	SysTick_DeleyuS(2);
}
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
unsigned char sdq_read_bit(void)
{
	//总时间按80us
	unsigned char bit=0;	
	SDQ_SetOut;				//设为输出模式
	SDQ_L; 						//拉低总线
	//SysTick_DeleyuS(15);		//至少1us时间
//	SDQ_H; 						//释放总线
	SDQ_SetIn;					//设置为输入模式
	//SDQ_H; 						//释放总线
	SysTick_DeleyuS(15);	//等待从机响应

	if(0!=SDQ_Read)				//读取总线状态
		bit = 1;
	else
		bit = 0;	
	SysTick_DeleyuS(60);
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
void sdq_write_byte(unsigned char dat)
{             
	unsigned char j;
	unsigned char testb;
	
	SDQ_SetOut;							//SET PG11 OUTPUT;
	
	for (j=1; j<=8; j++)
	{
		testb = dat&0x01;
		dat = dat>>1;
		sdq_write_bit(testb);		//1-wire 一位（1bit）写操作-写0&写1
	}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	从Dallas读取一个字节---从低位开始读取
* 输入			: void
* 返回值			: 读到的数据
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char sdq_read_byte(void)
{        
	unsigned char i,j,dat;
	SDQ_SetIn;
	dat = 0;
	for (i=1; i<=8; i++)
	{
		j = sdq_read_bit();		//读取一位数据
		dat = (j<<7)|(dat>>1);	//从低位开始保存数据
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
sdq_result bq26100_rest(void)		//复位Dallas,返回结果
{
	unsigned short retry=0;
	//----------------------复位时间：拉低信号大于480us
	SDQ_H;
	SDQ_SetOut;							//SET PG11 OUTPUT
	SDQ_H;
	SysTick_DeleyuS(10);		//拉低750us（大于480uS)
	SDQ_L;									//拉低DQ
	SysTick_DeleyuS(750);		//拉低750us（大于480uS)
//	SDQ_H;								//DQ=1 
//	SysTick_DeleyuS(15);		//15US---检测响应需要在15uS后
	
	//----------------------检测响应：总时间需要大于480us
	//1释放总线
	//2等待15us~60uS
	//3从机拉低总线60uS~240uS
	//4从机释放/拉高总线
	SDQ_SetIn;				//SET PG11 INPUT		//输入模式
	SysTick_DeleyuS(15);		//15US---检测响应需要在15uS后
	while(SDQ_Read	&& (retry < 260))			//响应检测时间不超过240uS
	{
		retry++;
		SysTick_DeleyuS(1);
	}
	if(retry >= 250)	//超时
		return sdq_error;
	else
		retry=0;
	//增加延时--为了满足总的检测时间480uS
	SysTick_DeleyuS(480);		//总的检测响应时间需要大于480uS
	return sdq_success;
}
//------------------------------------------------------------------------------






//------------------------------------------------------------------------------



/**********************************************************************/
/* 	unsigned char TestPresence(void)							      */
/*																      */
/*	Description : 		Detects if a device responds to Reset signal  */
/* 	Arguments : 		PresenceTimer - Sets timeout if no device	  */
/*							present									  */
/*						InputData - Actual state of GPIO			  */
/*						GotPulse - States if a pulse was detected	  */
/*	Global Variables:	None   										  */
/*  Returns: 			GotPulse         							  */
/**********************************************************************/
unsigned char TestPresence(void)
{
	unsigned int PresenceTimer;
	static volatile unsigned char InputData;
	static volatile unsigned char GotPulse;

	SDQ_SetIn;		//Set GPIO P9.3 as Input
	PresenceTimer = 300;	//Set timeout, enough time to allow presence pulse
	GotPulse = 0;			//Initialize as no pulse detected
	while ((PresenceTimer > 0) && (GotPulse == 0))
	{
		InputData = SDQ_Read;	//Monitor logic state of GPIO 
		if (InputData == 0) 
		{  		//If GPIO is Low,
			GotPulse = 1;			//it means that device responded
		}
		else 
		{						//If GPIO is high
			GotPulse = 0;			//it means that device has not responded
			--PresenceTimer;		//Decrease timeout until enough time has been allowed for response
		}
	}
	SysTick_DeleyuS(200);					//Wait some time to continue SDQ communication
	return GotPulse;				//Return if device detected or not
}
//--------------------------------------------------------------------------







/**********************************************************************/
//*	unsigned long Rotl(unsigned long x, int n)
//*																      
//*	Description : 		This procedure is a rotate left n spaces of	32-bit word x.
//*	Arguments : 		x - word to be rotated n - amount of spaces to rotated to the left
//*	Global Variables:	None
//*	Returns: 			Result of 32-bit word rotated n times
/**********************************************************************/
unsigned long Rotl(unsigned long x, int n)
{
	return (x<<n) |  (x>>(32-n));
}
unsigned long rRotl(unsigned long x, int n)
{
	return (x>>n) |  (x<<(32-n));
}
/**********************************************************************/
//* 	unsigned long W(int t)
//*	Description : 		This procedure determines the work schedule for W(16) through W(79)
//* 	Arguments : 		t - index of work schedule 16 through 79
//*	Global Variables:	Ws[]
//*  Returns: 			Work schedule value with index t
/**********************************************************************/
unsigned long W(int t)
{
	return Rotl(Ws[t-3] ^ Ws[t-8] ^ Ws[t-14] ^ Ws[t-16], 1);
}	

/**********************************************************************/
//*	unsigned long K(int t)
//*																      */
//*	Description : 		This procedure selects one of the K values depending on the index t.
//*	Arguments : 		t - index 
//*	Global Variables:	None
//*	Returns: 			One of the 4 K values
/**********************************************************************/
unsigned long K(int t)
{
	if (t<=19)
		return 0x5a827999;
	else if (t>=20 && t<=39)
		return 0x6ed9eba1;
	else if (t>=40 && t<=59)
		return 0x8f1bbcdc;
	else if (t>=60 && t<=79)
		return 0xca62c1d6;
	else
		return 0;		//Invalid value, not expected
}
	
/*******************************************************************************/
//*	unsigned long f(unsigned long x, unsigned long y, unsigned long z, int t)
//*																               */
//*	Description : 		This procedure selects the ft(b,c,d) function based on SLUA389 and FIPS 180-2 document.
//*	Arguments : 	x - b as seen in document 
//*								y - c as seen in document
//*             	z - d as seed in document
//*              	t - index
//*	Global Variables:	None
//*	Returns: 			Result of ft function
/*******************************************************************************/
unsigned long f(u32 x, u32 y, u32 z, int t)
{
	if (t<=19)
		return (x & y) ^ ((~x) & z);
	else if (t>=20 && t<=39)
		return x ^ y ^ z;
	else if (t>=40 && t<=59)
		return (x & y) ^ (x & z) ^ (y & z);
	else if (t>=60 && t<=79)
		return x ^ y ^ z;
	else
		return 0;       //Invalid value, not expected
}
//-----------------------------------------------------------------------------




/*******************************************************************************
*函数名			:	bq26100_step1_send_message
*功能描述		:	Send 160-bit Message to bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result bq26100_step1_send_message(const unsigned char* message,unsigned char len)
{
	unsigned long i = 0;
	unsigned char Value	=	0;
	sdq_result result;
	// Send 160-bit Message to bq26100
	result	=	bq26100_rest();
	if(sdq_error	==	result)
		return sdq_error;	
	sdq_write_byte(0xCC);			// Skip ROM Command
	sdq_write_byte(0x22);			//Write Message Command
	sdq_write_byte(0x00);			//Address Low Byte
	sdq_write_byte(0x00);			//Address High Byte
	
		// Write first byte of message, Message[0]
	sdq_write_byte(message[0x00]);	
	Value = sdq_read_byte();	//Read CRC of Message Command, Address and Data
	//CRC are not being calculated by the microcontroller, the results given by
	//bq26100 are being ignored
	// Read Data to Verify Write
	Value = sdq_read_byte();
	// Write the remaining bytes of the message
	for (i=1; i<=19; i++)
	{
		sdq_write_byte(message[i]);
		Value	=	sdq_read_byte();		//Read CRC of Message Command, Address and Data
																//CRC are not being calculated by the microcontroller, the results given by
																//bq26100 are being ignored
		Value	=	sdq_read_byte();		// Read Data to Verify Write		
	}
	return sdq_success;
}
/*******************************************************************************
*函数名			:	bq26100_step2_write_control
*功能描述		:	Write Control to set AUTH bit to initiate Authentication by bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result bq26100_step2_write_control(void)
{
	unsigned char Value	=	0;
	unsigned char i = 0;
	sdq_result result;
	// Write Control to set AUTH bit to initiate Authentication by bq26100
	result	=	bq26100_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//Value = TestPresence();
	// Skip ROM Command
	sdq_write_byte(0xCC);
	// Write Control Command
	sdq_write_byte(sdq_mem_write_control);	//Write Control Command
	//bq26100_reg_addr
//	sdq_write_byte(bq26100_reg_addr&0xFF);	//Address Low Byte
//	sdq_write_byte((bq26100_reg_addr>>8)&0xFF);	//Address High Byte
	sdq_write_byte(0x00);	//Address Low Byte
	sdq_write_byte(0x00);	//Address High Byte	
//	sdq_write_byte(0x05);			// Write Auth Bit of Control Register
//	Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data
	// Read Data to Verify Write
	//----------读数据，直到读到低位为1,最多重试100次
	for(i=0;i<100;i++)
	{
		sdq_write_byte(0x05);			// Write Auth Bit of Control Register
		Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data
		Value = sdq_read_byte();	//Read Data，直到读到低位为1
		if(0x01==(Value&0x01))
			break;
	}
	Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data	

	return sdq_success;
}
/*******************************************************************************
*函数名			:	bq26100_step3_read_control
*功能描述		:	Write Control to set AUTH bit to initiate Authentication by bq26100
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result bq26100_step3_read_control(void)
{
	unsigned char Value	=	0;
	unsigned char i=0;
	sdq_result result;
	// Write Control to set AUTH bit to initiate Authentication by bq26100
	result	=	bq26100_rest();
	if(sdq_error	==	result)
		return sdq_error;
	//Value = TestPresence();
	
	sdq_write_byte(0xCC);				// Skip ROM Command
	sdq_write_byte(sdq_mem_read_control);	//Write Control Command
	sdq_write_byte(0x00);			//Address Low Byte
	sdq_write_byte(0x00);			//Address High Byte
	Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data	
	//----------读数据，直到读到低位为1,最多重试100次
	for(i=0;i<100;i++)
	{
		Value = sdq_read_byte();	//Read Data，直到读到低位为1
		if(0x01==(Value&0x01))
			break;
	}
	Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data
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
sdq_result bq26100_step4_read_digest(unsigned char* digest)
{
	unsigned char i = 0;
	unsigned char Value	=	0;
	sdq_result result;
	//Read Digest
	result	=	bq26100_rest();
	if(sdq_error	==	result)
		return sdq_error;	
	sdq_write_byte(0xCC);			// Skip ROM Command
	sdq_write_byte(0xDD);			//Read Digest Command
	sdq_write_byte(0x00);			//Address Low Byte
	sdq_write_byte(0x00);			//Address High Byte
	Value = sdq_read_byte();	//Read CRC of Read Digest Command, Address
	// Read Digest
	for (i=0; i<=19; i++)
	{
		digest[i] = sdq_read_byte();
	}
	// Read CRC of Digest
	Value = sdq_read_byte();

	return sdq_success;
}
//------------------------------------------------------------------------------
/*******************************************************************************
*函数名			:	bq26100_step5_Auth
*功能描述		:	本程序按照BQ26100的要求计算SHA1/HMAC
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
* 	void Auth(void)
*														
*	Description : 		This procedure computes the SHA1/HMAC as
*				  		required by the bq26100
* 	Arguments : 		i - times that SHA1 is executing			 
*						t - index 0 through 79
*     					temp - Used to update working variables
*	Global Variables:	Random[], Message[], Key[], Ws[], H[],
*						A, B, C, D, E	
*  Returns: 			Result of 32-bit word rotated n times
*******************************************************************************/
void bq26100_step5_Auth(void)
{
	int i;			//Used for doing two times the SHA1 as required by the bq26100
	int t;			//Used for the indexes 0 through 79
	u32 temp;		//Used as the temp variable during the loop in which the working
							//variables A, B, C, D and E are updated
	
	// The 20 bytes of random message that are given to the bq26100 are arranged in 32-bit words
	// so that the microcontroller can compute the SHA1/HMAC
	Random[0] = Message[0x10] + Message[0x11]*0x0100 + Message[0x12]*0x010000 + Message[0x13]*0x01000000;
	Random[1] = Message[0x0C] + Message[0x0D]*0x0100 + Message[0x0E]*0x010000 + Message[0x0F]*0x01000000;
	Random[2] = Message[0x08] + Message[0x09]*0x0100 + Message[0x0A]*0x010000 + Message[0x0B]*0x01000000;
	Random[3] = Message[0x04] + Message[0x05]*0x0100 + Message[0x06]*0x010000 + Message[0x07]*0x01000000;
	Random[4] = Message[0x00] + Message[0x01]*0x0100 + Message[0x02]*0x010000 + Message[0x03]*0x01000000;
	// The SHA1 is computed two times so that it complies with the bq26100 specification
	for (i=0; i<=1; i++)	//转换两次
	{
		//-------------------加载密钥
		// Work Schedule
		// The first four Working schedule variables Ws[0-3], are based on the key that is 
		// implied that the bq26100 contains.
		Ws[0] = Key[0x0C] + Key[0x0D]*0x0100 + Key[0x0E]*0x010000 + Key[0x0F]*0x01000000;
		Ws[1] = Key[0x08] + Key[0x09]*0x0100 + Key[0x0A]*0x010000 + Key[0x0B]*0x01000000;
		Ws[2] = Key[0x04] + Key[0x05]*0x0100 + Key[0x06]*0x010000 + Key[0x07]*0x01000000;
		Ws[3] = Key[0x00] + Key[0x01]*0x0100 + Key[0x02]*0x010000 + Key[0x03]*0x01000000;
		//-------------------第一次转换加载消息，第二将转换加载第一次转换的消息摘要
		// On the first run of the SHA1 the random message is used 		
		if (i==0)
		{
			Ws[4] = Random[0];
			Ws[5] = Random[1];
			Ws[6] = Random[2];
			Ws[7] = Random[3];
			Ws[8] = Random[4];
		}
		// On the second run of the SHA1, H(Kd || M) is used		
		else
		{
			Ws[4] = H[0];
			Ws[5] = H[1];
			Ws[6] = H[2];
			Ws[7] = H[3];
			Ws[8] = H[4];
		}
		//-------------------补位操作:
		//Sha-1算法标准规定，必须对消息摘要进行补位操作，即将输入的数据进行填充，使得数据长度对512求余的结果为448，
		//填充比特位的最高位补一个1，其余的位补0，如果在补位之前已经满足对512取模余数为448，也要进行补位，在其后补一位1即可。总之，补位是至少补一位，最多补512位
		// The Work schedule variables Ws[9-15] remain the same regardless of which run of the SHA1.
		// These values are as required by bq26100.	//以下为按照bq26100规定的数值
		Ws[9]	= 0x80000000;		//输入消息长度对512取模余数结果非448(512-64)
		Ws[10] = 0x00000000;
		Ws[11] = 0x00000000;
		Ws[12] = 0x00000000;
		Ws[13] = 0x00000000;
		//--------------------附加信息长度(64位):表示消息摘要的字节数，
		Ws[14] = 0x00000000;
		Ws[15] = 0x00000120;		//总共长度为0x0120(288=160+128)密钥和消息的长度
		
		//--------------------补充混合数据
		// The Work schedule variables Ws[16-79] are determined by the W(t) function	
		for (t = 16; t <= 79; t++)
			Ws[t]=W(t);
		//--------------------初始化缓存
		//一个160位MD缓冲区用以保存中间和最终散列函数的结果。它可以表示为5个32位的寄存器(H0,H1,H2,H3,H4)。初始化为：
		// Working Variables, always start the same	regardless of which SHA1 run
		A = 0x67452301;
		B = 0xefcdab89;
		C = 0x98badcfe;
		D = 0x10325476;
		E = 0xc3d2e1f0;
		// Hash Values, always start the same regardless of what SHA1 run
		H[0] = A;
		H[1] = B;
		H[2] = C;
		H[3] = D;
		H[4] = E;
		//---------------------循环转换数据
		// Loop to change working variables A, B, C, D and E
		// This is defined by FIPS 180-2 document
		for (t = 0; t <= 79; t++)
		{
				temp = Rotl(A,5) + f(B,C,D,t) + E + K(t) + Ws[t];
				E = D;
				D = C;
				C = Rotl(B,30);
				B = A;
				A = temp;
		}
		//----------------------转换出验证结果:转换再次，第一次结果参当作第二次转换的数据
		// 160-Bit SHA-1 Digest
		H[0] = A + H[0];
		H[1] = B + H[1];
		H[2] = C + H[2];
		H[3] = D + H[3];
		H[4] = E + H[4];
	}//End of for loop
}	//End Auth() function
//------------------------------------------------------------------------------


/*******************************************************************************
*函数名			:	bq26100_step6_verify
*功能描述		:	Read Digest
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
sdq_result bq26100_step6_verify(unsigned char* digest)
{
	// The 20 bytes of the digest returned by the bq26100 is arranged in 32-bit words so that it
	// can be compared with the results computed by the microcontroller
	
	Digest_32[4] = digest[0x00] + digest[0x01]*0x0100 + digest[0x02]*0x010000 + digest[0x03]*0x01000000;
	Digest_32[3] = digest[0x04] + digest[0x05]*0x0100 + digest[0x06]*0x010000 + digest[0x07]*0x01000000;
	Digest_32[2] = digest[0x08] + digest[0x09]*0x0100 + digest[0x0A]*0x010000 + digest[0x0B]*0x01000000;
	Digest_32[1] = digest[0x0C] + digest[0x0D]*0x0100 + digest[0x0E]*0x010000 + digest[0x0F]*0x01000000;
	Digest_32[0] = digest[0x10] + digest[0x11]*0x0100 + digest[0x12]*0x010000 + digest[0x13]*0x01000000;
	if ((Digest_32[0] == H[0])&&(Digest_32[1] == H[1])&&(Digest_32[2] == H[2])&&(Digest_32[3] == H[3]))
	{
		return sdq_success;
		//------------------------验证通过
	}
	return sdq_error;
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
unsigned char api_bq26100_get_digest(const unsigned char* message,unsigned char* digest)
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
	result	=	bq26100_step1_send_message(message,20);
	if(result	==	sdq_error)
		return 0;
	//--------------------
	result	=	bq26100_step2_write_control();
	//bq26100_step3_read_control();
	
	if(result	==	sdq_error)
		return 0;
	//--------------------读取bq26100生成的SHA1/HMAC并存储在digest
	result	=	bq26100_step4_read_digest(digest);
	
	return 1;
}
//---------------------------------------------------------------------------





/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_bq26100_message_verify(const unsigned char* message,unsigned char* digest)
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
	result	=	bq26100_step1_send_message(message,20);
	if(result	==	sdq_error)
		return 0;
	//--------------------
	result	=	bq26100_step2_write_control();
	//bq26100_step3_read_control();
	
	if(result	==	sdq_error)
		return 0;
	//--------------------读取bq26100生成的SHA1/HMAC并存储在digest
	result	=	bq26100_step4_read_digest(digest);
	
	return 1;
}
//---------------------------------------------------------------------------











/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void bq26100_set_key(void)
{
	// Select the key to be used here. In this example 0x3601FCFB12B87356C1548630FD3EA0D2 is used.  
	Key[0] = 0xD2;
	Key[1] = 0xA0;
	Key[2] = 0x3E;
	Key[3] = 0xFD;
	Key[4] = 0x30;
	Key[5] = 0x86;
	Key[6] = 0x54;
	Key[7] = 0xC1;
	Key[8] = 0x56;
	Key[9] = 0x73;
	Key[10] = 0xB8;
	Key[11] = 0x12;
	Key[12] = 0xFB;
	Key[13] = 0xFC;
	Key[14] = 0x01;
	Key[15] = 0x36;
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
static void bq26100_set_message(void)
{
	unsigned char i = 0;
	static unsigned char data=0;
	//static unsigned char time=0;
	//-----------------------------随机数
	srand(0);
	for (i=0; i<=19; i++)
	{
		Message[i] = rand()%256;		//获取随机数
	}
	//-----------------------------上电启动:验证通过
	Message[0]	=	0xb8;
	Message[1]	=	0xd3;
	Message[2]	=	0xaa;
	Message[3]	=	0x20;
	Message[4]	=	0x82;
	Message[5]	=	0xfe;
	Message[6]	=	0xd1;
	Message[7]	=	0xf0;
	Message[8]	=	0x7c;
	Message[9]	=	0x0a;
	Message[10]	=	0x07;
	Message[11]	=	0x8e;
	Message[12]	=	0x26;
	Message[13]	=	0xc7;
	Message[14]	=	0xde;
	Message[15]	=	0x10;
	Message[16]	=	0xca;
	Message[17]	=	0xa3;
	Message[18]	=	0x2c;
	Message[19]	=	0xc8;	
	//-----------------------------逻辑分析仪001:验证通过
	//0xEC 0x8D 0x31 0x3B 0xCF 0xAD 0x8A 0x2E 0xAA 0xF9 0xAA 0x26 0xC1 0xFF  0xF1 0xD2 0x95 0x91 0xAA 0xEA 
	Message[0]	=	0xEC;
	Message[1]	=	0x8D;
	Message[2]	=	0x31;
	Message[3]	=	0x3B;
	Message[4]	=	0xCF;
	Message[5]	=	0xAD;
	Message[6]	=	0x8A;
	Message[7]	=	0x2E;
	Message[8]	=	0xAA;
	Message[9]	=	0xF9;
	Message[10]	=	0xAA;
	Message[11]	=	0x26;
	Message[12]	=	0xC1;
	Message[13]	=	0xFF;
	Message[14]	=	0xF1;
	Message[15]	=	0xD2;
	Message[16]	=	0x95;
	Message[17]	=	0x91;
	Message[18]	=	0xAA;
	Message[19]	=	0xEA;
	//-----------------------------逻辑分析仪002:
	//0x4D 0x9B 0x16 0x28 0x66 0x43 0xB9 0x85 0x9C 0x46 0xE2 0xB7 0x1A 0x91 0x7F 0xEA 0xE0 0x1C 0x21 0xBD
	Message[0]	=	0x4D;
	Message[1]	=	0x9B;
	Message[2]	=	0x16;
	Message[3]	=	0x28;
	Message[4]	=	0x66;
	Message[5]	=	0x43;
	Message[6]	=	0xB9;
	Message[7]	=	0x85;
	Message[8]	=	0x9C;
	Message[9]	=	0x46;
	Message[10]	=	0xE2;
	Message[11]	=	0xB7;
	Message[12]	=	0x1A;
	Message[13]	=	0x91;
	Message[14]	=	0x7F;
	Message[15]	=	0xEA;
	Message[16]	=	0xE0;
	Message[17]	=	0x1C;
	Message[18]	=	0x21;
	Message[19]	=	0xBD;
	
	//-----------------------------测试数据3:通过验证
	//83 b2 06 2e 21 60 79 00 f3 60 b0 f1 b7 2e 27 bc ff a7 e3 f1
	Message[0]	=	0x83;
	Message[1]	=	0xB2;
	Message[2]	=	0x06;
	Message[3]	=	0x2E;
	Message[4]	=	0x21;
	Message[5]	=	0x60;
	Message[6]	=	0x79;
	Message[7]	=	0x00;
	Message[8]	=	0xF3;
	Message[9]	=	0x60;
	Message[10]	=	0xB0;
	Message[11]	=	0xF1;
	Message[12]	=	0xB7;
	Message[13]	=	0x2E;
	Message[14]	=	0x27;
	Message[15]	=	0xBC;
	Message[16]	=	0xFF;
	Message[17]	=	0xA7;
	Message[18]	=	0xE3;
	Message[19]	=	0xF1;
	
	//-----------------------------测试数据1：不通过
	Message[0]	=	0x34;
	Message[1]	=	0xb9;
	Message[2]	=	0x03;
	Message[3]	=	0x4c;
	Message[4]	=	0xf9;
	Message[5]	=	0x9c;
	Message[6]	=	0xcf;
	Message[7]	=	0x8e;
	Message[8]	=	0xe7;
	Message[9]	=	0x43;
	Message[10]	=	0xe9;
	Message[11]	=	0xf2;
	Message[12]	=	0x62;
	Message[13]	=	0x8a;
	Message[14]	=	0xc1;
	Message[15]	=	0x8b;
	Message[16]	=	0x32;
	Message[17]	=	0x92;
	Message[18]	=	0x48;
	Message[19]	=	0x46;
	
	//-----------------------------测试数据1:摘要
	Message[0]	=	0xda;
	Message[1]	=	0x31;
	Message[2]	=	0x55;
	Message[3]	=	0x86;
	Message[4]	=	0x0e;
	Message[5]	=	0x90;
	Message[6]	=	0xa3;
	Message[7]	=	0x1f;
	Message[8]	=	0x88;
	Message[9]	=	0xd6;
	Message[10]	=	0xc9;
	Message[11]	=	0xdc;
	Message[12]	=	0x41;
	Message[13]	=	0x6d;
	Message[14]	=	0xd3;
	Message[15]	=	0x1c;
	Message[16]	=	0x4a;
	Message[17]	=	0xd6;
	Message[18]	=	0x59;
	Message[19]	=	0xf6;
	
	//-----------------------------测试数据2:不通过
	Message[0]	=	0x95;
	Message[1]	=	0x79;
	Message[2]	=	0x80;
	Message[3]	=	0x1c;
	Message[4]	=	0x75;
	Message[5]	=	0x80;
	Message[6]	=	0x94;
	Message[7]	=	0x58;
	Message[8]	=	0x41;
	Message[9]	=	0xb1;
	Message[10]	=	0x31;
	Message[11]	=	0x37;
	Message[12]	=	0x96;
	Message[13]	=	0x92;
	Message[14]	=	0x91;
	Message[15]	=	0xdb;
	Message[16]	=	0x34;
	Message[17]	=	0x9d;
	Message[18]	=	0x1d;
	Message[19]	=	0x5a;
	
	//-----------------------------测试数据2:不通过
	//80 58 a6 4d 42 72 3b 4f 7e d1 56 bd 72 c5 4a cc d8 cc 6c 4b
	Message[0]	=	0x80;
	Message[1]	=	0x58;
	Message[2]	=	0xa6;
	Message[3]	=	0x4d;
	Message[4]	=	0x42;
	Message[5]	=	0x72;
	Message[6]	=	0x3b;
	Message[7]	=	0x4f;
	Message[8]	=	0x7e;
	Message[9]	=	0xd1;
	Message[10]	=	0x56;
	Message[11]	=	0xbd;
	Message[12]	=	0x72;
	Message[13]	=	0xc5;
	Message[14]	=	0x4a;
	Message[15]	=	0xcc;
	Message[16]	=	0xd8;
	Message[17]	=	0xcc;
	Message[18]	=	0x6c;
	Message[19]	=	0x4b;
	
	//-----------------------------逻辑分析仪007:验证通过
	Message[0]	=	0xFD;
	Message[1]	=	0xAB;
	Message[2]	=	0x14;
	Message[3]	=	0xB6;
	Message[4]	=	0xCE;
	Message[5]	=	0x0A;
	Message[6]	=	0x28;
	Message[7]	=	0x41;
	Message[8]	=	0x2F;
	Message[9]	=	0x08;
	Message[10]	=	0x6D;
	Message[11]	=	0xD4;
	Message[12]	=	0xDB;
	Message[13]	=	0x62;
	Message[14]	=	0x89;
	Message[15]	=	0xB0;
	Message[16]	=	0x77;
	Message[17]	=	0x30;
	Message[18]	=	0x59;
	Message[19]	=	0x5D;
	
	
	//-----------------------------测试数据2:
	//0xA5,0xE7,0x6E,0x31,0x81,0xCA,0xD3,0x63,0x19,0x3C,0xFD,0x17,0x40,0x07,0xA3,0xA7,0xAA,0xA9,0x96,0x08
	Message[0]	=	0xA5;
	Message[1]	=	0xE7;
	Message[2]	=	0x6E;
	Message[3]	=	0x31;
	Message[4]	=	0x81;
	Message[5]	=	0xCA;
	Message[6]	=	0xD3;
	Message[7]	=	0x63;
	Message[8]	=	0x19;
	Message[9]	=	0x3C;
	Message[10]	=	0xFD;
	Message[11]	=	0x17;
	Message[12]	=	0x40;
	Message[13]	=	0x07;
	Message[14]	=	0xA3;
	Message[15]	=	0xA7;
	Message[16]	=	0xAA;
	Message[17]	=	0xA9;
	Message[18]	=	0x96;
	Message[19]	=	0x08;
	//bq26100_change_message();
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
void api_bq26100_set_message2(unsigned char* buffer)
{
	unsigned char i = 0;

	for(i=0;i<20;i++)
	{
		Message[i]=buffer[i];
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
static void bq26100_change_message(void)
{
	static unsigned char num=0;
	unsigned char i =0;
	unsigned char buffer[20]={0};
	//memcpy(buffer,Message,20);
	memset(Message,0x00,20);
}

//-------------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void key_loop(void)
{
	if (key_key32[0]++ >=0xFFFFFFFF)
	{
		if(key_key32[1]++ >= 0xFFFFFFFF)
		{
			if (key_key32[2]++ >= 0xFFFFFFFF)
			{
				if (key_key32[3]++ >= 0xFFFFFFFF)
				{

				}
			}

		}
	}
	/*Ws[0] = Key[0x0C] + Key[0x0D] * 0x0100 + Key[0x0E] * 0x010000 + Key[0x0F] * 0x01000000;
	Ws[1] = Key[0x08] + Key[0x09] * 0x0100 + Key[0x0A] * 0x010000 + Key[0x0B] * 0x01000000;
	Ws[2] = Key[0x04] + Key[0x05] * 0x0100 + Key[0x06] * 0x010000 + Key[0x07] * 0x01000000;
	Ws[3] = Key[0x00] + Key[0x01] * 0x0100 + Key[0x02] * 0x010000 + Key[0x03] * 0x01000000;*/
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
unsigned char api_bq26100_test_example_getkey(void)
{
	static char flag=0;
	int i;			//Used for doing two times the SHA1 as required by the bq26100
	int t;			//Used for the indexes 0 through 79
	u32 temp;		//Used as the temp variable during the loop in which the working
							//variables A, B, C, D and E are updated
	if(flag!=0)
		return 1;
	key_loop();
	// The 20 bytes of random message that are given to the bq26100 are arranged in 32-bit words
	// so that the microcontroller can compute the SHA1/HMAC
//	Random[0] = key_message[4];
//	Random[1] = key_message[3];
//	Random[2] = key_message[2];
//	Random[3] = key_message[1];
//	Random[4] = key_message[0];
	// The SHA1 is computed two times so that it complies with the bq26100 specification
	for (i=0; i<=1; i++)	//转换两次
	{
		//-------------------加载密钥
		// Work Schedule
		// The first four Working schedule variables Ws[0-3], are based on the key that is 
		// implied that the bq26100 contains.
		Ws[0] = key_key32[3];
		Ws[1] = key_key32[2];
		Ws[2] = key_key32[1];
		Ws[3] = key_key32[0];
		//-------------------第一次转换加载消息，第二将转换加载第一次转换的消息摘要
		// On the first run of the SHA1 the random message is used 		
		if (i==0)
		{
			Ws[4] = key_message[0];
			Ws[5] = key_message[1];
			Ws[6] = key_message[2];
			Ws[7] = key_message[3];
			Ws[8] = key_message[4];
		}
		// On the second run of the SHA1, H(Kd || M) is used		
		else
		{
			Ws[4] = H[0];
			Ws[5] = H[1];
			Ws[6] = H[2];
			Ws[7] = H[3];
			Ws[8] = H[4];
		}
		//-------------------补位操作:
		//Sha-1算法标准规定，必须对消息摘要进行补位操作，即将输入的数据进行填充，使得数据长度对512求余的结果为448，
		//填充比特位的最高位补一个1，其余的位补0，如果在补位之前已经满足对512取模余数为448，也要进行补位，在其后补一位1即可。总之，补位是至少补一位，最多补512位
		// The Work schedule variables Ws[9-15] remain the same regardless of which run of the SHA1.
		// These values are as required by bq26100.	//以下为按照bq26100规定的数值
		Ws[9]	= 0x80000000;		//输入消息长度对512取模余数结果非448(512-64)
		Ws[10] = 0x00000000;
		Ws[11] = 0x00000000;
		Ws[12] = 0x00000000;
		Ws[13] = 0x00000000;
		//--------------------附加信息长度(64位):表示消息摘要的字节数，
		Ws[14] = 0x00000000;
		Ws[15] = 0x00000120;		//总共长度为0x0120(288=160+128)密钥和消息的长度
		
		//--------------------补充混合数据
		// The Work schedule variables Ws[16-79] are determined by the W(t) function	
		for (t = 16; t <= 79; t++)
			Ws[t]=W(t);
		//--------------------初始化缓存
		//一个160位MD缓冲区用以保存中间和最终散列函数的结果。它可以表示为5个32位的寄存器(H0,H1,H2,H3,H4)。初始化为：
		// Working Variables, always start the same	regardless of which SHA1 run
		A = 0x67452301;
		B = 0xefcdab89;
		C = 0x98badcfe;
		D = 0x10325476;
		E = 0xc3d2e1f0;
		// Hash Values, always start the same regardless of what SHA1 run
		H[0] = A;
		H[1] = B;
		H[2] = C;
		H[3] = D;
		H[4] = E;
		//---------------------循环转换数据
		// Loop to change working variables A, B, C, D and E
		// This is defined by FIPS 180-2 document
		for (t = 0; t <= 79; t++)
		{
				temp = Rotl(A,5) + f(B,C,D,t) + E + K(t) + Ws[t];
				E = D;
				D = C;
				C = Rotl(B,30);
				B = A;
				A = temp;
		}
		//----------------------转换出验证结果:转换再次，第一次结果参当作第二次转换的数据
		// 160-Bit SHA-1 Digest
		H[0] = A + H[0];
		H[1] = B + H[1];
		H[2] = C + H[2];
		H[3] = D + H[3];
		H[4] = E + H[4];
	}//End of for loop
	
	if ((key_digest[0] == H[0]) && (key_digest[1] == H[1]) && (key_digest[2] == H[2]) && (key_digest[3] == H[3]))
	{
		flag	=	1;
		return 1;//------------------------验证通过
	}
	return 1;
}
/*******************************************************************************
*函数名			:	get_up_E
*功能描述		:	获取E值
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned long bq26100_get_E(unsigned long* ARRY)
{
	unsigned long temp=0;
	
	return temp;
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
unsigned long api_bq26100_test_example_getkey2(unsigned long* H1,unsigned long* ARRY)
{
	//ARRY[0]=A,ARRY[1]=B,ARRY[2]=C,ARRY[3]=D,ARRY[4]=E,
	static unsigned char num=80;
	static unsigned char t=0;
	static unsigned long A1,B1,C1,D1,E1;
	unsigned long temp=0;
	if(num==0)
		return 0;
	//----------------导入最终结果数据:求出第80次转换的ABCDE结果
	if(80==num)
	{
		//-----计算A
		if(H1[0]>=0x67452301)
			ARRY[0]=H1[0]-0x67452301;
		else
			ARRY[0]=H1[0]+(0xFFFFFFFF-0x67452301)+1;
		//-----计算B
		if(H1[1]>=0xefcdab89)
			ARRY[1]=H1[1]-0xefcdab89;
		else
			ARRY[1]=H1[1]+(0xFFFFFFFF-0xefcdab89)+1;
		//-----计算C
		if(H1[2]>=0x98badcfe)
			ARRY[2]=H1[2]-0x98badcfe;
		else
			ARRY[2]=H1[2]+(0xFFFFFFFF-0x98badcfe)+1;
		//-----计算D
		if(H1[3]>=0x10325476)
			ARRY[3]=H1[3]-0x10325476;
		else
			ARRY[3]=H1[3]+(0xFFFFFFFF-0x10325476)+1;
		//-----计算E
		if(H1[4]>=0xc3d2e1f0)
			ARRY[4]=H1[4]-0xc3d2e1f0;
		else
			ARRY[4]=H1[4]+(0xFFFFFFFF-0xc3d2e1f0)+1;
						//0x2	0xE
		num--;
		return 0;
	}
	//=====往前推算
	//求上一次的A值:此次的B值
	ARRY[0]=ARRY[1];
	//求上一次的B值:此次的C值反移动
	ARRY[1]=rRotl(ARRY[2],30);
	//求上一次的C值:此次的D值
	ARRY[2]=ARRY[3];
	//求上一次的D值:此次的E值
	ARRY[3]=ARRY[4];
	//求上一次的E值:通过已经ABCD计算
	ARRY[4]=bq26100_get_E(ARRY);
	
	num--;
//	ARRY[0]=Rotl(H[0], num);
//	if(num<32)
//		num++;
	return temp;
}
//---------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_bq26100_test_example(void)
{
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
	unsigned char digest[20];
  sdq_result result;
	if(flag++>=5)
	{
		flag	=	0;
		//bq26100_reg_addr++;
	}
	//--------------------设置密钥
	bq26100_set_key();	
	
	//--------------------生成随机数消息
	srand(0);
	for (i=0; i<=19; i++)
	{
		Message[i] = rand()%256;		//获取随机数
	}
	
	
	//--------------------按照BQ26100的要求计算SHA1/HMAC
	//bq26100_step5_Auth();	
	
	bq26100_set_message();		//测试实际数据
	
	//--------------------发送消息
	result	=	bq26100_step1_send_message(Message,20);
	if(result	==	sdq_error)
		return ;
	bq26100_step4_read_digest(digest);
	bq26100_step3_read_control();
	//--------------------
	result	=	bq26100_step2_write_control();
	//bq26100_step3_read_control();
	
	if(result	==	sdq_error)
		return ;
	result	=	bq26100_step4_read_digest(digest);
	//--------------------读取bq26100生成的SHA1/HMAC并对比
	
	
	result	=	bq26100_step6_verify(digest);
}
void api_bq26100_test_example2(void)
{
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
	unsigned char digest[20];
  sdq_result result;
	if(flag++>=5)
	{
		flag	=	0;
		//bq26100_reg_addr++;
	}
	//--------------------设置密钥
	bq26100_set_key();	
	
	//--------------------生成随机数消息
	srand(0);
	for (i=0; i<=19; i++)
	{
		Message[i] = rand()%256;		//获取随机数
	}
	
	
	//--------------------按照BQ26100的要求计算SHA1/HMAC
	//bq26100_step5_Auth();	
	
	bq26100_set_message();		//测试实际数据
	
	//--------------------发送消息
	result	=	bq26100_step1_send_message(Message,20);
	if(result	==	sdq_error)
		return ;
	bq26100_step4_read_digest(digest);
	//--------------------
	result	=	bq26100_step2_write_control();
	//bq26100_step3_read_control();
	
	if(result	==	sdq_error)
		return ;
	//--------------------读取bq26100生成的SHA1/HMAC并对比
	result	=	bq26100_step4_read_digest(digest);
	
	result	=	bq26100_step6_verify(digest);
}
//---------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_bq26100_test_example22(unsigned long* buffer)
{
	#include "usb_data.h"
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
	unsigned char digest[20];
  sdq_result result;
//	if(flag++>=5)
//	{
//		flag	=	0;
//		//bq26100_reg_addr++;
//	}
//	//--------------------设置密钥
//	bq26100_set_key();	
//	
//	//--------------------生成随机数消息
//	srand(0);
//	for (i=0; i<=19; i++)
//	{
//		Message[i] = rand()%256;		//获取随机数
//	}
//	
//	
//	//--------------------按照BQ26100的要求计算SHA1/HMAC
//	bq26100_step5_Auth();	
	
//	bq26100_set_message();		//测试实际数据
	
	//--------------------发送消息
	result	=	bq26100_step1_send_message(Message,20);
	if(result	==	sdq_error)
		return ;
	//--------------------
	result	=	bq26100_step2_write_control();
	//bq26100_step3_read_control();
	
	if(result	==	sdq_error)
		return ;
	//--------------------读取bq26100生成的SHA1/HMAC并对比
	result	=	bq26100_step4_read_digest(digest);
	
	api_usb_in_set_data(Digest,20);
	
	for(i=0;i<5;i++)
	{
		buffer[i]=Digest_32[i];
	}
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------



//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------



/**********************************************************************/
/* 	void Auth(void)					      							  */
/*																      */
/*	Description : 		This procedure computes the SHA1/HMAC as 	  */
/*				  		required by the bq26100						  */
/* 	Arguments : 		i - times that SHA1 is executing			  */
/*						t - index 0 through 79						  */
/*     					temp - Used to update working variables		  */
/*	Global Variables:	Random[], Message[], Key[], Ws[], H[],		  */
/*						A, B, C, D, E								  */
/*  Returns: 			Result of 32-bit word rotated n times         */
/**********************************************************************/ 
void Auth(void)
{
	int i;			//Used for doing two times the SHA1 as required by the bq26100
	int t;			//Used for the indexes 0 through 79
	u32 temp;		//Used as the temp variable during the loop in which the working
					//variables A, B, C, D and E are updated
	
// The 20 bytes of random message that are given to the bq26100 are arranged in 32-bit words
// so that the microcontroller can compute the SHA1/HMAC
	Random[0] = Message[0x10] + Message[0x11]*0x0100 + Message[0x12]*0x010000 + Message[0x13]*0x01000000;
	Random[1] = Message[0x0C] + Message[0x0D]*0x0100 + Message[0x0E]*0x010000 + Message[0x0F]*0x01000000;
	Random[2] = Message[0x08] + Message[0x09]*0x0100 + Message[0x0A]*0x010000 + Message[0x0B]*0x01000000;
	Random[3] = Message[0x04] + Message[0x05]*0x0100 + Message[0x06]*0x010000 + Message[0x07]*0x01000000;
	Random[4] = Message[0x00] + Message[0x01]*0x0100 + Message[0x02]*0x010000 + Message[0x03]*0x01000000;
// The SHA1 is computed two times so that it complies with the bq26100 specification
	for (i=0; i<=1; i++)
	{
//Work Schedule
	// The first four Working schedule variables Ws[0-3], are based on the key that is 
	// implied that the bq26100 contains.
		Ws[0] = Key[0x0C] + Key[0x0D]*0x0100 + Key[0x0E]*0x010000 + Key[0x0F]*0x01000000;
		Ws[1] = Key[0x08] + Key[0x09]*0x0100 + Key[0x0A]*0x010000 + Key[0x0B]*0x01000000;
		Ws[2] = Key[0x04] + Key[0x05]*0x0100 + Key[0x06]*0x010000 + Key[0x07]*0x01000000;
		Ws[3] = Key[0x00] + Key[0x01]*0x0100 + Key[0x02]*0x010000 + Key[0x03]*0x01000000;
// On the first run of the SHA1 the random message is used 		
		if (i==0)
		{
			Ws[4] = Random[0];
			Ws[5] = Random[1];
			Ws[6] = Random[2];
			Ws[7] = Random[3];
			Ws[8] = Random[4];
		}
// On the second run of the SHA1, H(Kd || M) is used		
		else
		{
			Ws[4] = H[0];
			Ws[5] = H[1];
			Ws[6] = H[2];
			Ws[7] = H[3];
			Ws[8] = H[4];
		}
// The Work schedule variables Ws[9-15] remain the same regardless of which run of the SHA1.
// These values are as required by bq26100.
		Ws[9] = 0x80000000;
		Ws[10] = 0x00000000;
		Ws[11] = 0x00000000;
		Ws[12] = 0x00000000;
		Ws[13] = 0x00000000;
		Ws[14] = 0x00000000;
		Ws[15] = 0x00000120;

// The Work schedule variables Ws[16-79] are determined by the W(t) function	
		for (t = 16; t <= 79; t++)
			Ws[t]=W(t);
// Working Variables, always start the same	regardless of which SHA1 run
		A = 0x67452301;
		B = 0xefcdab89;
		C = 0x98badcfe;
		D = 0x10325476;
		E = 0xc3d2e1f0;
// Hash Values, always start the same regardless of what SHA1 run
		H[0] = A;
		H[1] = B;
		H[2] = C;
		H[3] = D;
		H[4] = E;
// Loop to change working variables A, B, C, D and E
// This is defined by FIPS 180-2 document
		for (t = 0; t <= 79; t++)
	{
			temp = Rotl(A,5) + f(B,C,D,t) + E + K(t) + Ws[t];
			E = D;
			D = C;
			C = Rotl(B,30);
			B = A;
			A = temp;
	}
// 160-Bit SHA-1 Digest
		H[0] = A + H[0];
		H[1] = B + H[1];
		H[2] = C + H[2];
		H[3] = D + H[3];
		H[4] = E + H[4];
	}//End of for loop
}	//End Auth() function
//-----------------------------------------------------------------------------






/**********************************************************************/
/* 	int main(void)					      							  */
/*																      */
/*	Description : 		This is the main function. It calls the SDQ	  */
/*				  		communication and the SHA1 function. Results  */
/*						are displayed through LEDs.					  */
/* 	Arguments : 		*Value - generic variable for read functions  */
/*						i - used for repeat loops				      */
/*	Global Variables:	Message[], Key[], Digest[], Digest_32[]		  */
/*  Returns: 			None								          */
/**********************************************************************/
void api_bq26100_test_examplebac(void)
{
	unsigned char Value;
	unsigned long i;
  	
// Select the key to be used here. In this example 0x3601FCFB12B87356C1548630FD3EA0D2 is used.  
	Key[0] = 0xD2;
	Key[1] = 0xA0;
	Key[2] = 0x3E;
	Key[3] = 0xFD;
	Key[4] = 0x30;
	Key[5] = 0x86;
	Key[6] = 0x54;
	Key[7] = 0xC1;
	Key[8] = 0x56;
	Key[9] = 0x73;
	Key[10] = 0xB8;
	Key[11] = 0x12;
	Key[12] = 0xFB;
	Key[13] = 0xFC;
	Key[14] = 0x01;
	Key[15] = 0x36;
	
	
/*	for (i=0; i<=15; i++){
  		Key[i] = 0x00;
  	}
*/ 
// Create a random message generator and insert here. In this example the message is fixed. 	
//	srand(0);
//	for (i=0; i<=19; i++)
//	{
//		Message[i] = rand()%256;		//获取随机数
//	}
	bq26100_set_message();	//生成消息
	Auth();	 		//Perform SHA1 authentication through host
   	
//// Initialize Port 9.3 for SDQ communication
//  	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
//  	GPIO_InitStruct.GPIO_Direction = GPIO_PinOutput;
//  	GPIO_InitStruct.GPIO_Type = GPIO_Type_PushPull;
//  	GPIO_InitStruct.GPIO_IPConnected = GPIO_IPConnected_Disable;
//  	GPIO_InitStruct.GPIO_Alternate = GPIO_OutputAlt1;
//  	GPIO_Init(GPIO9, &GPIO_InitStruct);

//// Initilaize ports 8.0 and 8.6 for LED driving
//  	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_All;
//  	GPIO_InitStruct.GPIO_Direction = GPIO_PinOutput;
//  	GPIO_InitStruct.GPIO_Type = GPIO_Type_PushPull;
//  	GPIO_InitStruct.GPIO_IPConnected = GPIO_IPConnected_Disable;
//  	GPIO_InitStruct.GPIO_Alternate = GPIO_OutputAlt1;
//  	GPIO_Init(GPIO8, &GPIO_InitStruct);

//  	GPIO_Write(GPIO8, 0x00);	//Ensure that all ports of GPIO 8 are low,
//							  	//given that they are connected to LCD and do not want
//							  	//disturb it.

  
	// Send 160-bit Message to bq26100
	bq26100_rest();	   			
//	*Value = TestPresence(); 		//Determine if there is a SDQ compatible device connected
//	if (*Value==0x00)	//No device connected or functional
//	{	
//		//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);
//		while (1)	//Blink red LED at a 0.5 sec on and 0.5 sec off rate
//		{				 		
////			GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_RESET);  	//Red LED off
////			wait_us(1e6/2);  					  			//wait approximately 0.5 seconds
////			GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 	  	//Red LED on
////			wait_us(1e6/2);									//wait approximately 0.5 seconds
//		}
//	}
	// Skip ROM Command
	sdq_write_byte(0xCC);
	// Write Message Command
	sdq_write_byte(0x22);	//Write Message Command
	// Write Address
	sdq_write_byte(0x00);	//Address Low Byte
	sdq_write_byte(0x00);	//Address High Byte
	// Write first byte of message, Message[0]
	sdq_write_byte(Message[0x00]);	
	Value = sdq_read_byte();	//Read CRC of Message Command, Address and Data
	//CRC are not being calculated by the microcontroller, the results given by
	//bq26100 are being ignored
	// Read Data to Verify Write
	Value = sdq_read_byte();
	// Write the remaining bytes of the message
	for (i=1; i<=19; i++)
	{
		sdq_write_byte(Message[i]);
		Value = sdq_read_byte();		//Read CRC
		Value = sdq_read_byte();		//Read Data
		
		
	}


	// Write Control to set AUTH bit to initiate Authentication by bq26100
	bq26100_rest();
	Value = TestPresence();
	// Skip ROM Command
	sdq_write_byte(0xCC);
	// Write Control Command
	sdq_write_byte(0x77);	//Write Control Command
	// Write Address
	sdq_write_byte(0x00);	//Address Low Byte
	sdq_write_byte(0x00);	//Address High Byte
	// Write Auth Bit of Control Register
	sdq_write_byte(0x01);	
	Value = sdq_read_byte();	//Read CRC of Write Control Command, Address and Data
	// Read Data to Verify Write
	Value = sdq_read_byte();

	
	//Read Digest
	bq26100_rest();
	Value = TestPresence();
	// Skip ROM Command
	sdq_write_byte(0xCC);
	// Read Digest Command
	sdq_write_byte(0xDD);	//Read Digest Command
	// Write Address
	sdq_write_byte(0x00);	//Address Low Byte
	sdq_write_byte(0x00);	//Address High Byte
	Value = sdq_read_byte();	//Read CRC of Read Digest Command, Address
	// Read Digest
	for (i=0; i<=19; i++)
	{
		Digest[i] = sdq_read_byte();
	}
	// Read CRC of Digest
	Value = sdq_read_byte();

	// The 20 bytes of the digest returned by the bq26100 is arranged in 32-bit words so that it
	// can be compared with the results computed by the microcontroller
	Digest_32[4] = Digest[0x00] + Digest[0x01]*0x0100 + Digest[0x02]*0x010000 + Digest[0x03]*0x01000000;
	Digest_32[3] = Digest[0x04] + Digest[0x05]*0x0100 + Digest[0x06]*0x010000 + Digest[0x07]*0x01000000;
	Digest_32[2] = Digest[0x08] + Digest[0x09]*0x0100 + Digest[0x0A]*0x010000 + Digest[0x0B]*0x01000000;
	Digest_32[1] = Digest[0x0C] + Digest[0x0D]*0x0100 + Digest[0x0E]*0x010000 + Digest[0x0F]*0x01000000;
	Digest_32[0] = Digest[0x10] + Digest[0x11]*0x0100 + Digest[0x12]*0x010000 + Digest[0x13]*0x01000000;

	// The results given by microcontroller and bq26100 are compared
	if (Digest_32[0] == H[0])
	{
		if (Digest_32[1] == H[1])
		{
			if (Digest_32[2] == H[2])
			{
				if (Digest_32[3] == H[3])
				{
					if (Digest_32[4] == H[4])
					{ 	//If all values are same then Authentication is successful
						//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_SET);		//LED Green on
						//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_RESET); 	//LED Red off
					}
					else
					{						//If any of the values do not match then authentication fails
						//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 		//LED Red on
						//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET); 	//LED Green off
					}
				}
				else
				{
					//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 			//LED Red on
					//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);		//LED Green off
				}
			}			
			else
			{
				//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 				//LED Red on
				//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);			//LED Green off	
			}
		}
		else
		{
			//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 					//LED Red on
			//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);				//LED Green off
		}
	}
	else
	{
		//GPIO_WriteBit(GPIO8, GPIO_Pin_6, Bit_SET); 						//LED Red on
		//GPIO_WriteBit(GPIO8, GPIO_Pin_0, Bit_RESET);					//LED Green off
	}  
}













//------------------------------------------------------------------------------


/*******************************************************************************
* 函数名			:	function
* 功能描述		:	初始化Dallas的IO口 DQ 同时检测DS的存在
* 输入			: void
* 返回值			: 返回1:不存在
							返回0:存在
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
sdq_result BQ26100_Init(void)
{
	sdq_result Result	=	sdq_error;
	GPIO_Configuration_OPP50	(SDQ_Port,	SDQ_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605

//	Result	=	Dallas_Rest();

	return Result;
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	从Dallas器件Rom读出ID值
* 输入			: void
* 返回值			: 开始8位是单线产品系列代码，接着48位是唯一序列号，最后8位是前56位的CRC值
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
sdq_result BQ26100_GetID(unsigned char *pBuf)
{
	unsigned char i;
	unsigned char buf[8];

//	Dallas_Rest();						//复位总线

//	Dallas_WriteByte(0x33);		//read romid
//	for(i=0; i<8; i++)
//		buf[i] = Dallas_ReadByte();
//	
//	if(buf[0] != 0x01)
//		return 1;
//	if(CRC8_Calculate(buf, 7) != buf[7])
//		return 1;

//	for(i=0; i<8; i++)
//		pBuf[i] = buf[i];
	
	return sdq_error;
}









































//#ifdef USE_DALLAS
//const uint8_t g_CRC8Tab[256]=
//{
//0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
//0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
//0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
//0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
//0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
//0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
//0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
//0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
//0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
//0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
//0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
//0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
//0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
//0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
//0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
//0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
//};

////查表计算方法
//uint8_t CRC8_Calculate(uint8_t *pBuf, uint8_t len)
//{
//	uint8_t crc8=0;	//CRC8字节初始化
//	
//	//进行CRC8位校验
//	while(len){
//		crc8 = g_CRC8Tab[*pBuf^crc8];
//		pBuf++;
//		len--;
//	}

//	return(crc8);	//返回CRC8校验数据
//}  

////复位Dallas
//void Dallas_Rst(void)
//{                 
//	DALLAS_IO_OUT();	//SET PG11 OUTPUT
//	DALLAS_OUT_LOW();	//拉低DQ
//	Delay_us(750);		//拉低750us
//	DALLAS_OUT_HIGH();	//DQ=1 
//	Delay_us(15);		//15US
//}

////等待Dallas的回应
////返回1:未检测到Dallas1的存在
////返回0:存在
//uint8_t Dallas_Check(void)
//{
//	uint8_t retry=0;
//	
//	DALLAS_IO_IN();		//SET PG11 INPUT	 
//	while(DALLAS_IN_STATE() && (retry < 200)){
//		retry++;
//		Delay_us(1);
//	} 
//	if(retry >= 200)
//		return 1;
//	else
//		retry=0;
//	while((!DALLAS_IN_STATE()) && (retry < 240)){
//		retry++;
//		Delay_us(1);
//	}
//	if(retry >= 240)
//		return 1;
//	
//	return 0;
//}

////从Dallas读取一个位
////返回值：1/0
//uint8_t Dallas_ReadBit(void)
//{
//	uint8_t data;
//	
//	DALLAS_IO_OUT();	//SET PG11 OUTPUT
//	DALLAS_OUT_LOW(); 
//	Delay_us(2);
//	DALLAS_OUT_HIGH(); 
//	DALLAS_IO_IN();		//SET PG11 INPUT
//	Delay_us(12);
//	if(DALLAS_IN_STATE())
//		data = 1;
//	else
//		data = 0;	 
//	Delay_us(50);
//	
//	return data;
//}

////从Dallas读取一个字节
////返回值：读到的数据
//uint8_t Dallas_ReadByte(void)
//{        
//	uint8_t i,j,dat;
//	
//	dat = 0;
//	for (i=1; i<=8; i++){
//		j = Dallas_ReadBit();
//		dat = (j<<7)|(dat>>1);
//	}
//	
//	return dat;
//}

////写一个字节到Dallas
////dat：要写入的字节
//void Dallas_WriteByte(uint8_t dat)
//{             
//	uint8_t j;
//	uint8_t testb;
//	
//	DALLAS_IO_OUT();	//SET PG11 OUTPUT;
//	for (j=1; j<=8; j++){
//		testb = dat&0x01;
//		dat = dat>>1;
//		if (testb){
//			DALLAS_OUT_LOW();	// Write 1
//			Delay_us(2);
//			DALLAS_OUT_HIGH();
//			Delay_us(60);
//		}else{
//			DALLAS_OUT_LOW();	// Write 0
//			Delay_us(60);
//			DALLAS_OUT_HIGH();
//			Delay_us(2);
//		}
//	}
//}

////初始化Dallas的IO口 DQ 同时检测DS的存在
////返回1:不存在
////返回0:存在    	 
//uint8_t Dallas_Init(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	GPIO_SetBits(DALLAS_GPIOx, DALLAS_PIN);
//	GPIO_InitStructure.GPIO_Pin = DALLAS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		  
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(DALLAS_GPIOx, &GPIO_InitStructure);

//	Dallas_Rst();

//	return Dallas_Check();
//}

////从Dallas器件Rom读出ID值
////数据：开始8位是单线产品系列代码，接着48位是唯一序列号，最后8位是前56位的CRC值
//uint8_t Dallas_GetID(uint8_t *pBuf)
//{
//	uint8_t i;
//	uint8_t buf[8];

//	Dallas_Rst();	
//	if(Dallas_Check())
//		return 1;
//	Dallas_WriteByte(0x33);	//read romid
//	for(i=0; i<8; i++)
//		buf[i] = Dallas_ReadByte();
//	
//	if(buf[0] != 0x01)
//		return 1;
//	if(CRC8_Calculate(buf, 7) != buf[7])
//		return 1;

//	for(i=0; i<8; i++)
//		pBuf[i] = buf[i];
//	
//	return 0;
//}
//#endif

