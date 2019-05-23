#include "AT24C02.H"	


	
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"


#define	Write_24C02Addr	(unsigned char)0xA0
#define	Read_24C02Addr	(unsigned char)0xA1


//AT24C02 2048位，256字节，8字节/页,共32页

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	在at24c02中的指定地址写入数据
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void AT24C02_Write(iic_def *sI2C,unsigned char Addr,unsigned char Data)
{
//	iic_master_start(sI2C);
	
//	api_iic_master_write_byte(sI2C,Write_24C02Addr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	api_iic_get_ack(sI2C);
//	
//	api_iic_master_write_byte(sI2C,Addr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	api_iic_get_ack(sI2C);
//	
//	api_iic_master_write_byte(sI2C,Data);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	api_iic_get_ack(sI2C);
//	
//	api_iic_master_set_stop(sI2C);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	在at24c02的指定地址中读出写入的数据
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char AT24C02_Read(iic_def *sI2C,unsigned char Addr)
{
	unsigned char Data	=	0;
	
//	iic_master_start(sI2C);
	
//	api_iic_master_write_byte(sI2C,Write_24C02Addr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	api_iic_get_ack(sI2C);
//	
//	api_iic_master_write_byte(sI2C,Addr);						//发送待读取地址
//	
//	api_iic_get_ack(sI2C);
//	
//	iic_master_start(sI2C);
//	
//	api_iic_master_write_byte(sI2C,Read_24C02Addr);	//发送待读取地址
//	
//	api_iic_get_ack(sI2C);
//	
//	
//	
//	iic_set_sda_in(sI2C);
//	
//	Data	=	api_iic_master_read_byte(sI2C);					//从I2C总线读取8个bits的数据  ,首先读出的是数据的最高位（MSB）
//	
//	api_iic_set_nack(sI2C);								//CPU产生一个NACK信号(NACK即无应答信号)
//	
//	iic_master_set_stop(sI2C);
	
	return Data;
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
void AT24C02_WritePage(iic_def *sI2C,unsigned char StartAddr,unsigned char *Buffer)
{
	unsigned char i	=	0;

	
//	iic_master_start(sI2C);
	
//	api_iic_master_write_byte(sI2C,Write_24C02Addr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return;
//	
//	api_iic_master_write_byte(sI2C,StartAddr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return;
//	
//	for(i=0;i<8;i++)
//	{
//		api_iic_master_write_byte(sI2C,Buffer[i]);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//		if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return;
//	}
//	
//	iic_master_set_stop(sI2C);
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
unsigned char AT24C02_ReadBuffer(iic_def *sI2C,unsigned char Addr,unsigned char *Buffer,unsigned char Length)
{
	unsigned char i	=	0;
		
//	iic_master_start(sI2C);
	
//	api_iic_master_write_byte(sI2C,Write_24C02Addr);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）
//	
//	if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return 0;
//	
//	api_iic_master_write_byte(sI2C,Addr);						//发送待读取地址
//	
//	if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return 0;
//	
////	iic_master_start(sI2C);
//	
//	api_iic_master_write_byte(sI2C,Read_24C02Addr);	//发送待读取地址
//	
//	if(api_iic_get_ack(sI2C)	==	iic_nack)
//			return 0;	
//	
//	for(i=0;i<Length;i++)
//	{
//		iic_set_sda_in(sI2C);
//		
//		Buffer[i]	=	api_iic_master_read_byte(sI2C);					//从I2C总线读取8个bits的数据  ,首先读出的是数据的最高位（MSB）
//	
//		iic_set_sda_out(sI2C);
//		
//		api_iic_set_ack(sI2C);											//CPU产生一个ACK信号
//	}
//	iic_set_sda_out(sI2C);
//	
//	api_iic_set_nack(sI2C);											//CPU产生一个NACK信号(NACK即无应答信号)
	
//	iic_master_set_stop(sI2C);

	return 0;
}




