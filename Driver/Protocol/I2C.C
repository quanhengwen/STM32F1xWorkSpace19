#include "I2C.H"	


#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_SYSTICK.H"

//void I2CDelay(unsigned short Time);
//void I2C_SCLHigh(sI2CDef *sI2C);
//void I2C_SCLLow(sI2CDef *sI2C);
//void I2C_SDAHigh(sI2CDef *sI2C);
//void I2C_SDALow(sI2CDef *sI2C);
//void I2C_SDASetOut(sI2CDef *sI2C);
//void I2C_SDASetIn(sI2CDef *sI2C);
unsigned	char i2cdelaytime=1;

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_Configuration(sI2CDef *sI2C)		//启用锁--配置
{
	//=====================================SDA脚
	GPIO_Configuration_OPP50	(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//=====================================SCL脚
	GPIO_Configuration_OPP50	(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//=====================================SDA,SCL拉低
	GPIO_SetBits(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin);
	GPIO_SetBits(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);	
	
	sI2C->DATA.EEtype=AT24C02;		//EEPROM类型24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
	
	//-------------------------------------页大小参数设置
	if((sI2C->DATA.EEtype==AT24C01)||(sI2C->DATA.EEtype==AT24C02))
	{	//24C01、24C02这两个型号是8个字节一个页
		sI2C->DATA.PageSize=8;
	}
	else
	{//24C04、24C08、24C16是16个字节一页
		sI2C->DATA.PageSize=16;
	}
	sI2C->DATA.DeviceAddr	=	0xA0;	//设备地址
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
void I2C_Server(sI2CDef *sI2C)
{
	
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
void iic_set_sda_out(sI2CDef *sI2C)
{
	iic_set_scl_low(sI2C);
	iic_set_sda_low(sI2C);
	GPIO_RegConfiguration_OPP50	(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
	//GPIO_Configuration_OOD50	(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
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
void iic_set_sda_in(sI2CDef *sI2C)
{
	iic_set_scl_low(sI2C);
	iic_set_sda_low(sI2C);
	GPIO_Configuration_INF	(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);			//将GPIO相应管脚配置为上拉输入模式----V20170605
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
void iic_set_scl_out(sI2CDef *sI2C)
{
	iic_set_scl_low(sI2C);
	iic_set_sda_low(sI2C);
	GPIO_RegConfiguration_OPP50	(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
	//GPIO_Configuration_OOD50	(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
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
void iic_set_scl_in(sI2CDef *sI2C)
{
	iic_set_scl_low(sI2C);
	iic_set_sda_low(sI2C);
	GPIO_Configuration_INF	(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin);			//将GPIO相应管脚配置为上拉输入模式----V20170605
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
void iic_set_sda_high(sI2CDef *sI2C)
{
	GPIO_SetBits(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);
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
void iic_set_sda_low(sI2CDef *sI2C)
{
	GPIO_ResetBits(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);
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
void iic_set_scl_high(sI2CDef *sI2C)
{
	GPIO_SetBits(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin);
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
void iic_set_scl_low(sI2CDef *sI2C)
{
	GPIO_ResetBits(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin);
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
unsigned char iic_get_sda(sI2CDef *sI2C)
{
	if(GPIO_ReadInputDataBit(sI2C->HW.SDA_Port,sI2C->HW.SDA_Pin))
		return 1;
	else
		return 0;
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
unsigned char iic_get_scl(sI2CDef *sI2C)
{
	if(GPIO_ReadInputDataBit(sI2C->HW.SCL_Port,sI2C->HW.SCL_Pin))
		return 1;
	else
		return 0;
}
//-----------------------------------------------------------------------------







/*******************************************************************************
* 函数名			:	function
* 功能描述		:	CPU产生一个时钟，并且读取器件的ACK应答信号 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char I2C_WaitAck(sI2CDef *sI2C)
{
	I2CACKDef ack=I2C_ACK;
	unsigned long i	=	0;
	iic_set_scl_low(sI2C);
	iic_set_sda_in(sI2C);	//设置为上拉输入模式
	iic_set_scl_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	while((iic_get_sda(sI2C))&&(i++<=40000))		//应答:SDA=0;无应答:SDA=1;
	if(i>=40000-1)		//应答超时
	{
		ack	= I2C_NACK;
	}
	else
	{
		ack	= I2C_ACK;
	}
	iic_set_scl_low(sI2C);
	
	iic_set_sda_out(sI2C);
	
	return (unsigned char)ack;
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	CPU产生一个ACK信号 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_Ack(sI2CDef *sI2C)
{
	iic_set_sda_out(sI2C);
	
	iic_set_sda_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	iic_set_sda_high(sI2C);
	I2C_Delayus(i2cdelaytime);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	CPU产生一个NACK信号(NACK即无应答信号) 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_NAck(sI2CDef *sI2C)
{
	iic_set_sda_out(sI2C);
	
	iic_set_sda_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_sda_low(sI2C);
}

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	开始信号：SCL为高电平时，SDA由高电平向低电平跳变，开始传送数据。
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_Start(sI2CDef *sI2C)
{
	iic_set_scl_low(sI2C);
	iic_set_sda_out(sI2C);
	//=====================================SDA,SCL设置为高
	iic_set_sda_high(sI2C);
	iic_set_scl_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	//=====================================SDA向低电平跳变
	iic_set_sda_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	iic_set_scl_low(sI2C);
	I2C_Delayus(i2cdelaytime);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	结束信号：SCL为高电平时，SDA由低电平向高电平跳变，结束传送数据。
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_Stop(sI2CDef *sI2C)
{
	iic_set_sda_out(sI2C);
	//=====================================SDA设置为低,SCL设置为高
	iic_set_sda_low(sI2C);
	iic_set_scl_high(sI2C);
	I2C_Delayus(i2cdelaytime);
	//=====================================SDA由低电平向高电平跳变
	iic_set_sda_high(sI2C);
	I2C_Delayus(i2cdelaytime);
//	iic_set_scl_low(sI2C);
//	I2C_Delay();
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_SendByte(sI2CDef *sI2C,unsigned char ucByte)
{
	unsigned char i	=	0;
	iic_set_scl_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	for(i=0;i<8;i++)
	{
		if(ucByte & 0x80)
		{
			iic_set_sda_high(sI2C);
		}
		else
		{
			iic_set_sda_low(sI2C);
		}
		I2C_Delayus(i2cdelaytime);
		iic_set_scl_high(sI2C);
		I2C_Delayus(i2cdelaytime);
		iic_set_scl_low(sI2C);
		
		ucByte<<=1;		//左移1个bit
		//I2C_Delayus(i2cdelaytime);
	}
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_WriteOneByte(sI2CDef *sI2C,const unsigned short address,unsigned char ucByte)
{
	unsigned char i	=	0;
	unsigned char temp	=	0xA0;		//bit0=0:write;bit0=1:read
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,temp);

	if(!I2C_WaitAck(sI2C))
	{		
		goto stopI2C;
	}
	//---------------------------写内存地址
	temp	=	address;
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,temp);
	
	if(!I2C_WaitAck(sI2C))
	{
		goto stopI2C;
	}
	//---------------------------写数据
	temp	=	ucByte;
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,temp);
	
	if(!I2C_WaitAck(sI2C))
	{
		goto stopI2C;
	}
	//---------------------------停止I2C
	stopI2C:
	I2C_Stop(sI2C);
}
/*******************************************************************************
* 函数名			:	I2C_WritePage
* 功能描述		:	页写 
							24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_WritePage(sI2CDef *sI2C,unsigned short PageAddress,const unsigned char* pBuffer)
{
	
	unsigned char		temp			=	0xA0;	//bit0=0:write;bit0=1:read
	//unsigned char	 	PageSize	=	0;
	unsigned short 	i					=	0;
	//---------------------------检查数据

	//---------------------------检查页地址
	if(0==sI2C->DATA.PageSize)
	{
		return;
	}
	if(0==PageAddress%sI2C->DATA.PageSize)
	{
		sI2C->DATA.WPageAddr+=sI2C->DATA.PageSize;
	}
	else
	{
		return;
	}
	
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		goto stopI2C;
	}
	//---------------------------写内存地址
	temp	=	PageAddress;
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,temp);
	
	if(!I2C_WaitAck(sI2C))
	{
		goto stopI2C;
	}
	//---------------------------写数据
	iic_set_sda_out(sI2C);
	for(i=0;i<sI2C->DATA.PageSize;i++)
	{
		temp=pBuffer[i];
		I2C_SendByte(sI2C,temp);
		if(!I2C_WaitAck(sI2C))
		{
			goto stopI2C;
		}
	}
	//---------------------------停止I2C
	stopI2C:
	I2C_Stop(sI2C);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_WriteNbyte(sI2CDef *sI2C,unsigned short address,const unsigned char* pBuffer,unsigned short length)
{
	unsigned short i						=	0;
//	unsigned char	PageSize			=	8;	//24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
	//unsigned char	WriteLen			=	0;
	//unsigned short WWritelen		=	0;	//剩余待写入长度
	//unsigned char	StartAddress	=	0;
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,sI2C->DATA.DeviceAddr);	//bit0=0:write;bit0=1:read
	if(!I2C_WaitAck(sI2C))
	{		
		goto stopI2C;
	}
	//---------------------------写内存地址
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,address);
	
	if(!I2C_WaitAck(sI2C))
	{
		goto stopI2C;
	}
	//---------------------------写数据
	iic_set_sda_out(sI2C);
	for(i=0;i<length;i++)
	{
		I2C_SendByte(sI2C,pBuffer[i]);
		if(!I2C_WaitAck(sI2C))
		{
			goto stopI2C;
		}
	}
	//---------------------------
	//---------------------------停止I2C
	stopI2C:
	I2C_Stop(sI2C);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void I2C_WriteBuffer(sI2CDef *sI2C,unsigned short address,const unsigned char* pBuffer,unsigned short length)
{
	unsigned short i						=	0;
//	unsigned char	PageSize			=	8;	//24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
	unsigned char	WriteLen			=	0;
	unsigned short WWritelen		=	0;	//剩余待写入长度
	unsigned char	StartAddress	=	0;
	unsigned char* DataAddr;
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,sI2C->DATA.DeviceAddr);	//bit0=0:write;bit0=1:read
	if(!I2C_WaitAck(sI2C))
	{		
		goto stopI2C;
	}
	StartAddress=address;			//
	WWritelen=length;
	DataAddr=(unsigned char*)&pBuffer[0];
	
	StartWrite:
	//---------------------------判断数据地址是否为整页起始地址

	if(0!=StartAddress%sI2C->DATA.PageSize)	//起始地址非页起始地址：先将不完整页写完
	{
		WriteLen=(address/sI2C->DATA.PageSize+1)*sI2C->DATA.PageSize-StartAddress;	//不完整页的起始待写入数据个数
		
		I2C_WriteNbyte(sI2C,StartAddress,DataAddr,WriteLen);		
	}
	else	//起始地址为页地址
	{
		//-----------------------是否为整页大小
		if(WWritelen>=sI2C->DATA.PageSize)
		{			
			WriteLen=sI2C->DATA.PageSize;
			
			I2C_WritePage(sI2C,StartAddress,DataAddr);
		}
		else
		{
			WriteLen=WWritelen;	//不完整页的起始待写入数据个数
			
			I2C_WriteNbyte(sI2C,StartAddress,DataAddr,WriteLen);
		}
	}
	//-------------------------------剩余待写入长度
	WWritelen=WWritelen-WriteLen;		//剩余待写入长度
	if(WWritelen>0)
	{
		StartAddress=StartAddress+WriteLen;		//下一个待写入数据的起始地址
		DataAddr=&DataAddr[WriteLen];	//新地址
		I2C_Delayms(5);		//等待5ms完成写入EEPROM内部存储
		do		//检查器件是否完成写入
		{
			I2C_Start(sI2C);
			//---------------------------写器件地址
			I2C_SendByte(sI2C,sI2C->DATA.DeviceAddr);	//bit0=0:write;bit0=1:read
		}
		while(I2C_NACK==I2C_WaitAck(sI2C));
		goto StartWrite;
	}
	//---------------------------停止I2C
	stopI2C:
	I2C_Stop(sI2C);
	I2C_Delayms(5);		//等待5ms完成写入EEPROM内部存储
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	从I2C总线读取8个bits的数据  ,首先读出的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char I2C_ReadByte(sI2CDef *sI2C)
{
	unsigned char ucByte;
	unsigned char i	=	0;
	for(i=0;i<8;i++)
	{
		ucByte<<=1;	
		iic_set_scl_low(sI2C);
		I2C_Delayus(i2cdelaytime);
		
		iic_set_scl_high(sI2C);
		I2C_Delayus(i2cdelaytime);
		
		if(iic_get_sda(sI2C))
		{
			ucByte+=1;
		}		
	}
	iic_set_scl_low(sI2C);
	I2C_Delayus(i2cdelaytime);
	
	return ucByte;
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	从I2C总线读取8个bits的数据  ,首先读出的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char I2C_ReadOneByte(sI2CDef *sI2C,unsigned short address)
{
	unsigned char ucByte;
	unsigned char i	=	0;
	unsigned char temp	=	0xA1;		//bit0=0:write;bit0=1:read
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------写内存地址
	temp	=	address;
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------启动读
	temp	=	0xA1;
	I2C_Start(sI2C);
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------读数据
	iic_set_sda_in(sI2C);	//设置为上拉输入模式
	ucByte	=	 I2C_ReadByte(sI2C);
	
	I2C_NAck(sI2C);								//CPU产生一个NACK信号(NACK即无应答信号)
	I2C_Stop(sI2C);
	
	return ucByte;
}

/*******************************************************************************
* 函数名			:	I2C_ReadPage
* 功能描述		:	按页读取  ,首先读出的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned short I2C_ReadPage(sI2CDef *sI2C,unsigned short PageAddress,unsigned char* pBuffer)
{
	unsigned char ucByte;	
	unsigned char temp	=	0xA0;		//bit0=0:write;bit0=1:read
	unsigned short i	=	0;
	//---------------------------检查页地址
	if(0==sI2C->DATA.PageSize)
	{
		return 0;
	}
	if(0==(PageAddress%sI2C->DATA.PageSize))
	{
		sI2C->DATA.WPageAddr+=sI2C->DATA.PageSize;
	}
	else
	{
		return 0;
	}
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------写内存地址
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,PageAddress);
	if(!I2C_WaitAck(sI2C))
	{
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------启动读
	temp	=	0xA1;				//bit0=0:write;bit0=1:read
	I2C_Start(sI2C);
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------读数据
	for(i=0;i<sI2C->DATA.PageSize;i++)
	{
		iic_set_sda_in(sI2C);	//设置为上拉输入模式
		pBuffer[i]=I2C_ReadByte(sI2C);
		if(i>=sI2C->DATA.PageSize-1)
		{
			I2C_NAck(sI2C);								//CPU产生一个NACK信号(NACK即无应答信号)
		}
		else
		{
			I2C_Ack(sI2C);								//CPU产生一个ACK信号
		}
	}	
	I2C_Stop(sI2C);

	return ucByte;
}
/*******************************************************************************
* 函数名			:	I2C_ReadBuffer
* 功能描述		:	从I2C总线连续读取n个数据  ,首先读出的是数据的最高位（MSB），读数据可以自动分页
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned short I2C_ReadBuffer(sI2CDef *sI2C,unsigned short address,unsigned char* pBuffer,unsigned short length)
{
	unsigned char ucByte;
	unsigned short i	=	0;
	unsigned char temp	=	0xA0;		//bit0=0:write;bit0=1:read
	//---------------------------启动I2C
	I2C_Start(sI2C);
	//---------------------------写器件地址
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------写内存地址
	temp	=	address;
	
	iic_set_sda_out(sI2C);
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------启动读
	temp	=	0xA1;				//bit0=0:write;bit0=1:read
	I2C_Start(sI2C);
	I2C_SendByte(sI2C,temp);
	if(!I2C_WaitAck(sI2C))
	{		
		I2C_Stop(sI2C);
		return 0;
	}
	//---------------------------读数据
	for(i=0;i<length;i++)
	{
		iic_set_sda_in(sI2C);	//设置为上拉输入模式
		pBuffer[i]=I2C_ReadByte(sI2C);
		if(i>=length-1)
		{
			I2C_NAck(sI2C);								//CPU产生一个NACK信号(NACK即无应答信号)
		}
		else
		{
			I2C_Ack(sI2C);								//CPU产生一个ACK信号
		}
	}	
	I2C_Stop(sI2C);

	return ucByte;
}
//------------------------------------------------------------------------------



//-----------------------------SLAVE
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iic_slave_get_line(sI2CDef *sI2C)
{
	unsigned char sdas	=	iic_get_sda(sI2C);			//获取SDA电平
	unsigned char scls	=	iic_get_scl(sI2C);			//获取SCL电平
	if((sdas	==	sI2C->slave.status.sdas)
		&&(scls	==	sI2C->slave.status.scls))
	return;		//SDA和SCL无变化
	
	//------------------------------检查是否为启动条件
	if(0==sI2C->slave.status.start)	//0-未启动，1-已启动
	{
		iic_slave_get_start(sI2C);
		return;
	}
	//------------------------------检查是否为停止条件
	iic_slave_get_stop(sI2C);
	if(1==sI2C->slave.status.stop)	//0-运行状态，1-已停止
		return;
	
	
	sI2C->slave.status.sdas	=	sdas;			//更新SDA状态
	sI2C->slave.status.scls	=	scls;			//更新SCL状态
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	IIC启动,在SCL线是高电平时SDA线从高电平向低电平切换这个情况表示起始条件
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iic_slave_get_start(sI2CDef *sI2C)
{
	unsigned char sdas	=	0;
	unsigned char scls	=	0;
	if(1==sI2C->slave.status.start)	//已启动
	{
		return;
	}
	sdas	=	iic_get_sda(sI2C);			//获取SDA电平
	scls	=	iic_get_scl(sI2C);			//获取SCL电平	
	if(0==sdas)
	{
		if((1==sI2C->slave.status.sdas)		//之前SDA为高电平,表示由高电平向低电平转换
			&&(1==scls))				//SCL为高电平
		{
			sI2C->slave.status.start	=	1;	//0-未启动，1-已启动
			sI2C->slave.status.stop		=	0;	//0-运行状态，1-已停止		
		}		
	}
	sI2C->slave.status.sdas	=	sdas;			//更新SDA状态
	sI2C->slave.status.scls	=	scls;			//更新SCL状态
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	IIC停止,当SCL是高电平时SDA线由低电平向高电平切换表示停止条件
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iic_slave_get_stop(sI2CDef *sI2C)
{
	unsigned char sdas	=	0;
	unsigned char scls	=	0;
	if(1==sI2C->slave.status.stop)	//已停止
	{
		return;
	}
	sdas	=	iic_get_sda(sI2C);			//获取SDA电平
	scls	=	iic_get_scl(sI2C);			//获取SCL电平	
	if(0!=sdas)		//高电平
	{
		if((0==sI2C->slave.status.sdas)		//之前SDA为低电平,表示由低电平向高电平转换
			&&(1==scls))				//SCL为高电平
		{
			sI2C->slave.status.start	=	0;	//0-未启动，1-已启动
			sI2C->slave.status.stop		=	1;	//0-运行状态，1-已停止		
		}
	}
	sI2C->slave.status.sdas	=	sdas;			//更新SDA状态
	sI2C->slave.status.scls	=	scls;			//更新SCL状态
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	先传输高位,上升沿
							在SCL呈现高电平期间，SDA上的电平必须保持稳定，低电平为数据0，高电平为数据1。
							只有在SCL为低电平期间，才允许SDA上的电平改变状态。逻辑0的电平为低电压，
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void iic_slave_get_bit(sI2CDef *sI2C)
{
	unsigned char bit	=	0;
	unsigned char sdas	=	iic_get_sda(sI2C);			//获取SDA电平
	unsigned char scls	=	iic_get_scl(sI2C);			//获取SCL电平
	//------------------------上升沿
	if(0==scls)		//SCL为低电平，数据不获取
		return;
	if(1	==	sI2C->slave.status.scls)						//SCL原电平为高，不符合上升沿
	{
		return;
	}
	//------------------------符合上升沿条件：先传高位
	if(0==sI2C->slave.get_bit_count)	//获取第1位
	{
		sI2C->slave.byte_data.bit7	=	sdas;
		sI2C->slave.get_bit_count		=	1;		//获取到第1位
	}
	else if(1==sI2C->slave.get_bit_count)	//获取第2位
	{
		sI2C->slave.byte_data.bit6	=	sdas;
		sI2C->slave.get_bit_count		=	2;		//获取到第2位
	}
	else if(2==sI2C->slave.get_bit_count)	//获取第3位
	{
		sI2C->slave.byte_data.bit5	=	sdas;
		sI2C->slave.get_bit_count		=	3;		//获取到第3位
	}
	else if(3==sI2C->slave.get_bit_count)	//获取第4位
	{
		sI2C->slave.byte_data.bit4	=	sdas;
		sI2C->slave.get_bit_count		=	4;		//获取到第4位
	}
	else if(4==sI2C->slave.get_bit_count)	//获取第5位
	{
		sI2C->slave.byte_data.bit3	=	sdas;
		sI2C->slave.get_bit_count		=	5;		//获取到第5位
	}
	else if(5==sI2C->slave.get_bit_count)	//获取第6位
	{
		sI2C->slave.byte_data.bit2	=	sdas;
		sI2C->slave.get_bit_count		=	6;		//获取到第6位
	}
	else if(6==sI2C->slave.get_bit_count)	//获取第7位
	{
		sI2C->slave.byte_data.bit1	=	sdas;
		sI2C->slave.get_bit_count		=	7;		//获取到第7位
	}
	else if(7==sI2C->slave.get_bit_count)	//获取第8位
	{
		sI2C->slave.byte_data.bit0	=	sdas;
		sI2C->slave.get_bit_count		=	8;		//获取到第8位
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
unsigned char iic_slave_get_byte(sI2CDef *sI2C,unsigned char* data)
{
	unsigned char i			=	0;
	unsigned char temp	=	0;
	for(i=0;i<8;i++)
	{
		iic_get_scl(sI2C);
		
	}	
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
static unsigned char iic_slave_read_buffer(sI2CDef *sI2C,unsigned char* pBuffer,unsigned char len)
{
	
	unsigned char i			=	0;
	unsigned char data	=	0;
	
	static unsigned char start = 0;
	static unsigned char j	=	0;
	
	iic_set_scl_in(sI2C);
	iic_set_sda_in(sI2C);
	//----------------------------------------
	if(0==start)
	{
//		if(0==iic_slave_get_start(sI2C))
//		{
//			return 0;
//		}
//		else
//		{
//			start	=	1;
//		}
	}
	//----------------------------------------
	if(iic_slave_read_byte(sI2C,&data))
	{
		pBuffer[j]=data;
		j+=1;
		if(j>=len)
		{
			j=0;
			start	=	0;
			return len;
		}
	}	
	return 0;
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
void I2C_Delayus(unsigned	short Time)
{
	SysTick_DeleyuS(Time);				//SysTick延时nmS
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
void I2C_Delayms(unsigned	short Time)
{
	SysTick_DeleymS(Time);				//SysTick延时nmS
}