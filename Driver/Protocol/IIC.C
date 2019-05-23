#include "IIC.H"	


#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"

unsigned	char i2cdelaytime=1;

//-----------------------------------------hw-static
static void iic_set_sda_out(iic_def *sI2C);
static void iic_set_sda_in(iic_def *sI2C);
//static void iic_set_scl_out(iic_def *sI2C);
//static void iic_set_scl_in(iic_def *sI2C);

static void iic_set_sda_high(iic_def *sI2C);
static void iic_set_sda_low(iic_def *sI2C);

static void iic_set_scl_high(iic_def *sI2C);
static void iic_set_scl_low(iic_def *sI2C);

static unsigned char iic_get_sda(iic_def *sI2C);
//static unsigned char iic_get_scl(iic_def *sI2C);


//-----------------------------------------sw-static2
static unsigned char iic_write_buffer(iic_def *sI2C,unsigned char* pBuffer,unsigned char len);
static unsigned char iic_set_WordAddress(iic_def *sIIC,unsigned char device_address,unsigned char word_address);
static void iic_write_byte(iic_def *sI2C,unsigned char ucByte);
static unsigned char iic_read_byte(iic_def *sIIC);

//-----------------------------------------sw-static1
static void iic_set_start(iic_def *sIIC);
static void iic_set_stop(iic_def *sIIC);
static unsigned char iic_get_ack(iic_def *sIIC);		//CPU产生一个时钟，并且读取器件的ACK应答信号
static void iic_set_ack(iic_def *sIIC);							//CPU产生一个ACK信号
static void iic_set_nack(iic_def *sIIC);						//CPU产生一个NACK信号(NACK即无应答信号) 


static void I2C_DelayNop(unsigned	short Time);
static void I2C_Delayus(unsigned	short Time);
static void I2C_Delayms(unsigned	short Time);

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void api_iic_configuration_gpio(iic_def *sI2C)		//启用锁--配置
{
	//=====================================SDA脚
	GPIO_Configuration_OPP50	(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//=====================================SCL脚
	GPIO_Configuration_OPP50	(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	//=====================================SDA,SCL拉低
	GPIO_ResetBits(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);
	GPIO_ResetBits(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);	
}
//-----------------------------------------------------------------------------


/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
void api_iic_write_command(iic_def *sIIC,const unsigned short device_address,unsigned char address)
{
	iicAck_def	iicAck	=	iic_nack;
	//---------------------------写存储器地址
	iicAck	=	(iicAck_def)iic_set_WordAddress(sIIC,device_address,address);
	//---------------------------停止I2C
	if(iicAck	==	iic_ack)
	iic_set_stop(sIIC);
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
unsigned char api_iic_write(iic_def *sIIC,const unsigned short device_address,unsigned char word_address,unsigned char* pBuffer,unsigned char len)
{
	unsigned char i	=	0;
	iicAck_def	iicAck	=	iic_nack;
	//---------------------------写存储器地址
	iicAck	=	(iicAck_def)iic_set_WordAddress(sIIC,device_address,word_address);
	//---------------------------停止I2C
	if(iicAck	!=	iic_ack)
	{
		iic_set_stop(sIIC);
		return 0;
	}
	i	=	iic_write_buffer(sIIC,pBuffer,len);
	return i;
}
/*******************************************************************************
* 函数名			:	api_iic_read
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char api_iic_read(iic_def *sIIC,const unsigned short device_address,unsigned char word_address,unsigned char* pBuffer,unsigned char len)
{
	unsigned char i	=	0;
	iicAck_def	iicAck	=	iic_nack;
	//1)-----------------------------------第一阶段：以写方式启动设备，检测设备应答
	iicAck	=	 (iicAck_def)iic_set_WordAddress(sIIC,device_address,word_address);
	if(iicAck	!=	iic_ack)
	{		
		i	=	0;
		goto stopI2C;
	}
	//1)-----------------------------------第二阶段：启动读功能
	iicAck	=	 (iicAck_def)api_iic_read_start(sIIC,device_address);
	if(iicAck	!=	iic_ack)
	{		
		i	=	0;
		goto stopI2C;
	}
	//2)-----------------------------------读数据
	i	=	api_iic_read_buffer(sIIC,device_address,pBuffer,len);
	
	stopI2C:
	iic_set_stop(sIIC);
	return i;
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
unsigned char api_iic_read_buffer(iic_def *sIIC,const unsigned short device_address,unsigned char* pBuffer,unsigned char len)
{
	unsigned char i	=	0;
	//2.4)---------------------------读数据
	for(i=0;i<len;i++)
	{
		iic_set_sda_in(sIIC);
		pBuffer[i]=iic_read_byte(sIIC);
		if(i<len-1)
			iic_set_ack(sIIC);
		else
			iic_set_nack(sIIC);
	}
	//---------------------------停止I2C
	stopI2C:
	iic_set_stop(sIIC);
	return i;
}

/*******************************************************************************
* 函数名			:	api_iic_read_start
* 功能描述		:	单独启动读功能
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
unsigned char api_iic_read_start(iic_def *sIIC,const unsigned short device_address)
{
	unsigned char i	=	0;
	iicAck_def	iicAck	=	iic_nack;
	for(i=0;i<100;i++)
	{
		iic_set_start(sIIC);									//启动IIC总线
		iic_write_byte(sIIC,device_address|0x01);	//以读命令方式写器件地址--bit0=0:write;bit0=1:read
		if(iic_ack	==	iic_get_ack(sIIC))
		{	
			iicAck	=	iic_ack;
			break;
		}
	}
	if(i>=100)
	{
		iicAck	=	iic_nack;
		iic_set_stop(sIIC);
	}
	return (unsigned char)iicAck;		//返回启动结果
}

//------------------------------------------------------------------------------







/*******************************************************************************
* 函数名			:	function
* 功能描述		:	向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB） 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static unsigned char iic_write_buffer(iic_def *sIIC,unsigned char* pBuffer,unsigned char len)
{
	unsigned char i	=	0;
	//---------------------------写数据	
	for(i=0;i<len;i++)
	{
		iic_set_sda_out(sIIC);
		iic_write_byte(sIIC,pBuffer[i]);
		if(iic_ack	!=	iic_get_ack(sIIC))
		{		
			goto end;
		}
	}
	//---------------------------停止I2C
	end:
	return i;
}
/*******************************************************************************
*函数名			:	iic_set_WordAddress
*功能描述		:	指定地址启动设备
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char iic_set_WordAddress(iic_def *sIIC,unsigned char device_address,unsigned char word_address)
{
	unsigned char i	=	0;
	iicAck_def	iicAck	=	iic_nack;
	//1)-----------------------------------第一阶段：以写方式启动设备，写入设备地址，检测设备应答，写入寄存器地址，检测应答
	//1.1)---------------------------------启动器件：重试100次
	for(i=0;i<100;i++)
	{
		iic_set_start(sIIC);									//启动IIC总线
		iic_write_byte(sIIC,device_address&0xFE);	//以写命令方式写器件地址--bit0=0:write;bit0=1:read
		if(iic_ack	==	iic_get_ack(sIIC))
		{	
			iicAck	=	iic_ack;
			break;
		}
	}
	if(i>=100)
	{
		iicAck	=	iic_nack;
		goto stopI2C;
	}
	//1.2)---------------------------------写入读取的存储器起始地址
	iic_set_sda_out(sIIC);
	iic_write_byte(sIIC,word_address);
	if(iic_ack	!=	iic_get_ack(sIIC))
	{	
		iicAck	=	iic_nack;		
		goto stopI2C;
	}
	else
	{
		iicAck	=	iic_ack;
		goto end;
	}
	//---------------------------停止I2C
	stopI2C:
	iic_set_stop(sIIC);
	end:
	return (unsigned char)iicAck;
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
static void iic_write_byte(iic_def *sIIC,unsigned char ucByte)
{
	unsigned char i	=	0;
	unsigned char time	=	5;
	for(i=0;i<8;i++)
	{		
		if(ucByte & 0x80)
		{
			iic_set_sda_high(sIIC);
		}
		else
		{
			iic_set_sda_low(sIIC);
		}
		//I2C_DelayNop(2);
		iic_set_scl_high(sIIC);
		I2C_DelayNop(8);
		iic_set_scl_low(sIIC);
		I2C_DelayNop(3);
		ucByte<<=1;		//左移1个bit		
	}
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
static unsigned char iic_read_byte(iic_def *sIIC)
{
	unsigned char ucByte=0;
	unsigned char i	=	0;
	for(i=0;i<8;i++)
	{
		ucByte<<=1;
		I2C_DelayNop(6);
		iic_set_scl_high(sIIC);
		I2C_DelayNop(2);		
		if(iic_get_sda(sIIC))
		{
			ucByte+=1;
		}
		iic_set_scl_low(sIIC);		
	}
	return ucByte;
}
//------------------------------------------------------------------------------

/*******************************************************************************
* 函数名			:	function
* 功能描述		:	开始信号：SCL为高电平时，SDA由高电平向低电平跳变，开始传送数据。
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void iic_set_start(iic_def *sIIC)
{
	//=====================================将SDA设置为输出模式并将SDA设置为高
	iic_set_sda_out(sIIC);
	iic_set_sda_high(sIIC);
	//=====================================SCL设置为高	
	iic_set_scl_high(sIIC);
	//=====================================SDA向低电平跳变
	iic_set_sda_low(sIIC);
	I2C_DelayNop(2);
	//=====================================将SCL设置为低，防止SDA信号线干扰造成数据错误
	iic_set_scl_low(sIIC);
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
static void iic_set_stop(iic_def *sIIC)
{
	iic_set_sda_out(sIIC);
	//iic_set_sda_low(sI2C);
	//=====================================SDA设置为低,SCL设置为高
	iic_set_scl_high(sIIC);
	//=====================================SDA由低电平向高电平跳变
	iic_set_sda_high(sIIC);
	iic_set_scl_low(sIIC);
}
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	CPU产生一个时钟，并且读取器件的ACK应答信号 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static unsigned char iic_get_ack(iic_def *sIIC)
{
	iicAck_def ack=iic_ack;
	unsigned long i	=	0;

	iic_set_sda_in(sIIC);		//设置为上拉输入模式
	I2C_DelayNop(10);
	iic_set_scl_high(sIIC);
	while((iic_get_sda(sIIC))&&(i++<=10))		//应答:SDA=0;无应答:SDA=1;
	if(i>=10-1)		//应答超时
	{
		ack	= iic_nack;
	}
	else
	{
		ack	= iic_ack;
	}
	iic_set_scl_low(sIIC);	
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
static void iic_set_ack(iic_def *sIIC)
{
	iic_set_sda_out(sIIC);
	I2C_DelayNop(1);	
	iic_set_scl_high(sIIC);
	I2C_DelayNop(5);	
	iic_set_scl_low(sIIC);
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
static void iic_set_nack(iic_def *sIIC)
{
	iic_set_sda_out(sIIC);
	
	iic_set_sda_high(sIIC);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_high(sIIC);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_scl_low(sIIC);
	I2C_Delayus(i2cdelaytime);
	
	iic_set_sda_low(sIIC);
}
//------------------------------------------------------------------------------




static void iic_set_sda_out(iic_def *sI2C)
{
	GPIO_RegConfiguration_OPP50	(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
	GPIO_ResetBits(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);
}
static void iic_set_sda_in(iic_def *sI2C)
{
	GPIO_Configuration_IPU(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);			//将GPIO相应管脚配置为上拉输入模式----V20170605
}
//static void iic_set_scl_out(iic_def *sI2C)
//{
//	GPIO_RegConfiguration_OPP50(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);	//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20190104--寄存器版本
//	GPIO_ResetBits(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);
//}
//static void iic_set_scl_in(iic_def *sI2C)
//{
//	GPIO_Configuration_IPU(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);			//将GPIO相应管脚配置为上拉输入模式----V20170605
//	GPIO_ResetBits(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);
//}
static void iic_set_sda_high(iic_def *sI2C)
{
	GPIO_SetBits(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);
}
static void iic_set_sda_low(iic_def *sI2C)
{
	GPIO_ResetBits(sI2C->port.SDA_Port,sI2C->port.SDA_Pin);
}
static void iic_set_scl_high(iic_def *sI2C)
{
	GPIO_SetBits(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);
}
static void iic_set_scl_low(iic_def *sI2C)
{
	GPIO_ResetBits(sI2C->port.SCL_Port,sI2C->port.SCL_Pin);
}
static unsigned char iic_get_sda(iic_def *sI2C)
{
	if(GPIO_ReadInputDataBit(sI2C->port.SDA_Port,sI2C->port.SDA_Pin))
		return 1;
	else
		return 0;
}
//static unsigned char iic_get_scl(iic_def *sI2C)
//{
//	if(GPIO_ReadInputDataBit(sI2C->port.SCL_Port,sI2C->port.SCL_Pin))
//		return 1;
//	else
//		return 0;
//}
//------------------------------------------------------------------------------
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
* 修改时间		: 无
* 修改内容		: 无
* 其它			: wegam@sina.com
*******************************************************************************/
static void I2C_DelayNop(unsigned	short Time)
{
	while(Time--)
	{
		__nop();
	}
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
static void I2C_Delayus(unsigned	short Time)
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
static void I2C_Delayms(unsigned	short Time)
{
	SysTick_DeleymS(Time);				//SysTick延时nmS
}


