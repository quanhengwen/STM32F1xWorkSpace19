#ifndef __I2C_H
#define __I2C_H

#include "stdint.h"

#include "stm32f10x_gpio.h"


//#include "STM32_GPIO.H"

typedef	enum	_I2CACK				//带反馈锁控制标志及吸合时间
{
	iic_ack	=0,
	I2C_NACK		=1
}I2CACKDef;
//EEPROM类型24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
typedef	enum	_EEtype				//带反馈锁控制标志及吸合时间
{
	AT24C01	=0,		//1K位 	(128 x 8)	/128字节	/页大小：8字节		/页个数：16
	AT24C02,			//2K位 	(256 x 8)	/256字节	/页大小：8字节		/页个数：32
	AT24C04,			//4K位 	(512 x 8)	/512字节	/页大小：16字节	/页个数：32
	AT24C08,			//8K位 	(1024 x 8)/1024字节	/页大小：16字节	/页个数：64
	AT24C16				//16K位 (2048 x 8)/2048字节	/页大小：16字节	/页个数：128
}EEtypeDef;

typedef	struct	_iic_port				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDA驱动
	GPIO_TypeDef*	SDA_Port;
	u16	SDA_Pin;
	//-------------------------SCL驱动
	GPIO_TypeDef*	SCL_Port;
	u16	SCL_Pin;
}iic_port_def;
typedef	struct	_iic_data				//带反馈锁控制标志及吸合时间
{
	EEtypeDef	EEtype;			//EEPROM类型24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
	unsigned char  PageSize;		//页大小：一页包含的字节数
	unsigned short WPageStart;	//开始写入的页地址
	unsigned short WPageAddr;		//当前写入的页地址
	unsigned short WDataTotal;	//需要写入的数据总数
	unsigned short WDataSize;		//剩余待写入的数据个数
	unsigned char  DeviceAddr;	//设备地址：bit0=0:write;bit0=1:read
	unsigned char* WDataAddr;		//待写入的数据起始地址
}iic_data_def;
typedef struct _slave_iic
{
	struct
	{
		unsigned char slave	:1;		//0-非从机,1-从机模式
		unsigned char write	:1;		//0-无标识,1-主机写标识
		unsigned char read	:1;		//0-无标识,1-主机读标识
		unsigned char start	:1;		//0-未启动，1-已启动
		unsigned char stop	:1;		//0-运行状态，1-已停止
		unsigned char	sdas	:1;		//sda脚之前状态
		unsigned char scls	:1;		//scl脚之前状态		
	}status;
	struct
	{
		unsigned char bit0	:1;		//1-已获取
		unsigned char bit1	:1;		//1-已获取
		unsigned char bit2	:1;		//1-已获取
		unsigned char bit3	:1;		//1-已获取
		unsigned char bit4	:1;		//1-已获取
		unsigned char bit5	:1;		//1-已获取
		unsigned char bit6	:1;		//1-已获取
		unsigned char bit7	:1;		//1-已获取		
	}byte_data;
	unsigned char get_bit_count;			//获取多少个位计算
}slave_iic_def;

typedef	struct	_iic				//带反馈锁控制标志及吸合时间
{
	//-------------------------硬件驱动
	iic_port_def	port;
	//-------------------------参数
	iic_data_def	DATA;
	//-------------------------模拟iic从机相关标识
	slave_iic_def	slave;
	
}iic_def;



void I2C_Configuration(iic_def *sI2C);		//启用锁--配置
void I2C_Server(iic_def *sI2C);



//-----------------------------------------hw
void iic_set_sda_out(iic_def *sI2C);
void iic_set_sda_in(iic_def *sI2C);
void iic_set_scl_out(iic_def *sI2C);
void iic_set_scl_in(iic_def *sI2C);

void iic_set_sda_high(iic_def *sI2C);
void iic_set_sda_low(iic_def *sI2C);

void iic_set_scl_high(iic_def *sI2C);
void iic_set_scl_low(iic_def *sI2C);


unsigned char iic_get_sda(iic_def *sI2C);
unsigned char iic_get_scl(iic_def *sI2C);



//-----------------------------------------sw--protocol
static void iic_slave_get_line(iic_def *sI2C);
static void iic_slave_get_start(iic_def *sI2C);
static void iic_slave_get_stop(iic_def *sI2C);
static void iic_slave_get_bit(iic_def *sI2C);
unsigned char iic_slave_get_byte(iic_def *sI2C,unsigned char* data);






//-----------------------------------------sw--protocol
void iic_master_set_start(iic_def *sI2C);
void iic_master_set_stop(iic_def *sI2C);

void iic_master_write_byte(iic_def *sI2C,unsigned char ucByte);
unsigned char iic_master_read_byte(iic_def *sI2C);

void iic_master_write_register(iic_def *sI2C,const unsigned short address,unsigned char reg,unsigned char* pBuffer,unsigned char len);
unsigned char iic_master_read_register(iic_def *sI2C,const unsigned short device_address,unsigned char address,unsigned char* pBuffer,unsigned char len);


void I2C_WriteOneByte(iic_def *sI2C,unsigned short address,unsigned char ucByte);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）





unsigned char api_iic_get_ack(iic_def *sI2C);		//CPU产生一个时钟，并且读取器件的ACK应答信号
void api_iic_set_ack(iic_def *sI2C);								//CPU产生一个ACK信号 
void api_iic_set_nack(iic_def *sI2C);								//CPU产生一个NACK信号(NACK即无应答信号) 





//-----------------------------------------sw--data


void I2C_WritePage(iic_def *sI2C,unsigned short PageAddress,const unsigned char* pBuffer);		//页写：
void I2C_WriteNbyte(iic_def *sI2C,unsigned short address,const unsigned char* pBuffer,unsigned short length);
void I2C_WriteBuffer(iic_def *sI2C,unsigned short address,const unsigned char* pBuffer,unsigned short length);	//向I2C总线设备发送8bits的数据 ,首先传输的是数据的最高位（MSB）


unsigned char I2C_ReadOneByte(iic_def *sI2C,unsigned short address);							//从I2C总线读取8个bits的数据  ,首先读出的是数据的最高位（MSB）
unsigned short I2C_ReadPage(iic_def *sI2C,unsigned short PageAddress,unsigned char* pBuffer);
unsigned short I2C_ReadBuffer(iic_def *sI2C,unsigned short address,unsigned char* pBuffer,unsigned short length);



void I2C_DelayNop(unsigned	short Time);
void I2C_Delayus(unsigned	short Time);
void I2C_Delayms(unsigned	short Time);
#endif

