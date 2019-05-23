#ifndef __IIC_H
#define __IIC_H

#include "stdint.h"

#include "stm32f10x_gpio.h"

typedef	enum	_iicAck				//带反馈锁控制标志及吸合时间
{
	iic_ack		=0,
	iic_nack	=1,
}iicAck_def;
//EEPROM类型24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
//typedef	enum	_EEtype				//带反馈锁控制标志及吸合时间
//{
//	AT24C01	=0,		//1K位 	(128 x 8)	/128字节	/页大小：8字节		/页个数：16
//	AT24C02,			//2K位 	(256 x 8)	/256字节	/页大小：8字节		/页个数：32
//	AT24C04,			//4K位 	(512 x 8)	/512字节	/页大小：16字节	/页个数：32
//	AT24C08,			//8K位 	(1024 x 8)/1024字节	/页大小：16字节	/页个数：64
//	AT24C16				//16K位 (2048 x 8)/2048字节	/页大小：16字节	/页个数：128
//}EEtypeDef;

typedef	struct	_iic_port				//带反馈锁控制标志及吸合时间
{
	//-------------------------SDA驱动
	GPIO_TypeDef*	SDA_Port;
	u16	SDA_Pin;
	//-------------------------SCL驱动
	GPIO_TypeDef*	SCL_Port;
	u16	SCL_Pin;
}iic_port_def;
//typedef	struct	_iic_data				//带反馈锁控制标志及吸合时间
//{
//	EEtypeDef	EEtype;			//EEPROM类型24C01、24C02这两个型号是8个字节一个页，而24C04、24C08、24C16是16个字节一页
//	unsigned char  PageSize;		//页大小：一页包含的字节数
//	unsigned short WPageStart;	//开始写入的页地址
//	unsigned short WPageAddr;		//当前写入的页地址
//	unsigned short WDataTotal;	//需要写入的数据总数
//	unsigned short WDataSize;		//剩余待写入的数据个数
//	unsigned char  DeviceAddr;	//设备地址：bit0=0:write;bit0=1:read
//	unsigned char* WDataAddr;		//待写入的数据起始地址
//}iic_data_def;
//typedef struct _slave_iic
//{
//	struct
//	{
//		unsigned char slave	:1;		//0-非从机,1-从机模式
//		unsigned char write	:1;		//0-无标识,1-主机写标识
//		unsigned char read	:1;		//0-无标识,1-主机读标识
//		unsigned char start	:1;		//0-未启动，1-已启动
//		unsigned char stop	:1;		//0-运行状态，1-已停止
//		unsigned char	sdas	:1;		//sda脚之前状态
//		unsigned char scls	:1;		//scl脚之前状态		
//	}status;
//	struct
//	{
//		unsigned char bit0	:1;		//1-已获取
//		unsigned char bit1	:1;		//1-已获取
//		unsigned char bit2	:1;		//1-已获取
//		unsigned char bit3	:1;		//1-已获取
//		unsigned char bit4	:1;		//1-已获取
//		unsigned char bit5	:1;		//1-已获取
//		unsigned char bit6	:1;		//1-已获取
//		unsigned char bit7	:1;		//1-已获取		
//	}byte_data;
//	unsigned char get_bit_count;			//获取多少个位计算
//}slave_iic_def;

typedef	struct	_iic				//带反馈锁控制标志及吸合时间
{
	//-------------------------硬件驱动
	iic_port_def	port;
	//-------------------------参数
//	iic_data_def	DATA;
	//-------------------------模拟iic从机相关标识
//	slave_iic_def	slave;
	
}iic_def;


//-----------------------------------------api
void api_iic_configuration_gpio(iic_def *sI2C);		//启用锁--配置
void api_iic_write_command(iic_def *sI2C,const unsigned short address,unsigned char reg);
unsigned char api_iic_write(iic_def *sIIC,const unsigned short device_address,unsigned char word_address,unsigned char* pBuffer,unsigned char len);
unsigned char api_iic_read(iic_def *sI2C,const unsigned short device_address,unsigned char reg_address,unsigned char* pBuffer,unsigned char len);

unsigned char api_iic_read_start(iic_def *sIIC,const unsigned short device_address);	
unsigned char api_iic_read_buffer(iic_def *sI2C,const unsigned short device_address,unsigned char* pBuffer,unsigned char len);





#endif

