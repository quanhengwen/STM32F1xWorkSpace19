#ifndef __SHT20M_H
#define __SHT20M_H

#include "STM32_SPI.H"
typedef enum	_sht20m_cmd
{
	cmd1	=	0x03,		//读保持寄存器
	cmd2	=	0x04,		//读输入寄存器
	cmd3	=	0x06,		//写单个保持寄存器
	cmd4	=	0x10,		//写多个保持寄存器
}sht20m_cmd_def;
typedef struct	_sht20m_pro_write
{
	unsigned char device_address;		//从机地址
	sht20m_cmd_def cmd;							//功能码
	unsigned char	reg_address_h;		//寄存器地址高字节
	unsigned char reg_address_l;		//寄存器地址低字节
	unsigned char reg_data_h;				//寄存器值高字节
	unsigned char reg_data_l;				//寄存器值低字节
}sht20m_pro_write_def;
typedef struct	_sht20m_pro_read
{
	unsigned char device_address;		//从机地址
	sht20m_cmd_def cmd;							//功能码
	unsigned char	len;							//字节数
	unsigned char data[256];				//寄存器地址低字节
}sht20m_pro_read_def;



void api_SHT20M_configuration(void);
	

#endif
