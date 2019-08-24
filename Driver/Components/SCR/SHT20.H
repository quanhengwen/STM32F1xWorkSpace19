#ifndef __SHT20_H
#define __SHT20_H

#include "IIC.H"

#define  SHT2X_ADR_W 0x80  //SHT2X IIC 读地址
#define  SHT2X_ADR_R 0x81  //SHT2X IIC 写地址

typedef enum	_sht20Command
{
	TRIG_TEMP_MEASUREMENT_HM   = 0xE3, // 触发 温度 测量 ，保持主机   命令
	TRIG_HUMI_MEASUREMENT_HM   = 0xE5, // 触发 湿度 测量 ，保持主机   命令
	TRIG_TEMP_MEASUREMENT_POLL = 0xF3, // 触发 温度 测量 ，非保持主机 命令
	TRIG_HUMI_MEASUREMENT_POLL = 0xF5, // 触发 湿度 测量 ，非保持主机 命令
	USER_REG_W                 = 0xE6, // 写寄存器命令
	USER_REG_R                 = 0xE7, // 读寄存器命令
	SOFT_RESET                 = 0xFE  // 软件复位命令
}sht20Command_def;

typedef enum {
    SHT2x_RES_12_14BIT         = 0x00, //RH=12bit, T=14bit 这是默认的值   我们不用修改
    SHT2x_RES_8_12BIT          = 0x01, //RH= 8bit, T=12bit
    SHT2x_RES_10_13BIT         = 0x80, //RH=10bit, T=13bit
    SHT2x_RES_11_11BIT         = 0x81, //RH=11bit, T=11bit
    SHT2x_RES_MASK             = 0x81  //Mask for res. bits (7,0) in user reg.
} SHT2xResolution;

typedef enum {
    SHT2x_HEATER_ON            = 0x04, //heater on
    SHT2x_HEATER_OFF           = 0x00, //heater off
    SHT2x_HEATER_MASK          = 0x04  //Mask for Heater bit(2) in user reg.
} SHT2xHeater;

typedef struct
{
	float temperature;		//温度--已转换
	float humidity;  			//湿度--已转换
	unsigned short SegTemperature;	//温度--未转换
	unsigned short SegHumidity;  	//湿度--未转换
}sht20data_def;

typedef struct	_sht20flag
{
	unsigned char h:1;		//0-无，1-湿度采集标识
	unsigned char t:1;		//0-无，1-温度采集标识
}sht20flag_def;

typedef struct _sht20
{
	iic_def iic;
	sht20data_def	data;
	sht20flag_def	flag;
}sht20def;


void api_sht20_configuration(sht20def *sI2C);
void api_sht20_server(void);

float api_sht20_get_temperature(void);
float api_sht20_get_humidity(void);

unsigned short api_sht20_get_SegTemperature(void);
unsigned short api_sht20_get_SegHumidity(void);




#endif
