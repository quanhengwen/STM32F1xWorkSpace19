#ifndef __SHT20_H
#define __SHT20_H

#include "IIC.H"

#define  SHT2X_ADR_W 0x80  //SHT2X IIC ����ַ
#define  SHT2X_ADR_R 0x81  //SHT2X IIC д��ַ

typedef enum	_sht20Command
{
	TRIG_TEMP_MEASUREMENT_HM   = 0xE3, // ���� �¶� ���� ����������   ����
	TRIG_HUMI_MEASUREMENT_HM   = 0xE5, // ���� ʪ�� ���� ����������   ����
	TRIG_TEMP_MEASUREMENT_POLL = 0xF3, // ���� �¶� ���� ���Ǳ������� ����
	TRIG_HUMI_MEASUREMENT_POLL = 0xF5, // ���� ʪ�� ���� ���Ǳ������� ����
	USER_REG_W                 = 0xE6, // д�Ĵ�������
	USER_REG_R                 = 0xE7, // ���Ĵ�������
	SOFT_RESET                 = 0xFE  // ������λ����
}sht20Command_def;

typedef enum {
    SHT2x_RES_12_14BIT         = 0x00, //RH=12bit, T=14bit ����Ĭ�ϵ�ֵ   ���ǲ����޸�
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
    float temperature;	//ʪ��
    float humidity;  		//ʪ��  
}sht20data_def;

typedef struct	_sht20flag
{
	unsigned char h:1;		//0-�ޣ�1-ʪ�Ȳɼ���ʶ
	unsigned char t:1;		//0-�ޣ�1-�¶Ȳɼ���ʶ
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




#endif