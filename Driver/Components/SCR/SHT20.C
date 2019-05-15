#include "SHT20.H"



sht20def	*pSHT20;
iic_def		*sht20iic;
//unsigned char dSH20[5]={0};

void sht20_set_mode_soft_reset(void);
void sht20_set_mode_slave_temperature(void);
void sht20_set_mode_slave_humidity(void);

void sht20_get_temperature_poll(void);
void sht20_get_humidity_poll(void);
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_sht20_configuration(sht20def *sht20)
{
	pSHT20	=	sht20;
	sht20iic	=	&pSHT20->iic;
	I2C_Configuration(&pSHT20->iic);		//������--����
	
	sht20_set_mode_soft_reset();
	
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_sht20_server(void)
{
	static unsigned short time = 0;
	if(time++<500)
		return;
	time = 0;
	if(pSHT20->flag.t)
		sht20_get_temperature_poll();
	else
		sht20_get_humidity_poll();
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
float api_sht20_get_temperature(void)
{
	return pSHT20->data.temperature;
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
float api_sht20_get_humidity(void)
{
	return pSHT20->data.humidity;
}
//------------------------------------------------------------------------------

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void sht20_set_mode_soft_reset(void)
{
	unsigned char pBuffer[3];
	pBuffer[0] =	0x03;
//	iic_master_write_register(sht20iic,SHT2X_ADR_W,SOFT_RESET,pBuffer,0);
//	iic_master_write_register(sht20iic,SHT2X_ADR_W,USER_REG_W,pBuffer,1);
	sht20_set_mode_slave_temperature();
	return;
	
	start:
	iic_master_set_start(sht20iic);
	//---------------------------д������ַ
	iic_master_write_byte(&pSHT20->iic,SHT2X_ADR_W);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------����ģʽ
	iic_master_write_byte(sht20iic,TRIG_TEMP_MEASUREMENT_POLL);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------ֹͣI2C
	stopI2C:
	iic_master_set_stop(sht20iic);
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void sht20_set_mode_slave_temperature(void)
{
	unsigned char pBuffer[3];
	iic_master_write_register(sht20iic,SHT2X_ADR_W,TRIG_TEMP_MEASUREMENT_POLL,pBuffer,0);
	pSHT20->flag.h	=	0;
	pSHT20->flag.t	=	1;
	return;
	
	start:
	iic_master_set_start(sht20iic);
	//---------------------------д������ַ
	iic_master_write_byte(&pSHT20->iic,SHT2X_ADR_W);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------����ģʽ
	iic_master_write_byte(sht20iic,TRIG_TEMP_MEASUREMENT_POLL);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------ֹͣI2C
	stopI2C:
	iic_master_set_stop(sht20iic);
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void sht20_set_mode_slave_humidity(void)
{
	unsigned char pBuffer[3];
	iic_master_write_register(sht20iic,SHT2X_ADR_W,TRIG_HUMI_MEASUREMENT_POLL,pBuffer,0);
	pSHT20->flag.h	=	1;
	pSHT20->flag.t	=	0;
	return;
	
	start:
	iic_master_set_start(sht20iic);
	//---------------------------д������ַ
	iic_master_write_byte(sht20iic,0x80);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------����ģʽ
	iic_master_write_byte(sht20iic,0xF3);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//�͵�ƽ-Ӧ��
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------ֹͣI2C
	stopI2C:
	iic_master_set_stop(sht20iic);
}
//------------------------------------------------------------------------------


/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void sht20_get_temperature_poll(void)
{
	unsigned char i = 0;
	unsigned short temp	=	0;
	
	//---------------------------������
	sht20read:
	pSHT20->data.temperature	=	0xFFFF;
	iic_master_set_start(sht20iic);	
	iic_master_write_byte(sht20iic,0x81);	//bit0=0:write;bit0=1:read
	if((iic_ack!=api_iic_get_ack(sht20iic))&&(i++<100))			//�͵�ƽ-Ӧ��
	{		
		goto sht20read;
	}
	if(i>=100)
		goto stopI2C;
	iic_set_sda_in(sht20iic);
	temp|=iic_master_read_byte(sht20iic);
	api_iic_set_ack(sht20iic);
	temp<<=8;
	iic_set_sda_in(sht20iic);
	temp|=iic_master_read_byte(sht20iic);
	api_iic_set_ack(sht20iic);
	iic_set_sda_in(sht20iic);
	iic_master_read_byte(sht20iic);
	api_iic_set_nack(sht20iic);
	//---------------------------ת���¶�����
	temp	=	temp&~0x0003;
	pSHT20->data.temperature	=	(temp*175.72)/65536.0 - 46.85;
	//---------------------------ֹͣI2C
	stopI2C:
	iic_master_set_stop(sht20iic);			
	sht20_set_mode_slave_humidity();	//�����´ζ�ת��
	//sht20_set_mode_slave_temperature();	//�����´ζ�ת��
}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void sht20_get_humidity_poll(void)
{
	unsigned char i = 0;
	unsigned short temp	=	0;
	
	//---------------------------������
	sht20read:
	pSHT20->data.humidity	=	0xFFFF;
	iic_master_set_start(sht20iic);	
	iic_master_write_byte(sht20iic,SHT2X_ADR_R);	//bit0=0:write;bit0=1:read
	if((iic_ack!=api_iic_get_ack(sht20iic))&&(i++<100))			//�͵�ƽ-Ӧ��
	{		
		goto sht20read;
	}
	if(i>=100)
		goto stopI2C;
	iic_set_sda_in(sht20iic);
	temp|=iic_master_read_byte(sht20iic);
	api_iic_set_ack(sht20iic);
	temp<<=8;
	iic_set_sda_in(sht20iic);
	temp|=iic_master_read_byte(sht20iic);
	api_iic_set_ack(sht20iic);
	iic_set_sda_in(sht20iic);
	iic_master_read_byte(sht20iic);
	api_iic_set_nack(sht20iic);
	//---------------------------ת���¶�����
	temp	=	temp&~0x000F;
	pSHT20->data.humidity	=	(temp*125.0)/65536.0 - 6;	
	//---------------------------ֹͣI2C
	stopI2C:
	iic_master_set_stop(sht20iic);
		
	sht20_set_mode_slave_temperature();	//�����´ζ�ת��
}

