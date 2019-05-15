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
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_sht20_configuration(sht20def *sht20)
{
	pSHT20	=	sht20;
	sht20iic	=	&pSHT20->iic;
	I2C_Configuration(&pSHT20->iic);		//启用锁--配置
	
	sht20_set_mode_soft_reset();
	
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
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
float api_sht20_get_temperature(void)
{
	return pSHT20->data.temperature;
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
float api_sht20_get_humidity(void)
{
	return pSHT20->data.humidity;
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
	//---------------------------写器件地址
	iic_master_write_byte(&pSHT20->iic,SHT2X_ADR_W);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------设置模式
	iic_master_write_byte(sht20iic,TRIG_TEMP_MEASUREMENT_POLL);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------停止I2C
	stopI2C:
	iic_master_set_stop(sht20iic);
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
void sht20_set_mode_slave_temperature(void)
{
	unsigned char pBuffer[3];
	iic_master_write_register(sht20iic,SHT2X_ADR_W,TRIG_TEMP_MEASUREMENT_POLL,pBuffer,0);
	pSHT20->flag.h	=	0;
	pSHT20->flag.t	=	1;
	return;
	
	start:
	iic_master_set_start(sht20iic);
	//---------------------------写器件地址
	iic_master_write_byte(&pSHT20->iic,SHT2X_ADR_W);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------设置模式
	iic_master_write_byte(sht20iic,TRIG_TEMP_MEASUREMENT_POLL);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------停止I2C
	stopI2C:
	iic_master_set_stop(sht20iic);
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
void sht20_set_mode_slave_humidity(void)
{
	unsigned char pBuffer[3];
	iic_master_write_register(sht20iic,SHT2X_ADR_W,TRIG_HUMI_MEASUREMENT_POLL,pBuffer,0);
	pSHT20->flag.h	=	1;
	pSHT20->flag.t	=	0;
	return;
	
	start:
	iic_master_set_start(sht20iic);
	//---------------------------写器件地址
	iic_master_write_byte(sht20iic,0x80);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------设置模式
	iic_master_write_byte(sht20iic,0xF3);	//bit0=0:write;bit0=1:read
	if(iic_ack	!=	api_iic_get_ack(sht20iic))		//低电平-应答
	{		
		iic_master_set_stop(sht20iic);		
		goto start;
	}
	//---------------------------停止I2C
	stopI2C:
	iic_master_set_stop(sht20iic);
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
void sht20_get_temperature_poll(void)
{
	unsigned char i = 0;
	unsigned short temp	=	0;
	
	//---------------------------读数据
	sht20read:
	pSHT20->data.temperature	=	0xFFFF;
	iic_master_set_start(sht20iic);	
	iic_master_write_byte(sht20iic,0x81);	//bit0=0:write;bit0=1:read
	if((iic_ack!=api_iic_get_ack(sht20iic))&&(i++<100))			//低电平-应答
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
	//---------------------------转换温度数据
	temp	=	temp&~0x0003;
	pSHT20->data.temperature	=	(temp*175.72)/65536.0 - 46.85;
	//---------------------------停止I2C
	stopI2C:
	iic_master_set_stop(sht20iic);			
	sht20_set_mode_slave_humidity();	//启动下次读转换
	//sht20_set_mode_slave_temperature();	//启动下次读转换
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
void sht20_get_humidity_poll(void)
{
	unsigned char i = 0;
	unsigned short temp	=	0;
	
	//---------------------------读数据
	sht20read:
	pSHT20->data.humidity	=	0xFFFF;
	iic_master_set_start(sht20iic);	
	iic_master_write_byte(sht20iic,SHT2X_ADR_R);	//bit0=0:write;bit0=1:read
	if((iic_ack!=api_iic_get_ack(sht20iic))&&(i++<100))			//低电平-应答
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
	//---------------------------转换温度数据
	temp	=	temp&~0x000F;
	pSHT20->data.humidity	=	(temp*125.0)/65536.0 - 6;	
	//---------------------------停止I2C
	stopI2C:
	iic_master_set_stop(sht20iic);
		
	sht20_set_mode_slave_temperature();	//启动下次读转换
}


