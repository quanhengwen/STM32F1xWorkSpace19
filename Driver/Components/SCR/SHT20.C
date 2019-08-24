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
	api_iic_configuration_gpio(&pSHT20->iic);		//启用锁--配置
	
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
	if(time++<100)
		return;
	time = 0;
	if(pSHT20->flag.t)
	{
		sht20_get_temperature_poll();
	}		
	else
	{
		sht20_get_humidity_poll();
	}
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
unsigned short api_sht20_get_SegTemperature(void)
{
	return pSHT20->data.SegTemperature;
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
unsigned short api_sht20_get_SegHumidity(void)
{
	return pSHT20->data.SegHumidity;
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
//	api_iic_write_command(sht20iic,SHT2X_ADR_W,SOFT_RESET,pBuffer,0);
//	api_iic_write_command(sht20iic,SHT2X_ADR_W,USER_REG_W,pBuffer,1);
	sht20_set_mode_slave_temperature();
	return;
}
/*******************************************************************************
*函数名			:	sht20_set_mode_slave_temperature
*功能描述		:	设置为被动读取温度模式
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void sht20_set_mode_slave_temperature(void)
{
	pSHT20->flag.h	=	0;
	pSHT20->flag.t	=	1;
	api_iic_write_command(sht20iic,SHT2X_ADR_W,TRIG_TEMP_MEASUREMENT_POLL);	
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
	pSHT20->flag.h	=	1;
	pSHT20->flag.t	=	0;
	api_iic_write_command(sht20iic,SHT2X_ADR_W,TRIG_HUMI_MEASUREMENT_POLL);	
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
	unsigned short temp	=	0;
	unsigned char dsht[10]={0};	
	iicAck_def	iicAck	=	iic_nack;
	
	pSHT20->data.SegTemperature	=	0;
	pSHT20->data.temperature		=	0;
	
	iicAck	=	 (iicAck_def)api_iic_read_start(sht20iic,0x81);	
	if(iicAck	==	iic_ack)
	{
			
		api_iic_read_buffer(sht20iic,0x81,dsht,3);
	
		//---------------------------转换湿度数据
		temp	=	dsht[0]<<8|dsht[1];
		temp	=	temp&~0x0003;		//低2位为状态位
		pSHT20->data.SegTemperature	=	temp;
		pSHT20->data.temperature	=	(temp*175.72)/65536.0 - 46.85;
	}	
	sht20_set_mode_slave_humidity();			//启动下次读转换
	
//	unsigned char i = 0;
//	unsigned short temp	=	0;
//	unsigned char dsht[10]={0};
//	
//	api_iic_read_buffer(sht20iic,0x81,dsht,3);
//	
//	//---------------------------转换温度数据
//	temp	=	dsht[0]<<8|dsht[1];
//	temp	=	temp&~0x0003;
//	pSHT20->data.temperature	=	(temp*175.72)/65536.0 - 46.85;
//	sht20_set_mode_slave_humidity();			//启动下次读转换
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
	unsigned short temp	=	0;
	unsigned char dsht[10]={0};	
	iicAck_def	iicAck	=	iic_nack;
	
	pSHT20->data.SegHumidity	=	0;
	pSHT20->data.humidity			=	0;
	
	iicAck	=	 (iicAck_def)api_iic_read_start(sht20iic,0x81);	
	if(iicAck	==	iic_ack)
	{			
		api_iic_read_buffer(sht20iic,0x81,dsht,3);
	
		//---------------------------转换湿度数据
		temp	=	dsht[0]<<8|dsht[1];
		temp	=	temp&~0x000F;
		pSHT20->data.SegHumidity	=	temp;
		pSHT20->data.humidity	=	(temp*125.0)/65536.0 - 6;
	}	
	sht20_set_mode_slave_temperature();			//启动下次读转换
}


