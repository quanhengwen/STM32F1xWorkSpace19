/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include "usb_lib.h"
//#include "usb_desc.h"
//#include "usb_mem.h"

//#include "usb_istr.h"
//#include "usb_pwr.h"
//#include "hw_config.h"

//#include "usb_endp.h"
#include "usb_data.h"
#include	"string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define USB_BUFFER_ARRY_SIZE								2		//缓存数据个数

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//unsigned char usb_buffer_out[USB_DATA_SIZE];
//unsigned char usb_buffer_in[USB_DATA_SIZE];

//u32 count_out = 0;
//u32 count_in = 0;



usb_data_arry usb_out_arry;	//USB主机输出数据到设备缓存
usb_data_arry usb_in_arry;	//设备发送数据到USB主机缓存

unsigned char usb_out_buffer_busy	=	0;		//0-空闲
unsigned char usb_in_buffer_busy	=	0;		//0-空闲
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//static unsigned short usb_endp_set_out_data(unsigned char*	buffer,unsigned short len);
//static unsigned char	usb_endp_get_in_data(unsigned char* rxbuffer);
//static unsigned char	get_buffer_size(usb_data_arry* arry);
//static unsigned short usb_data_set_data(usb_data_arry* arry,const unsigned char*	buffer,unsigned short len);
//static unsigned char	usb_data_get_data(usb_data_arry* arry,unsigned char* rxbuffer);

//------------------------------------------------------------------------------





//-----------------------------------------------------------------------------

/*******************************************************************************
*函数名			:	api_usb_in_add_data
*功能描述		:	添加数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_in_add_data(const unsigned char*	buffer,unsigned short len)
{
	if((usb_data_read==usb_in_arry.status)||(usb_data_full==usb_in_arry.status))	//缓存未准备好
	{
		if(usb_in_arry.len)	//有数据
			return 0;
	}
	if(usb_in_arry.len+len<=USB_BUFFER_SIZE)		//未超出缓存
	{
		memcpy(&usb_in_arry.buffer[usb_in_arry.len],buffer,len);
		usb_in_arry.len+=len;
		usb_in_arry.status	=	usb_data_write;	//正在写数据
	}
	else if(USB_BUFFER_SIZE>len)
	{
		len=USB_BUFFER_SIZE-usb_in_arry.len;
		memcpy(&usb_in_arry.buffer[usb_in_arry.len],buffer,len);
		usb_in_arry.len+=len;
		usb_in_arry.status	=	usb_data_full;	//缓存满
	}
	else
	{
		usb_in_arry.status	=	usb_data_full;	//缓存满
	}
	
	return usb_in_arry.len;
}
/*******************************************************************************
*函数名			:	api_usb_in_set_data
*功能描述		:	设置输入/上传数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_usb_in_set_complete_end(void)
{
	usb_in_arry.status	=	usb_data_read_enable;	//可读
}
/*******************************************************************************
*函数名			:	api_usb_in_set_data
*功能描述		:	设置输入/上传数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_in_set_data(const unsigned char*	buffer,unsigned short len)
{
	len=api_usb_in_add_data(buffer,len);
	return len;
}
/*******************************************************************************
*函数名			:	usb_endp_get_in_data
*功能描述		:	获取待上传到USB主机的数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_in_get_data(unsigned char* rxbuffer)
{
	//VIRTUAL_COM_PORT_DATA_SIZE
	unsigned short len	=	0;
	static unsigned short startaddr=0;
	if((usb_data_read_enable!=usb_in_arry.status)			//非可读状态
		&&(usb_data_read!=usb_in_arry.status)						//正在读数据
		&&(usb_data_full!=usb_in_arry.status))					//缓存未满
		return 0;
	
	if(usb_in_arry.len>USB_COM_PORT_SIZE)
	{
		len	=	USB_COM_PORT_SIZE;		
		memcpy(rxbuffer,&usb_in_arry.buffer[startaddr],len);
		usb_in_arry.len-=len;
		usb_in_arry.status	=	usb_data_read;		//正在读数据
		startaddr+=len;
	}
	else
	{
		len	=	usb_in_arry.len;
		memcpy(rxbuffer,&usb_in_arry.buffer[startaddr],len);
		usb_in_arry.len=0;
		usb_in_arry.status	=	usb_data_idle;		//空闲
		startaddr=0;
	}
	return len;
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
unsigned short api_usb_out_set_data(const unsigned char*	buffer,unsigned short len)
{
	if((usb_data_read_enable==usb_out_arry.status)	//从机可读数据状态
		||(usb_data_full==usb_out_arry.status))				//缓存满
	{
		if(usb_out_arry.len)				//有数据
			return 0;
		else
			usb_out_arry.status=usb_data_idle;		//空闲
	}
	if(usb_out_arry.len+len<=USB_BUFFER_SIZE)		//未超出缓存
	{
		memcpy(&usb_out_arry.buffer[usb_out_arry.len],buffer,len);
		usb_out_arry.len+=len;
		usb_out_arry.status	=	usb_data_write;	//正在写数据
	}
	else if(USB_BUFFER_SIZE>len)
	{
		len=USB_BUFFER_SIZE-usb_out_arry.len;
		memcpy(&usb_out_arry.buffer[usb_out_arry.len],buffer,len);
		usb_out_arry.len+=len;
		usb_out_arry.status	=	usb_data_full;	//缓存满
	}
	else
	{
		usb_out_arry.status	=	usb_data_full;	//缓存满
	}
	return len;
}
/*******************************************************************************
*函数名			:	api_usb_endp_get_out_data
*功能描述		:	读取USB下发数据缓存
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_out_get_data(unsigned char* rxbuffer)
{
	unsigned short len	=	0;
	if((usb_data_full!=usb_out_arry.status)						//缓存满
		&&(usb_data_read!=usb_out_arry.status)					//正在读数据
		&&(usb_data_read_enable!=usb_out_arry.status))	//可读
		return 0;

	len	=	usb_out_arry.len;
	memcpy(rxbuffer,usb_out_arry.buffer,len);
	usb_out_arry.len	=	0;
	usb_out_arry.status	=	usb_data_idle;	//空闲
	return len;
}
/*******************************************************************************
*函数名			:	api_usb_out_set_complete_end
*功能描述		:	存储数据完成/空闲
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_usb_out_set_complete_end(void)
{
	usb_out_arry.status	=	usb_data_read_enable;		//可读
}
/*******************************************************************************
*函数名			:	api_usb_out_set_complete_end
*功能描述		:	存储数据完成/空闲
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_usb_out_get_read_enable(void)
{
	if((usb_data_full!=usb_out_arry.status)						//缓存未满
		||(usb_data_read_enable!=usb_out_arry.status))	//不可读
	{
		return 0;
	}		
	else
	{
		if(0==usb_out_arry.len)
		{
			usb_out_arry.status=usb_data_idle;	//空闲
			return 0;
		}
		return 1;
	}
}
/*******************************************************************************
*函数名			:	api_usb_out_set_complete_end
*功能描述		:	存储数据完成/空闲
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned char api_usb_out_get_write_enable(void)
{
	if((usb_data_idle!=usb_out_arry.status)							//非空闲
		||(usb_data_write!=usb_out_arry.status)						//非写状态
		||(usb_data_write_enable!=usb_out_arry.status))		//非可写状态
		return 0;
	else
		return 1;
}
/*******************************************************************************
*函数名			:	api_usb_out_set_complete_end
*功能描述		:	存储数据完成/空闲
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_out_get_complete_count(void)
{
	return usb_out_arry.len;
}
//------------------------------------------------------------------------------



/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

