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
#include "usb_desc.h"
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

typedef struct
{
	unsigned char complete;				//传送完成标志：0-完成/未激活；1-传送未完成
	unsigned short len;						//当前节点中缓存数据大小
	unsigned char buffer[USB_BUFFER_SIZE];
}usb_data_arry;	//USB主机输出数据到设备缓存

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
*函数名			:	api_usb_in_set_data
*功能描述		:	设置输入/上传数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_in_add_data(const unsigned char*	buffer,unsigned short len)
{
	if(0==usb_in_arry.complete)	//缓存未准备好
	{
		if(usb_in_arry.len)	//有数据
			return 0;
	}
	if(usb_out_arry.len+len<=USB_BUFFER_SIZE-1)		//未超出缓存
	{
		memcpy(&usb_in_arry.buffer[usb_in_arry.len],buffer,len);
		usb_in_arry.len+=len;
		usb_in_arry.complete	=	1;	//未完成写入
	}
	else
	{
		len=USB_BUFFER_SIZE-usb_in_arry.len;
		memcpy(&usb_in_arry.buffer[usb_in_arry.len],buffer,len);
		usb_in_arry.len+=len;
		usb_in_arry.complete	=	0;	//完成写入
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
	usb_in_arry.complete	=	0;	//完成写入
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
	if(usb_in_arry.len>VIRTUAL_COM_PORT_DATA_SIZE)
	{
		len	=	VIRTUAL_COM_PORT_DATA_SIZE;		
		memcpy(rxbuffer,&usb_in_arry.buffer[startaddr],len);
		usb_in_arry.len-=len;
		usb_in_arry.complete	=	1;
		startaddr+=len;
	}
	else
	{
		len	=	usb_in_arry.len;
		memcpy(rxbuffer,&usb_in_arry.buffer[startaddr],len);
		usb_in_arry.len=0;
		usb_in_arry.complete	=	0;
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
	if(0==usb_out_arry.complete)	//缓存未准备好
	{
		if(usb_out_arry.len)	//有数据
			return 0;
	}
	if(usb_out_arry.len+len<=USB_BUFFER_SIZE-1)		//未超出缓存
	{
		memcpy(&usb_out_arry.buffer[usb_out_arry.len],buffer,len);
		usb_out_arry.len+=len;
		usb_out_arry.complete	=	1;	//未完成写入
	}
	else
	{
		len=USB_BUFFER_SIZE-usb_out_arry.len;
		memcpy(&usb_out_arry.buffer[usb_out_arry.len],buffer,len);
		usb_out_arry.len+=len;
		usb_out_arry.complete	=	0;	//完成写入
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
	if(usb_out_arry.complete)
		return 0;

	len	=	usb_out_arry.len;
	memcpy(rxbuffer,usb_out_arry.buffer,len);
	usb_out_arry.len	=	0;
	usb_out_arry.complete	=	1;	//缓存空
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
	usb_out_arry.complete	=	0;
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
unsigned char api_usb_out_get_complete_flag(void)
{
	return usb_out_arry.complete;		//0：完成，1：缓存空闲
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

