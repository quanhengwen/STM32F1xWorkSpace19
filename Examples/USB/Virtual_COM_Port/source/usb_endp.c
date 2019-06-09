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
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"

#include "usb_istr.h"
#include "usb_pwr.h"
#include "hw_config.h"

#include "usb_endp.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VCOMPORT_IN_FRAME_INTERVAL         	5
#define USB_BUFFER_ARRY_SIZE								16		//缓存数据个数
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char endp_buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
unsigned char endp_buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];

u32 count_out = 0;
u32 count_in = 0;

typedef struct
{
	unsigned char len;
	unsigned char serial;
	unsigned char buffer[VIRTUAL_COM_PORT_DATA_SIZE];
}usb_data_arry;	//USB主机输出数据到设备缓存

usb_data_arry usb_out_arry[USB_BUFFER_ARRY_SIZE];	//USB主机输出数据到设备缓存
usb_data_arry usb_in_arry[USB_BUFFER_ARRY_SIZE];	//设备发送数据到USB主机缓存

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//static unsigned short usb_endp_set_out_data(unsigned char*	buffer,unsigned short len);
//static unsigned char	usb_endp_get_in_data(unsigned char* rxbuffer);
static unsigned char	get_arry_not_empty(usb_data_arry* arry);
static unsigned short usb_endp_set_data(usb_data_arry* arry,unsigned char*	buffer,unsigned short len);
static unsigned char	usb_endp_get_data(usb_data_arry* arry,unsigned char* rxbuffer);
/*******************************************************************************
* Function Name  :	EP3_IN_Callback
* Description    :	端点3输出回调	---设备接收
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  count_out = GetEPRxCount(ENDP3);														//获取USB接收到的数据
  PMAToUserBufferCopy(endp_buffer_out, ENDP3_RXADDR, count_out);		//USB接口到的数据从串口发送
	SetEPRxValid(ENDP3);																				//使能端点3
	
	//----添加程序---USB发送给串口
	//USB_To_USART_Send_Data(buffer_out,count_out);
	usb_endp_set_out_data(endp_buffer_out,count_out);
	api_usb_endp_set_in_data(endp_buffer_out,count_out);
	//返回数据
//	UserToPMABufferCopy(endp_buffer_out, ENDP1_TXADDR, count_out);
//	SetEPTxCount(ENDP1, count_out);																		//设置端点数据长度
//	SetEPTxValid(ENDP1);																							//使能端点
}

/*******************************************************************************
* Function Name  : 	EP1_IN_Callback
* Description    :	端点1输入的回调函数	--设备发送
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
//	count_in = 0;
//	Handle_USBAsynchXfer();
//	USART_To_USB_Send_Data();
//	count_in=usb_endp_get_in_data(endp_buffer_in);
//	if(count_in)
//	{
//		UserToPMABufferCopy(endp_buffer_in, ENDP1_TXADDR, count_in);
//		SetEPTxCount(ENDP1, count_in);
//		SetEPTxValid(ENDP1);
//	}
}
/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
//  static u32 FrameCount = 0;
//  
//  if(bDeviceState == CONFIGURED)
//  {
//    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
//    {
//      /* Reset the frame counter */
//      FrameCount = 0;
//      
//      /* Check the data to be sent through IN pipe */
//      Handle_USBAsynchXfer();
//    }
//  }
//	Handle_USBAsynchXfer();
//	USART_To_USB_Send_Data();
//	if(api_get_arry_not_empty(usb_in_arry))
//	{
//		SetEPTxValid(ENDP1);
//	}
	//---------------------------------向USB发送数据
//	count_in=usb_endp_get_in_data(endp_buffer_in);
//	if(count_in)
//	{
//		UserToPMABufferCopy(endp_buffer_in, ENDP1_TXADDR, count_in);
//		SetEPTxCount(ENDP1, count_in);
//		SetEPTxValid(ENDP1);
//	}
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
void api_usb_endp_set_buffer_clear(unsigned char* rxbuffer)
{
	unsigned char i	=	0;
	for(i=0;i<VIRTUAL_COM_PORT_DATA_SIZE;i++)
	{
		usb_out_arry[i].len			=	0;
		usb_out_arry[i].serial	=	0;
		
		usb_in_arry[i].len			=	0;
		usb_in_arry[i].serial		=	0;
	}
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
unsigned char api_usb_endp_get_out_data(unsigned char* rxbuffer)
{
	unsigned char len	=	0;
	len	=	usb_endp_get_data(usb_out_arry,rxbuffer);
	return len;
}
/*******************************************************************************
*函数名			:	usb_endp_set_in_data
*功能描述		:	将数据发送到USB主机缓存
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short api_usb_endp_set_in_data(unsigned char*	txbuffer,unsigned short len)
{
	unsigned short in_len_succes	=	0;
	in_len_succes	=	usb_endp_set_data(usb_in_arry,txbuffer,len);
	return in_len_succes;
}
//------------------------------------------------------------------------------




/*******************************************************************************
*函数名			:	usb_endp_set_out_data
*功能描述		:	缓存USB主发下发的数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
unsigned short usb_endp_set_out_data(unsigned char*	buffer,unsigned short len)
{
	unsigned short in_len_succes	=	0;
	in_len_succes	=	usb_endp_set_data(usb_out_arry,buffer,len);
	return in_len_succes;
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
unsigned char usb_endp_get_in_data(unsigned char* rxbuffer)
{
	unsigned char len	=	0;
	len	=	usb_endp_get_data(usb_in_arry,rxbuffer);
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
static unsigned char get_arry_not_empty(usb_data_arry* arry)
{
	unsigned char i	=	0;
	for(i=0;i<USB_BUFFER_ARRY_SIZE;i++)
	{
		if(usb_in_arry[i].len>0)	//有数据
		{
			return 1;
		}
	}	
	return 0;		//无数据
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
static unsigned short usb_endp_set_data(usb_data_arry* arry,unsigned char*	buffer,unsigned short len)
{
	unsigned char i	=	0,j=0;
	unsigned char serial	=	0;
	unsigned char in_len	=	0;
	unsigned short in_len_succes	=	0;
	usb_data_arry* data_arry	=	arry;
	//----------------------校验数据
	if(NULL==buffer)
		return	ERROR;
	
	start_buffer_in:
	if(0==len)
		goto end_buffer_in;
	//----------------------缓存大小检查
	if(len>VIRTUAL_COM_PORT_DATA_SIZE)
	{
		in_len=VIRTUAL_COM_PORT_DATA_SIZE;
	}
	else
	{
		in_len	=	len;
	}	
	//----------------------获取最大序号
	for(i=0;i<USB_BUFFER_ARRY_SIZE;i++)
	{
		if(data_arry[i].len>0)	//有数据
		{
			if(data_arry[i].serial>=serial)
			{
				serial	=	data_arry[i].serial+serial+1;		//新序号+1
			}
		}
	}
	//----------------------检查缓存容量
	if(serial>=USB_BUFFER_ARRY_SIZE)	//超出
		goto end_buffer_in;
	//----------------------存储数据
	for(i=0;i<USB_BUFFER_ARRY_SIZE;i++)
	{
		if(0==data_arry[i].len)	//无数据
		{
			data_arry[i].len			=	in_len;
			data_arry[i].serial	=	serial;
			for(j=0;j<in_len;j++)
			{
				data_arry[i].buffer[j]=buffer[j];
			}
			break;		//退出循环
		}
	}
	
	in_len_succes+=in_len;
	len	=	len-in_len;
	if(len>0)
		buffer	=	&buffer[in_len_succes];
	goto start_buffer_in;
	
	end_buffer_in:
	return in_len_succes;		//正确执行
}
/*******************************************************************************
*函数名			:	usb_endp_get_data
*功能描述		:	获取待上传到USB主机的数据
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char usb_endp_get_data(usb_data_arry* arry,unsigned char* rxbuffer)
{
	unsigned char i	=	0,j=0;
	unsigned char len	=	0;
	usb_data_arry* data_arry	=	arry;
	//------------------------------读取数据
	for(i=0;i<USB_BUFFER_ARRY_SIZE;i++)
	{
		if(data_arry[i].len>0)	//有数据
		{
			if(0==data_arry[i].serial)
			{
				len	=	data_arry[i].len;
				for(j=0;j<len;j++)
				{
					rxbuffer[j]=data_arry[i].buffer[j];
				}
				data_arry[i].len	=	0;
				break;
			}
		}
	}
	//------------------------------清除一个缓存
	for(i=0;i<USB_BUFFER_ARRY_SIZE;i++)
	{
		if(data_arry[i].serial>0)	//序号减1
		{
			data_arry[i].serial-=1;
		}
	}	
	return len;
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

