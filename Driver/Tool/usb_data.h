/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_istr.h
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : This file includes the peripherals header files in the
*                      user application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DATA_H
#define __USB_DATA_H

/* Includes ------------------------------------------------------------------*/
//#include "usb_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USB_BUFFER_SIZE	2048
#define USB_COM_PORT_SIZE 64
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

	
typedef enum
{
	usb_data_idle		=	(unsigned char)0,		//空闲
	usb_data_write 	= (unsigned char)1,		//正在写数据
	usb_data_read 	= (unsigned char)2,		//正在读数据
	usb_data_full 	= (unsigned char)3,		//缓存满
	usb_data_read_enable	=	(unsigned char)4,	//可读
	usb_data_write_enable	=	(unsigned char)5,	//可写
}usb_data_status_def;
typedef struct
{
	usb_data_status_def status;				//传送完成标志：0-完成/未激活；1-传送未完成
	unsigned short 			len;						//当前节点中缓存数据大小
	unsigned char 			buffer[USB_BUFFER_SIZE];
}usb_data_arry;	//USB主机输出数据到设备缓存
/* function prototypes Automatically built defining related macros */
unsigned short api_usb_in_add_data(const unsigned char*	buffer,unsigned short len);
void api_usb_in_set_complete_end(void);
unsigned short api_usb_in_set_data(const unsigned char*	buffer,unsigned short len);
unsigned short api_usb_in_get_data(unsigned char* rxbuffer);


unsigned short api_usb_out_set_data(const unsigned char*	buffer,unsigned short len);
unsigned short api_usb_out_get_data(unsigned char* rxbuffer);
void api_usb_out_set_complete_end(void);
unsigned char api_usb_out_get_read_enable(void);
unsigned char api_usb_out_get_write_enable(void);
unsigned short api_usb_out_get_complete_count(void);

#endif /*__USB_ISTR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
