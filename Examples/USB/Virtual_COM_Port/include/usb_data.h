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
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


/* function prototypes Automatically built defining related macros */
unsigned short api_usb_in_set_data(const unsigned char*	buffer,unsigned short len);
unsigned short api_usb_in_get_data(unsigned char* rxbuffer);
unsigned short api_usb_out_set_data(const unsigned char*	buffer,unsigned short len);
unsigned short api_usb_out_get_data(unsigned char* rxbuffer);
void api_usb_out_set_complete_end(void);
unsigned char api_usb_out_get_complete_flag(void);
unsigned short api_usb_out_get_complete_count(void);

#endif /*__USB_ISTR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
