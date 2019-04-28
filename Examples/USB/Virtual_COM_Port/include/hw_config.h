/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "usb_type.h"

//#include "stm32f10x_gpio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF

typedef struct _usb_en
{
	unsigned long* 	usb_connect_port;				//GPIOX
	unsigned short 	usb_connect_pin;				//GPIO_Pin_x
}usb_en_def;

/* Exported functions ------------------------------------------------------- */
//------------------------------------------------------api
void api_usb_virtual_com_configuration(usb_en_def* pInfo);		//–Èƒ‚¥Æø⁄≈‰÷√



//------------------------------------------------------Œ¥∂®
void set_usb_en(FunctionalState NewState);
void set_usart_default(void);
bool set_usart_config(void);
void usb_to_usart_Send_data(u8* data_buffer, u8 Nb_bytes);
void usart_to_usb_send_data(void);



//------------------------------------------------------static
static void set_usb_clock(void);				//…Ë÷√USB ±÷”
static void set_usb_Interrupt(void);		//…Ë÷√USB÷–∂œ






void Handle_USBAsynchXfer (void);

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
