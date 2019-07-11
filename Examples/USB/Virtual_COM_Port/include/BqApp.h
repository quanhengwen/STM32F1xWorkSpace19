/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_pwr.h
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Connection/disconnection & power management header
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BqApp_H
#define __BqApp_H

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef struct 
{
	unsigned char flag;				//1需要验证，0空闲
	unsigned char message[20];
}CertificationDataDef;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void api_BqApp_configuration(void);
void api_BqApp_server(void);
void api_BqApp_sample_data_verify(void);
void api_BqApp_sample_data_get_digest(void);
unsigned short api_BqApp_get_digest(void);
void api_BqApp_gpio_configuration(void);


void api_BqApp_certification_message_server(void);
static unsigned char api_BqApp_set_certification_message(const unsigned char* message);

#endif  /*__USB_PWR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
