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


unsigned short api_BqApp_get_digest(void);
void api_BqApp_gpio_configuration(void);


unsigned char api_BqApp_Work_As_Smt_Server(void);
unsigned char api_BqApp_Work_As_Feeder_Server(void);
unsigned char api_BqApp_Sample_Data_Verify(void);
unsigned char api_BqApp_Get_Sample_Digest(void);


static void api_BqApp_usb_to_feeder(void);
static void api_BqApp_set_sys_led(unsigned char flag);
//static void api_BqApp_set_feeder_connect(unsigned char flag);
static void api_BqApp_set_usart_connect(void);





static unsigned char api_BqApp_set_certification_message(const unsigned char* message);

#endif  /*__USB_PWR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
