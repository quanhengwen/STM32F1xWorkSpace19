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
#ifndef __Virtual_COM_Port_H
#define __Virtual_COM_Port_H

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef struct
{
  unsigned long bitrate;			//波特率
  unsigned char format;				//停止位
  unsigned char paritytype;		//检验位
  unsigned char datatype;			//数据位
}LINE_CODING;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define Virtual_Com_Port_GetConfiguration          NOP_Process
//#define Virtual_Com_Port_SetConfiguration          NOP_Process
#define Virtual_Com_Port_GetInterface              NOP_Process
#define Virtual_Com_Port_SetInterface              NOP_Process
#define Virtual_Com_Port_GetStatus                 NOP_Process
#define Virtual_Com_Port_ClearFeature              NOP_Process
#define Virtual_Com_Port_SetEndPointFeature        NOP_Process
#define Virtual_Com_Port_SetDeviceFeature          NOP_Process
//#define Virtual_Com_Port_SetDeviceAddress          NOP_Process

#define SEND_ENCAPSULATED_COMMAND   0x00			//
#define GET_ENCAPSULATED_RESPONSE   0x01			//
#define SET_COMM_FEATURE            0x02			//设置串口特性请求
#define GET_COMM_FEATURE            0x03			//获取串口特性请求
#define CLEAR_COMM_FEATURE          0x04			//清除串口特性请求
#define SET_LINE_CODING             0x20			//设置串口通讯信息请求
#define GET_LINE_CODING             0x21			//获取串口通信信息请求
#define SET_CONTROL_LINE_STATE      0x22			//设置控制信息状态
#define SEND_BREAK                  0x23			//
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void api_usb_virtual_com_configuration(void);
void api_usb_virtual_com_server(void);




void USART_Config_Default(void);
char USART_Config(void);
//void USB_To_USART_Send_Data(unsigned char* data_buffer, unsigned char Nb_bytes);
void USART_To_USB_Send_Data(void);
void usb_virtual_com_AsynchXfer (void);


#endif  /*__USB_PWR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
