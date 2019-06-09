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
#include "stm32f10x_gpio.h"

#include "usb_conf.h"
#include "usb_lib.h"
//	#include "hw_config.h"
//#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_core.h"
#include "usb_pwr.h"
/* Exported types ------------------------------------------------------------*/
//------------------------------------------------------usb_hw
typedef struct _usb_en
{
	unsigned long* 	usb_connect_port;				//GPIOX
	unsigned short 	usb_connect_pin;				//GPIO_Pin_x
}usb_en_def;
//------------------------------------------------------usb_prop
typedef struct
{
  u32 bitrate;			//波特率
  u8 format;				//停止位
  u8 paritytype;		//检验位
  u8 datatype;			//数据位
}LINE_CODING;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define lora	1
#define STM32_FSMC 0
#define STM32_USB_TEST 0
#define usart_buffer_size	1024

#ifdef PS005
		#define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif lora
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif STM32_USB_TEST
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_15
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#elif STM32_FSMC
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
	#else
	  #define USB_DISCONNECT            GPIOA  
		#define USB_DISCONNECT_PIN        GPIO_Pin_8
		#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
#endif

#define ComPort	USART1

//----------------------------------------usb_prop
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


/* Exported functions ------------------------------------------------------- */
void Virtual_COM_Port_Configuration(void);
void Virtual_COM_Port_Server(void);

void api_usb_virtual_com_configuration(usb_en_def* pInfo);		//虚拟串口配置
//------------------------------------------------------usb_hw
void usb_hw_set_connect(FunctionalState NewState);
void usb_hw_set_usart_default(void);
bool usb_hw_set_usart_config(void);
void usb_hw_receive_from_usb(u8* data_buffer, u8 Nb_bytes);
void usb_hw_send_to_usb(void);
void usb_hw_Handle_USBAsynchXfer (void);


void usb_hw_set_clock(void);				//设置USB时钟
void usb_hw_set_Interrupt(void);		//设置USB中断


//------------------------------------------------------usb_prop
void usb_prop_init(void);
void usb_prop_reset(void);
void usb_prop_set_configuration(void);
void usb_prop_set_device_address(void);
void usb_prop_status_in(void);
void usb_prop_status_out(void);
RESULT usb_prop_data_setup(u8);
RESULT usb_prop_no_data_setup(u8);
RESULT usb_prop_get_interface_setting(u8 Interface, u8 AlternateSetting);
u8 *usb_prop_get_device_descriptor(u16 );
u8 *usb_prop_get_config_descriptor(u16);
u8 *Virtual_Com_Port_GetStringDescriptor(u16);

u8 *Virtual_Com_Port_GetLineCoding(u16 Length);	//获取串口通信信息
u8 *Virtual_Com_Port_SetLineCoding(u16 Length);	//设置串口通讯信息

void Get_SerialNum(void);												//获取设备ID号


//------------------------------------------------------usb_istr
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USB_Istr(void);

/* function prototypes Automatically built defining related macros */

#ifdef CTR_CALLBACK
  void CTR_Callback(void);
#endif

#ifdef DOVR_CALLBACK
  void DOVR_Callback(void);
#endif

#ifdef ERR_CALLBACK
  void ERR_Callback(void);
#endif

#ifdef WKUP_CALLBACK
  void WKUP_Callback(void);
#endif

#ifdef SUSP_CALLBACK
  void SUSP_Callback(void);
#endif

#ifdef RESET_CALLBACK
  void RESET_Callback(void);
#endif

#ifdef SOF_CALLBACK
  void SOF_Callback(void);
#endif

#ifdef ESOF_CALLBACK
  void ESOF_Callback(void);
#endif

void usb_endp_in_1(void);
void EP2_IN_Callback(void);
void EP3_IN_Callback(void);
void EP4_IN_Callback(void);
void EP5_IN_Callback(void);
void EP6_IN_Callback(void);
void EP7_IN_Callback(void);

void EP1_OUT_Callback(void);
void EP2_OUT_Callback(void);
void usb_endp_out(void);
void EP4_OUT_Callback(void);
void EP5_OUT_Callback(void);
void EP6_OUT_Callback(void);
void EP7_OUT_Callback(void);






/* External variables --------------------------------------------------------*/

#endif  /*__USB_PWR_H*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
