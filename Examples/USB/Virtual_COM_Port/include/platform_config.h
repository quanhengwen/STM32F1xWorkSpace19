/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */


#if !defined (USE_STM3210B_EVAL) &&  !defined (USE_STM3210E_EVAL)
 //#define USE_STM3210B_EVAL
 #define USE_STM3210E_EVAL
#endif



//#define ComPort	USART1

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
