/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_conf.h
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Virtual COM Port Demo configuration  header
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H


#define	USB_MULTI_IF	1

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/

#define EP_NUM                          (4)			//����˵�����

/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/
///* buffer table base address */
///* buffer table base address */
//#define BTABLE_ADDRESS      (0x00)

///* EP0  */
///* rx/tx buffer base address */
//#define ENDP0_RXADDR        (0x40)
//#define ENDP0_TXADDR        (0x80)

///* EP1  */
///* tx buffer base address */
//#define ENDP1_TXADDR        (0xC0)
//#define ENDP2_TXADDR        (0x100)
//#define ENDP3_RXADDR        (0x110)

/* buffer table base address */
/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)

/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x48)

/* EP1  */
/* tx buffer base address */
#define ENDP1_TXADDR        (0x50)
#define ENDP2_TXADDR        (0x60)
#define ENDP2_RXADDR        (0xA0)
#define ENDP3_TXADDR        (0xE0)
#define ENDP4_TXADDR        (0xF0)
#define ENDP4_RXADDR        (0x130)
#define ENDP5_TXADDR        (0x170)
#define ENDP6_TXADDR        (0x180)
#define ENDP6_RXADDR        (0x1C0)

/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | CNTR_SOFM  | CNTR_RESETM )

/*#define CTR_CALLBACK*/
/*#define DOVR_CALLBACK*/
/*#define ERR_CALLBACK*/
/*#define WKUP_CALLBACK*/
/*#define SUSP_CALLBACK*/
/*#define RESET_CALLBACK*/
#define SOF_CALLBACK
/*#define ESOF_CALLBACK*/

/* CTR service routines */
/* associated to defined endpoints */
/*#define  EP1_IN_Callback   NOP_Process*/
#define  EP2_IN_Callback   NOP_Process
#define  EP3_IN_Callback   NOP_Process
#define  EP4_IN_Callback   NOP_Process
#define  EP5_IN_Callback   NOP_Process
#define  EP6_IN_Callback   NOP_Process
#define  EP7_IN_Callback   NOP_Process

#define  EP1_OUT_Callback   NOP_Process
#define  EP2_OUT_Callback   NOP_Process
/*#define  EP3_OUT_Callback   NOP_Process*/
#define  EP4_OUT_Callback   NOP_Process
#define  EP5_OUT_Callback   NOP_Process
#define  EP6_OUT_Callback   NOP_Process
#define  EP7_OUT_Callback   NOP_Process

#endif /* __USB_CONF_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/