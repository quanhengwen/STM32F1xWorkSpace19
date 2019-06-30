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
#include "usb_data.h"

#include "Virtual_COM_Port.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VCOMPORT_IN_FRAME_INTERVAL         	5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char usb_out_loop	=	0;		//���������־/����������м���
unsigned char usb_in_loop		=	0;		//���������־/����������м���
unsigned char endp_buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
unsigned char endp_buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];
unsigned long usbnum=0;
unsigned long usbsavenum=0;
extern unsigned long sendnum;
extern unsigned long sendednum;

//u32 count_out = 0;
//u32 count_in = 0;
unsigned char usb_out_complete_flag=0;
unsigned short usb_out_save_count	=	0;
unsigned short usb_out_count	=	0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void usb_out_end(void);


/*******************************************************************************
* Function Name  :	EP3_IN_Callback
* Description    :	�˵�3����ص�	---�豸����
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	
	
	unsigned char complete=api_usb_out_get_complete_flag();
	unsigned short counnt=api_usb_out_get_complete_count();	
	
	usb_out_loop	=	0;
	if(0==counnt)
	{
		goto save_data;
	}
	else
	{
		if(complete)
		{
			goto save_data;
		}
		else
		{
			SetEPRxStatus(ENDP3, EP_RX_NAK);			//�˵���STALL������Ӧ���еķ�������
			return ;
		}
	}
	
	save_data:
	usb_out_count = GetEPRxCount(ENDP3);														//��ȡUSB���յ�������
  PMAToUserBufferCopy(endp_buffer_out, ENDP3_RXADDR, usb_out_count);		//USB�ӿڵ������ݴӴ��ڷ���
	if(usb_out_count)
	{		
		usb_out_count	=api_usb_out_set_data(endp_buffer_out,usb_out_count);
		if(usb_out_count)
			SetEPRxValid(ENDP3);																				//ʹ�ܶ˵�3			
		else
			SetEPRxStatus(ENDP3, EP_RX_NAK);			//�˵���STALL������Ӧ���еķ�������
			usbsavenum+=usb_out_count;
	}
	return;
	//  count_out = GetEPRxCount(ENDP3);														//��ȡUSB���յ�������
	//  PMAToUserBufferCopy(endp_buffer_out, ENDP3_RXADDR, count_out);		//USB�ӿڵ������ݴӴ��ڷ���
	//	SetEPRxValid(ENDP3);																				//ʹ�ܶ˵�3
	
	
	//SaveRState = _GetEPRxStatus(ENDP3);		//������ն˵�״̬
	//if(count_out)
		//api_usb_out_set_data(endp_buffer_out,count_out);
	//_SetEPRxStatus(ENDP3, SaveRState);		//��ԭ״̬
	//SetEPRxValid(ENDP3);
	//----��ӳ���---USB���͸�����

//	api_usb_data_set_out_data(endp_buffer_out,count_out);
//	api_usb_in_set_data(endp_buffer_out,count_out);
	
	
	
	//��������
//	UserToPMABufferCopy(endp_buffer_out, ENDP1_TXADDR, count_out);
//	SetEPTxCount(ENDP1, count_out);																		//���ö˵����ݳ���
//	SetEPTxValid(ENDP1);																							//ʹ�ܶ˵�
}

/*******************************************************************************
* Function Name  : 	EP1_IN_Callback
* Description    :	�˵�1����Ļص�����	--�豸����
* Input          : 	None.
* Output         : 	None.
* Return         : 	None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
	unsigned short count_in	=	0;
	usb_in_loop	=	0;
	
	usb_out_end();

	count_in=api_usb_in_get_data(endp_buffer_in);
	if(count_in)
	{
		UserToPMABufferCopy(endp_buffer_in, ENDP1_TXADDR, count_in);
		SetEPTxCount(ENDP1, count_in);		
	}
	else
	{
		SetEPTxCount(ENDP1, 0);		
	}
	SetEPTxValid(ENDP1);
	
	//	Handle_USBAsynchXfer();
	//	USART_To_USB_Send_Data();
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
	unsigned short count_in	=	0;
	static unsigned short temp=0;

	//---------------------------------�����������м����ﵽԤ�����������µ����뻺��
	if(usb_in_loop++>VCOMPORT_IN_FRAME_INTERVAL)
	{
		usb_in_loop	=	0;
		
		count_in	=	api_usb_in_get_data(endp_buffer_in);
		
		if(count_in)
		{
			UserToPMABufferCopy(endp_buffer_in, ENDP1_TXADDR, count_in);
			SetEPTxCount(ENDP1, count_in);
			SetEPTxValid(ENDP1);
		}
		else
		{
			SetEPTxCount(ENDP1, 0);
			SetEPTxValid(ENDP1);
		}
	//SetEPTxCount(ENDP1, 63);
	//SetEPTxValid(ENDP1);
	}	
}
/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_out_end(void)
{
	unsigned short count_in	=	0;
	static unsigned short temp=0;

	//FNR_COUNT1	=	GetFNR();
	//FNR_COUNT	=	FNR_COUNT1;
	//---------------------------------����������м����ﵽԤ���������������д���־
	if(usb_out_loop++>=VCOMPORT_IN_FRAME_INTERVAL)
	//if(((fnrcc>count_in)&&(fnrcc-count_in>5))||((count_in>fnrcc)&&(count_in-fnrcc>5)))
	{		
		unsigned char complete=api_usb_out_get_complete_flag();
		unsigned short counnt=api_usb_out_get_complete_count();		
		if(complete)	//�ɽ��ձ�־
		{
			if(counnt)		//������
			{
				api_usb_out_set_complete_end();	
				//usb_to_uart_server();
				SetEPRxStatus(ENDP3, EP_RX_NAK);			//�˵���STALL������Ӧ���еķ�������
			}
			else
			{
				SetEPRxValid(ENDP3);
			}
		}
		else
		{
			if(counnt)		//������
			{
				//usb_to_uart_server();
				SetEPRxStatus(ENDP3, EP_RX_NAK);			//�˵���STALL������Ӧ���еķ�������
			}
			else
			{
				SetEPRxValid(ENDP3);
			}			
		}
		usb_out_loop	=	0;		
	}
}
//------------------------------------------------------------------------------



/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

