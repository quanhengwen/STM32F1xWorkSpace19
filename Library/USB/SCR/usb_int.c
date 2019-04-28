/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_int.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Endpoint CTR (Low and High) interrupt's service routines
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u16 SaveRState;
u16 SaveTState;
/*  The number of current endpoint, it will be used to specify an endpoint */
 u8	EPindex;		//�˵�������
/* Extern variables ----------------------------------------------------------*/
extern void (*pEpInt_IN[7])(void);    /*  Handles IN  interrupts   */
extern void (*pEpInt_OUT[7])(void);   /*  Handles OUT interrupts   */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : CTR_LP.
* Description    : ����˵����ݴ�������ж�
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CTR_LP(void)			//�����ã�usb_istr.c			//���ڶ˵��������������жϴ�
{
//  u32 wEPVal = 0;
  /* stay in loop while pending ints */
  while (((USB_REG.wIstr.istr = _GetISTR()) & ISTR_CTR) != 0)	//��ȡ��ȷ�Ĵ���
  {
    _SetISTR((u16)CLR_CTR); 		/* clear CTR flag */	//�����ȷ�Ĵ����־ (Correct transfer)
    /* extract highest priority endpoint number */
    EPindex = (u8)(USB_REG.wIstr.istr_flag.EP_ID);				//�ҵ��ڵ�������//�˵�ID (Endpoint Identifier)////��ȡ���ݴ�����ԵĶ˵��
    if (EPindex == 0)			//0�Ŷ˵�--���ƴ��䣨����USBö�٣�
    {
      SaveRState = _GetEPRxStatus(ENDP0);		//������ն˵�״̬
      SaveTState = _GetEPTxStatus(ENDP0);		//���淢�Ͷ˵�״̬
      _SetEPRxStatus(ENDP0, EP_RX_NAK);			//�˵���NAK������Ӧ���еķ�������
      _SetEPTxStatus(ENDP0, EP_TX_NAK);			//�˵���NAK������Ӧ���еĽ�������

      if ((USB_REG.wIstr.istr_flag.DIR) == 0)		//����//���䷽�� (Direction of transaction)�������豸
      {
        _ClearEP_CTR_TX(ENDP0);		//������ն˵�״̬
        In0_Process();						//usb_core.c

				_SetEPRxStatus(ENDP0, SaveRState);		//��ԭ״̬
				_SetEPTxStatus(ENDP0, SaveTState);		//��ԭ״̬
				return;
      }
      else			//����(�豸������)
      {
        usb_EP_def	EP;
				EP.wEPVal	=	 get_endpint(ENDP0);				//wEPVal = _GetENDPOINT(ENDP0);		//��ȡ�˵�״̬
        if(0!=EP.wEPVal_flag.CTR_TX)		//if ((wEPVal & EP_CTR_TX) != 0)					//��ȷ���ͱ�־λ (Correct transfer for transmission)
        {
          _ClearEP_CTR_TX(ENDP0);
          In0_Process();						//usb_core.c//�˵�0���ʹ��������������գ�
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
        else if (0!=EP.wEPVal_flag.SETUP)	//else if ((wEPVal &EP_SETUP) != 0)		//SETUP���鴫����ɱ�־λ (Setup transaction completed)
        {
          _ClearEP_CTR_RX(ENDP0); /* SETUP bit kept frozen while CTR_RX = 1 */
          Setup0_Process();				//usb_core.c
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
        else if (0!=EP.wEPVal_flag.CTR_RX)		//else if ((wEPVal & EP_CTR_RX) != 0)		//��ȷ���ձ�־λ (Correct Transfer for reception)
        {
          _ClearEP_CTR_RX(ENDP0);
          Out0_Process();			//usb_core.c//�˵�0���봦�������������ͣ�
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
      }
    }/* if(EPindex == 0) */
    else									//�����˵�
    {
			usb_EP_def	EP;
			EP.wEPVal	=	 get_endpint(EPindex);				//wEPVal = _GetENDPOINT(ENDP0);		//��ȡ�˵�״̬
      if (0!=EP.wEPVal_flag.CTR_RX)	//if ((wEPVal & EP_CTR_RX) != 0)		//���ս��ջ�����������
      {
        /* clear int flag */
        _ClearEP_CTR_RX(EPindex);				//������˵�״̬

        /* call OUT service function */
        (*pEpInt_OUT[EPindex-1])();			//�����������--�������

      } /* if((wEPVal & EP_CTR_RX) */
      if (0!=EP.wEPVal_flag.CTR_TX)	//if ((wEPVal & EP_CTR_TX) != 0)
      {
        /* clear int flag */
        _ClearEP_CTR_TX(EPindex);

        /* call IN service function */
        (*pEpInt_IN[EPindex-1])();				//�˵㴦����--������ȡ
      } /* if((wEPVal & EP_CTR_TX) != 0) */

    }/* if(EPindex == 0) else */

  }/* while(...) */
}

/*******************************************************************************
* Function Name  : CTR_HP.
* Description    : High Priority Endpoint Correct Transfer interrupt's service 
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CTR_HP(void)
{
  u32 wEPVal = 0;

  while (((USB_REG.wIstr.istr = _GetISTR()) & ISTR_CTR) != 0)
  {
    _SetISTR((u16)CLR_CTR); /* clear CTR flag */
    /* extract highest priority endpoint number */
    EPindex = (u8)(USB_REG.wIstr.istr & ISTR_EP_ID);
    /* process related endpoint register */
    wEPVal = get_endpint(EPindex);
    if ((wEPVal & EP_CTR_RX) != 0)
    {
      /* clear int flag */
      _ClearEP_CTR_RX(EPindex);

      /* call OUT service function */
      (*pEpInt_OUT[EPindex-1])();

    } /* if((wEPVal & EP_CTR_RX) */
    else if ((wEPVal & EP_CTR_TX) != 0)
    {
      /* clear int flag */
      _ClearEP_CTR_TX(EPindex);

      /* call IN service function */
      (*pEpInt_IN[EPindex-1])();


    } /* if((wEPVal & EP_CTR_TX) != 0) */

  }/* while(...) */
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
