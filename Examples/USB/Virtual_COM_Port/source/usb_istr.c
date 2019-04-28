/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_istr.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : ISTR events interrupt service routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/******************************** ˵��20160912**********************************
********************************************************************************
* ���ܣ�ֱ�Ӵ���USB�ж�
* 
* 
* 
* 
* 
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "usb_istr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//volatile u16 wIstr;  					/* ISTR register last read value */	//�ж�ֵ
volatile u8 bIntPackSOF = 0;  /* SOFs received between 2 consecutive packets */
//usb_istr_def	wIstr;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* function pointers to non-control endpoints service routines */
void (*pEpInt_IN[7])(void) =
  {
    EP1_IN_Callback,
    EP2_IN_Callback,
    EP3_IN_Callback,
    EP4_IN_Callback,
    EP5_IN_Callback,
    EP6_IN_Callback,
    EP7_IN_Callback,
  };

void (*pEpInt_OUT[7])(void) =
  {
    EP1_OUT_Callback,
    EP2_OUT_Callback,
    EP3_OUT_Callback,
    EP4_OUT_Callback,
    EP5_OUT_Callback,
    EP6_OUT_Callback,
    EP7_OUT_Callback,
  };


/*******************************************************************************
* Function Name  : USB_Istr.
* Description    : ISTR events interrupt service routine.
*	����˵��				:		�ж��¼�����-�����ж�״̬�Ĵ����ж��ж����ͣ�ת����Ӧ�Ĵ�����
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Istr(void)
{

  USB_REG.wIstr.istr	=	_GetISTR();	//wIstr = _GetISTR();			//��ȡ�жϱ�־���õ��ж�Դ
	//--------------------------USB��λ���� (USB reset request)
#if (IMR_MSK & ISTR_RESET)
  if (USB_REG.wIstr.istr & ISTR_RESET & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_RESET);		//���USB��λ���� (USB reset request)
		pProperty->Reset();					//USB��λ
#ifdef RESET_CALLBACK
    RESET_Callback();
#endif
  }
#endif
  //--------------------------PMA�������(Packet memory area over / underrun)
#if (IMR_MSK & ISTR_DOVR)
  if (USB_REG.wIstr.istr & ISTR_DOVR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_DOVR);	//������黺������� (Packet memory area over / underrun)
#ifdef DOVR_CALLBACK
    DOVR_Callback();
#endif
  }
#endif
  //--------------------------�������USBӦ�ó���ͨ�����Ժ�����Щ������ΪUSBģ��������ڷ�������ʱ���������ش����ơ�
	//													��λ�������жϿ�������Ӧ�ó���Ŀ����׶Σ������������USB���ߵĴ�����������ʶ�û����ܷ����Ĵ���(�������ɣ������������أ�USB���𻵵�)��
#if (IMR_MSK & ISTR_ERR)			//���� (Error)
  if (USB_REG.wIstr.istr & ISTR_ERR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ERR);		//������� (Error)
#ifdef ERR_CALLBACK
    ERR_Callback();
#endif
  }
#endif
  //--------------------------�������� (Wakeup)
#if (IMR_MSK & ISTR_WKUP)
  if (USB_REG.wIstr.istr & ISTR_WKUP & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_WKUP);	//����������� (Wakeup)
    Resume(RESUME_EXTERNAL);
#ifdef WKUP_CALLBACK
    WKUP_Callback();
#endif
  }
#endif
  //--------------------------����ģ������ (Suspend mode request)
#if (IMR_MSK & ISTR_SUSP)
  if (USB_REG.wIstr.istr & ISTR_SUSP & wInterrupt_Mask)
  {
    /* check if SUSPEND is possible */
    if (fSuspendEnabled)
    {
      Suspend();
    }
    else
    {
      /* if not possible then resume after xx ms */
      Resume(RESUME_LATER);
    }
    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    _SetISTR((u16)CLR_SUSP);
#ifdef SUSP_CALLBACK
    SUSP_Callback();
#endif
  }
#endif
  //--------------------------//֡��(SOF)�жϱ�־���жϷ���������ͨ�����SOF�¼��������������1msͬ
#if (IMR_MSK & ISTR_SOF)										
  if (USB_REG.wIstr.istr & ISTR_SOF & wInterrupt_Mask)		//�������жϱ�־��SOF�жϱ�־����SOF�ж�ʹ����
  {
    _SetISTR((u16)CLR_SOF);									//���SOF�жϱ�־
    bIntPackSOF++;													//ͳ�ƹ����յ�����SOF

#ifdef SOF_CALLBACK
    SOF_Callback();													//��������SOF_CALLBACK�������SOF_Callback,���Ӻ���һ�����ڷ���SOF�ж�ʱ����ʲô
#endif
  }
#endif
  //--------------------------����֡�ױ�ʶλ (Expected start of frame)	�����������3��ESOF�жϣ�Ҳ��������3��δ�յ�SOF���飬������SUSP�жϡ���ʹ�ڹ���ʱ��δ������ʱ����SOF���鶪ʧ����λҲ�ᱻ��λ��
#if (IMR_MSK & ISTR_ESOF)								
  if (USB_REG.wIstr.istr & ISTR_ESOF & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ESOF);
    /* resume handling timing is made with ESOFs */
    Resume(RESUME_ESOF); /* request without change of the machine state */

#ifdef ESOF_CALLBACK
    ESOF_Callback();
#endif
  }
#endif
  //--------------------------��ȷ�Ĵ��� (Correct transfer)����λ�ڶ˵���ȷ���һ�����ݴ������Ӳ����λ��Ӧ�ó������ͨ��DIR��EP_IDλ��ʶ�����ĸ��˵��������ȷ�����ݴ��䡣
#if (IMR_MSK & ISTR_CTR)									
  if (USB_REG.wIstr.istr & ISTR_CTR & wInterrupt_Mask)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    CTR_LP();				//usb_int.c						//������ȷ�����жϷ�����򣺿��ƶ˵㼰�����˵�������
#ifdef CTR_CALLBACK			//δ����
    CTR_Callback();
#endif
  }
#endif
} /* USB_Istr */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
