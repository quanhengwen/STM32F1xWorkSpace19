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
 u8	EPindex;		//端点索引号
/* Extern variables ----------------------------------------------------------*/
extern void (*pEpInt_IN[7])(void);    /*  Handles IN  interrupts   */
extern void (*pEpInt_OUT[7])(void);   /*  Handles OUT interrupts   */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : CTR_LP.
* Description    : 处理端点数据传输完成中断
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CTR_LP(void)			//被调用：usb_istr.c			//用于端点数据输入输入中断处
{
//  u32 wEPVal = 0;
  /* stay in loop while pending ints */
  while (((USB_REG.wIstr.istr = _GetISTR()) & ISTR_CTR) != 0)	//获取正确的传输
  {
    _SetISTR((u16)CLR_CTR); 		/* clear CTR flag */	//清除正确的传输标志 (Correct transfer)
    /* extract highest priority endpoint number */
    EPindex = (u8)(USB_REG.wIstr.istr_flag.EP_ID);				//找到节点索引号//端点ID (Endpoint Identifier)////获取数据传输针对的端点号
    if (EPindex == 0)			//0号端点--控制传输（包含USB枚举）
    {
      SaveRState = _GetEPRxStatus(ENDP0);		//保存接收端点状态
      SaveTState = _GetEPTxStatus(ENDP0);		//保存发送端点状态
      _SetEPRxStatus(ENDP0, EP_RX_NAK);			//端点以NAK分组响应所有的发送请求。
      _SetEPTxStatus(ENDP0, EP_TX_NAK);			//端点以NAK分组响应所有的接收请求。

      if ((USB_REG.wIstr.istr_flag.DIR) == 0)		//发送//传输方向 (Direction of transaction)主机到设备
      {
        _ClearEP_CTR_TX(ENDP0);		//清除接收端点状态
        In0_Process();						//usb_core.c

				_SetEPRxStatus(ENDP0, SaveRState);		//还原状态
				_SetEPTxStatus(ENDP0, SaveTState);		//还原状态
				return;
      }
      else			//接收(设备到主机)
      {
        usb_EP_def	EP;
				EP.wEPVal	=	 get_endpint(ENDP0);				//wEPVal = _GetENDPOINT(ENDP0);		//获取端点状态
        if(0!=EP.wEPVal_flag.CTR_TX)		//if ((wEPVal & EP_CTR_TX) != 0)					//正确发送标志位 (Correct transfer for transmission)
        {
          _ClearEP_CTR_TX(ENDP0);
          In0_Process();						//usb_core.c//端点0发送处理函数（主机接收）
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
        else if (0!=EP.wEPVal_flag.SETUP)	//else if ((wEPVal &EP_SETUP) != 0)		//SETUP分组传输完成标志位 (Setup transaction completed)
        {
          _ClearEP_CTR_RX(ENDP0); /* SETUP bit kept frozen while CTR_RX = 1 */
          Setup0_Process();				//usb_core.c
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
        else if (0!=EP.wEPVal_flag.CTR_RX)		//else if ((wEPVal & EP_CTR_RX) != 0)		//正确接收标志位 (Correct Transfer for reception)
        {
          _ClearEP_CTR_RX(ENDP0);
          Out0_Process();			//usb_core.c//端点0输入处理函数（主机发送）
          /* before terminate set Tx & Rx status */
          _SetEPRxStatus(ENDP0, SaveRState);
          _SetEPTxStatus(ENDP0, SaveTState);
          return;
        }
      }
    }/* if(EPindex == 0) */
    else									//其它端点
    {
			usb_EP_def	EP;
			EP.wEPVal	=	 get_endpint(EPindex);				//wEPVal = _GetENDPOINT(ENDP0);		//获取端点状态
      if (0!=EP.wEPVal_flag.CTR_RX)	//if ((wEPVal & EP_CTR_RX) != 0)		//接收接收缓冲区有数据
      {
        /* clear int flag */
        _ClearEP_CTR_RX(EPindex);				//清楚读端点状态

        /* call OUT service function */
        (*pEpInt_OUT[EPindex-1])();			//处理接收数据--主机输出

      } /* if((wEPVal & EP_CTR_RX) */
      if (0!=EP.wEPVal_flag.CTR_TX)	//if ((wEPVal & EP_CTR_TX) != 0)
      {
        /* clear int flag */
        _ClearEP_CTR_TX(EPindex);

        /* call IN service function */
        (*pEpInt_IN[EPindex-1])();				//端点处理函数--主机读取
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
