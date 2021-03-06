/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "stm32f10x.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];		//存放USB发送过来的数据，然后转存到FifoRx
extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_in;		//串口接收数据计数器
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t  USB_Tx_State;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
  uint16_t USB_Tx_ptr;					//上传数据所在缓存中的起始地址
  uint16_t USB_Tx_length;				//上传数据长度

  if (USB_Tx_State == 1)				//有发送请求
  {
    if (USART_Rx_length == 0) 
    {
      USB_Tx_State = 0;
    }
    else 
    {
			//-------------------------------------检查数据长度有无超过一帧64字节
      if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
			{
				//--------------------获取待发送数据的起始缓存序号及长度
        USB_Tx_ptr = USART_Rx_ptr_out;						
        USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
        //--------------------更新下次待发数据的起始缓存地址及更新剩余待发送长度
        USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
        USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;    
      }
      else 
      {
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = USART_Rx_length;
        
        USART_Rx_ptr_out += USART_Rx_length;
        USART_Rx_length = 0;
      }			
      UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1); 
    }
  }
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :	USB输出数据
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);		//读取USB端点数据，放入USB接收缓冲区USB_Rx_Buffer
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the USART Xfer */
  
  USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);		//USB向USART发数据---将USB接收缓冲区USB_Rx_Buffer的数据放入FifoRx队列
 
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);		//使能USB接收
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
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();			//检查输入端点的数据发送状况
    }
  }  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

