#ifdef VirtualCom

#include "VirtualCom.H"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_type.h"
#include "STM32F10x_BitBand.H"

#include "STM32_SYS.H"
#include "STM32_PWM.H"
#include "STM32_SYSTICK.H"
#include "STM32_GPIO.H"
#include "STM32_MEM.H"


#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

#include "STM32_USART.H"


//extern u8 buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
//extern u8 buffer_rx[VIRTUAL_COM_PORT_DATA_SIZE];
//extern u8 buffer_tx[VIRTUAL_COM_PORT_DATA_SIZE];

extern u32 count_out;

u8	*VirtualCom_Buffer;
u32 VirtualCom_BufferSize=0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void VirtualCom_Configuration(void)
{
	SYS_Configuration();	//ϵͳ���� STM32_SYS.H
	
	Set_System();					//hw_config.h
	
	VirtualCom_LedConfiguration();
	
	Set_USBClock();			//hw_config.h
	
	USB_Interrupts_Config();		//hw_config.h
	
	USB_Init();						//usb_lib.h->usb_init.h			//���ڶ˵��������������жϴ���
	
//	PWM_Configuration(TIM2,7200,5000,50);
	PWM_OUT(TIM2,PWM_OUTChannel1,200000,2);	//PWM�趨-20161127�汾
	
	SysTick_Configuration(100);	//ϵͳ���ʱ������72MHz,��λΪuS

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void VirtualCom_Server(void)
{
	//---DMA
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
  {
    /* Send the received data to the PC Host*/
    USART_To_USB_Send_Data();
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
  }
	//---DMA
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    USART_To_USB_Send_Data();
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
//	//----ԭ����
//	if ((count_out != 0) && (bDeviceState == CONFIGURED))
//	{
//		USB_To_USART_Send_Data(&buffer_out[0], count_out);
//		count_out = 0;
//	}
		
//	VirtualCom_LedRun();
//	if (count_out > 0&&VirtualCom_BufferSize==0)
//	{
//		//1��USB�������ݴ��ڷ���
//		VirtualCom_LedRun();
//		VirtualCom_BufferSize=count_out;
//		count_out = 0;
//		VirtualCom_Buffer = (unsigned char*)malloc(sizeof(char) * VirtualCom_BufferSize);
//		memcpy(VirtualCom_Buffer,buffer_out,VirtualCom_BufferSize);
//		USB_To_USART_Send_Data(VirtualCom_Buffer, VirtualCom_BufferSize);		//WOW
//		free(VirtualCom_Buffer);						//�ͷŶ�̬�ռ�
//		memset(VirtualCom_Buffer,0,VirtualCom_BufferSize);							//��ʼ������
//		VirtualCom_BufferSize=0;
//	}
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//  {
//    /* Send the received data to the PC Host*/
//    USART_To_USB_Send_Data();
//  }
		
		//2��USB�������ݴ��ڷ���
//		if(bDeviceState == CONFIGURED)
//		{			
//			USB_To_USART_Send_Data(buffer_out, count_out);		//WOW
//			count_out = 0;
//			memset(VirtualCom_Buffer,0,VirtualCom_BufferSize);							//��ʼ������
//		}
		//3��USB����������ͨ��USB����
		
//		USART_To_USB_Send_Data();
//		USART_TX_DMAFlagClear(USART1);
//		VirtualCom_LedRun();
//	}
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void VirtualCom_LedConfiguration(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	
//	/* ��LED */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* ��LED */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* VBUS */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
////	GPIO_SetBits(GPIOB,GPIO_Pin_0);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_0);			//����LED��
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);		//����LED��
//	GPIO_Configuration_AOD2(GPIOB,GPIO_Pin_9);				//VBUS
	GPIO_Configuration_OPP2(LED_CONNECTED_PORT,LED_CONNECTED_PIN);		//��LED
	GPIO_Configuration_OPP2(LED_RUNNING_PORT,LED_RUNNING_PIN);		//��LED
	GPIO_Configuration_OPP2(GPIOB,GPIO_Pin_10);										//CMSIS��Դ����
	PB10=1;

	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void VirtualCom_LedRun(void)
{
//	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0))
//	{
//		PB0=0;			//����LED��
//		PB10=1;
//	}
//	else
//	{
//		PB0=1;			//����LED��
//		PB10=0;
//	}
//	PB0=~PB0;				//����LED��
//	PB0=~PB10;			//����LED��
	PB0=PB10;
	PB10=~PB10;
}

#endif
