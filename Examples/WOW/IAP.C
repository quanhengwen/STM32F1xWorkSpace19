/******************************** User_library *********************************
* �ļ��� 	: SPI_FLASH.C
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/04/16
* ˵��   	: 
********************************************************************************
SPI_FLASHʹ�ù��ܲ���
1����Ҫ�궨�� SPI_FLASH
2��ʹ��USB_TEST �����
3����Ҫ�궨��SPI����

*******************************************************************************/
#ifdef IAP							//���������A3987_TEST �˹�����Ч

#include "cortexm3_macro.h"			//__MSR_MSPʹ��

#include "STM32F10x_BitBand.H"
#include "stm32f10x_nvic.h"


#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�
	
#include	"stddef.h"
#include	"stdint.h"



#include "IAP.H"
#include "STM32_PWM.H"
#include "STM32_SYS.H"
#include "STM32_USART.H"
#include "STM32_SYSTICK.H"
#include "STM32_FLASH.H"
#include "STM32_CRC.H"







#define FLASH_IAP_ADDR		0x08000000  //�û������ִ�е�ַ
#define FLASH_APP1_ADDR		0x08006000  //�û������ִ�е�ַ
#define FLASH_USER_ADDR		0x08006000  //�û������ִ�е�ַ
#define FLASH_APP2_ADDR   0x08013000  //�洢���յ����û�����ĵ�ַ
#define FLASH_DATA_ADDR   0x0801E004  //�洢�û����ݵĵ�ַ





#define BufferSize	STM_SECTOR_SIZE/2
u16 RevBuffer[BufferSize]={0};
u16 RxdBuffer[BufferSize]={0};
u16 TxdBuffer[BufferSize]={0};




u32	crc_data	=	0;
u32 SYS_TIME	=	0;
u32 JumpAddress;

/* �������� -----------------------------------------------------------------*/
typedef  void (*pFunction)(void);

pFunction	UserApplication;

void MAIN_Configuration(void);	//
void SET_MSP(u32 addr) ;			//����ջ����ַ---���û��
void Jump_To_Application(void);	//��ת��Ӧ�ó���
void Jump_To_IAP(void);
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void IAP_Configuration(void)
{
	//�ж��û��Ƿ��Ѿ����س�����Ϊ��������´˵�ַ��ջ��ַ��
	//��û����һ��Ļ�����ʹû�����س���Ҳ�����������ܷɡ�
	if (((*(vu32*)FLASH_USER_ADDR) & 0x2FFE0000 ) != 0x20000000)
	{
		MAIN_Configuration();
		USART_DMAPrintf(USART1,"�������\n");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
	}
	else
	{
		USART_DMAPrintf(USART1,"��ת��Ӧ�ó���\n");					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
		Jump_To_Application();	//��ת��Ӧ�ó���
	}
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void IAP_Server(void)		//CRC--TEST
{	
//	u16	Num	=	0;
//	Num	=	USART_ReadBufferIDLE(USART1,(u32*)RevBuffer,(u32*)RxdBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
//	if(Num)
//	{
//		if(RevBuffer[0]	==	UpdataKey2)
//		{
//			STM32_FLASH_Erase(FLASH_USER_ADDR,512);			//����FLASH
//			NVIC_GenerateSystemReset();
//		}
//		else
//		{
//			memcpy(TxdBuffer,RevBuffer,Num);
//			API_USART_DMA_Send(USART1,(u32*)TxdBuffer,Num);	//����DMA���ͳ���
//		}
//	}
//	
	SYS_TIME++;
	if(SYS_TIME>=5000)
	{
		Jump_To_IAP();
//		NVIC_GenerateSystemReset();
//		NVIC_DeInit();		//
//		__disable_irq();
	}
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void MAIN_Configuration(void)
{
	SYS_Configuration();											//ϵͳ���� STM32_SYS.H	
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);
		
	SysTick_Configuration(500);							//ϵͳ���ʱ������72MHz,��λΪuS
	
	USART_DMA_ConfigurationNR	(USART1,115200,(u32*)RxdBuffer,BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	CRC_SetEnable();
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void IAP_CRCTEST(void)		//CRC--TEST
{	
	u16	Num	=	0;
	Num	=	USART_ReadBufferIDLE(USART1,(u32*)RevBuffer,(u32*)RxdBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(Num)
	{
		memcpy(TxdBuffer,RevBuffer,Num);
//		crc_data	=	Get_CalcBlockCrc32((u32*)TxdBuffer,Num/2);
		crc_data	=	CRC16_MODBUS((unsigned char *)TxdBuffer, Num);
		if(Num%2	==	0)
		{
			TxdBuffer[Num/2]	=	crc_data;
		}
		else
		{
			TxdBuffer[Num/2]		=	(TxdBuffer[Num/2]&0x00FF)|((crc_data<<8)&0xFF00);			//��8λ���������ֵĸ�8λ
			TxdBuffer[Num/2+1]	=	(TxdBuffer[Num/2+1]&0xFF00)|((crc_data>>8)&0x00FF);		//��8λ�����������ֵĵ�8λ
		}
		API_USART_DMA_Send(USART1,(u32*)TxdBuffer,Num+2);	//����DMA���ͳ���
	}
}
/*******************************************************************************
*������			:	function
*��������		:	����ջ����ַ---���û��
*����				: addr:ջ����ַ
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void SET_MSP(u32 addr) 
{
	__MSR_MSP(addr);
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void Jump_To_Application(void)
{
//	ApplicationAddr
	if (((*(vu32*)FLASH_USER_ADDR) & 0x2FFE0000 ) == 0x20000000)
	{           
		USART_DMAPrintf(USART1,"Execute user Program\r\n\n");
		//��ת���û�����
		//	asm("CPSID I");
		__disable_irq();
		JumpAddress = *(vu32*) (FLASH_USER_ADDR + 4);
		UserApplication = (pFunction) JumpAddress;

		//��ʼ���û�����Ķ�ջָ��
		SET_MSP(*(vu32*) FLASH_USER_ADDR);
		UserApplication();		//�����û�����
	}
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void Jump_To_IAP(void)
{
	vu32	RAMAddr	=	0;
	RAMAddr	=	*(vu32*)FLASH_IAP_ADDR;
	if ((RAMAddr	& 0x2FFE0000) == 0x20000000)
//	if (((*(vu32*)FLASH_IAP_ADDR) & 0x2FFE0000 ) == 0x20000000)
	{           
		//��ת���û�����
		//	asm("CPSID I");
		__disable_irq();
		JumpAddress = *(vu32*) (FLASH_IAP_ADDR);
		UserApplication = (pFunction) JumpAddress;

		//��ʼ���û�����Ķ�ջָ��
		SET_MSP(*(vu32*) FLASH_IAP_ADDR);
		UserApplication();		//�����û�����
	}
}






















/*******************************************************************************
*������			:	function
*��������		:	����ջ����ַ---���û��
*����				: addr:ջ����ַ
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
//void iap_load_app(u32 appxaddr)
//{
//	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000) //���ջ����ַ�Ƿ�Ϸ�.
//	{
//		jump2app = (iapfun)*(vu32*)(appxaddr+4);//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)���˴��鿴�ж���������֪
//		MSR_MSP(*(vu32*)appxaddr);//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
//		jump2app(); //��ת��APP��ִ�и�λ�жϳ���
//	}
//}
/*******************************************************************************
*������			:	function
*��������		:	����ջ����ַ---���û��
*����				: addr:ջ����ַ
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
//void Jump_Address(void)
//{
//	if (((*(volatile u32*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
//	{
//		test = (*(volatile u32*)ApplicationAddress);
//		JumpAddress = *(volatile u32*) (ApplicationAddress + 4);
//		Jump_To_Application = (pFunction) JumpAddress;
//		__set_MSP(*(volatile u32*) ApplicationAddress);
//		Jump_To_Application();
//	}
//}
































#endif