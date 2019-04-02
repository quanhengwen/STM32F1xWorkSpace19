#ifdef USART_TEST

#include "USART_TEST.H"

#include "STM32_USART.H"
#include "STM32_DMA.H"
#include "STM32_TIM.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�


#define	BufferSize 500		//DMA1�����С

u32	num_temp=0;
u16	tema=0;

u32 DMASTAST=0;
ErrorStatus status = ERROR;

u8	txflg1=0;	//USART1���ͱ�־
u16	tx1_tcont=0;	//USART1���ͳ�ʱ-��ʱ

u8 rxBuffer1[BufferSize]={0};
u8 txBuffer1[BufferSize]={0};
u8 num=0;
int *pr=NULL;


//u8 itf=0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void Usart_test_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);						//PWM�趨-20161127�汾	
	
	USART_DMA_ConfigurationNR	(USART2,115200,BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
  
//  IWDG_Configuration(1000);			//�������Ź�����---������λms	
  
  SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
  pr=(int*)0x20024300;
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void Usart_test_Server(void)
{	
	u16 Length	=	0;
	
	IWDG_Feed();								//�������Ź�ι��
  USART_Status(USART2);		//����״̬���
	Length	=	USART_ReadBufferIDLE	(USART2,rxBuffer1);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ���
	if(Length)
	{
    memcpy(txBuffer1,rxBuffer1,Length);
    USART_DMASendList(USART2,txBuffer1,Length);	//����DMA���ͳ���
		memset(rxBuffer1,0xFF,Length);
	}
	tx1_tcont++;
	if(tx1_tcont>=2000)
  {
    unsigned short i=0;
		tx1_tcont=0;
    for(i=0;i<1000;i++)
    {
      USART_DMAPrintfList(USART2,"%0.4d�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����\r\n",i);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
    }    
  }
//  USART_TxServer(USART2);
}
void Usart_test(void)
{

}
#endif
