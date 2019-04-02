#ifdef CAN_TEST


#include "CAN_TEST.H"

#include "STM32_TIM.H"
#include "SWITCHID.H"
#include "STM32_WDG.H"
#include "STM32_EXTI.H"
#include "STM32_GPIO.H"
#include "STM32_PWM.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_CAN.H"
#include "STM32F10x_BitBand.H"

u32 SYSTIME	=	0;
u8 CANdata[8]={0};
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void CAN_TEST_Configuration(void)
{
	SYS_Configuration();						//ϵͳ���� STM32_SYS.H	
	CAN_Configuration_NR(100000);			//CAN1����---��־λ��ѯ��ʽ�������ж�--����500K
	CAN_FilterInitConfiguration_StdData(0x05,0x08,0x00);	//CAN�˲�������---��׼����֡ģʽ
	SysTick_Configuration(1000);		//ϵͳ���ʱ������72MHz,��λΪuS----��ʱɨ��PC006V21_Server
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void CAN_TEST_Server(void)
{
	SYSTIME++;
	if(SYSTIME>0xFFFFFF)
	{
		SYSTIME	=	0;
		
	}
	if(SYSTIME%100==0)															//��������---��ת�����ʼ����ʱ��ʼ�ɼ��ź�
	{
		CANdata[0]=0x0F;
		CANdata[1]=0x0E;
		CANdata[2]=0x0A;
		CANdata[3]=0x0B;
		
		CAN_StdTX_DATA(0x08,4,CANdata);			//CANʹ�ñ�׼֡��������---���ʹ��������� 0.5ms���ͼ��
	}
}





#endif