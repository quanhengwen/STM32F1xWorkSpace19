#ifdef ADC_Temperature
#include "ADC_Temperature.H"


#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_USART.H"
#include "STM32_TIM.H"
#include "STM32_DMA.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_ADC.H"
#include "STM32_DAC.H"
#include "STM32_RCC.H"


#define ADC_TEST_BUFFERSIZE 128

u16	SYSTIME	=	0;
u16 ADCBuffer=0;
float Temperature=0.0;

RCC_ClocksTypeDef RCC_ClocksStatus0;
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void ADC_Temperature_Configuration(void)
{
	SYS_Configuration();											//ϵͳ����
	GPIO_DeInitAll();													//�����е�GPIO�ر�----V20170605
	SysTick_Configuration(1000);						//ϵͳ���ʱ������72MHz,��λΪuS

	USART_DMA_ConfigurationNR	(USART2,115200,ADC_TEST_BUFFERSIZE);	//USART_DMA����--��ѯ��ʽ�������ж�
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);			//PWM�趨-20161127�汾	ռ�ձ�1/1000

	ADC_TempSensorConfiguration((u32*)&ADCBuffer);																									//STM32�ڲ��¶ȴ���������
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void ADC_Temperature_Server(void)
{
	
	SYSTIME++;
	if(SYSTIME>=1000)
	{
		SYSTIME	=	0;
	Get_Clocks(&RCC_ClocksStatus0);
//	if(!USART_TX_DMAFlagClear(USART1))
//	{
		Temperature=Get_ADC_Temperature(ADCBuffer);														//��ȡ�ڲ��¶ȴ������¶�
		USART_DMAPrintf(USART2,"��ǰSTM32�ڲ��¶�Ϊ��%6.2f��,�ⲿ����ʱ��Ƶ��Ϊ��%dHz, %d, %d,  %d, %d, %d\n",Temperature,HSE_Value,RCC_ClocksStatus0.SYSCLK_Frequency,RCC_ClocksStatus0.HCLK_Frequency,RCC_ClocksStatus0.PCLK1_Frequency,RCC_ClocksStatus0.PCLK2_Frequency,RCC_ClocksStatus0.ADCCLK_Frequency);
//	}
	}
}

#endif