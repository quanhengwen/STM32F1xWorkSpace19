#ifdef BQ26100_TEST
#include "BQ26100_TEST.H"

#include "BQ26100.H"
#include 	"CRC.H"

#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_PWM.H"
#include "STM32_SPI.H"
#include "STM32_SYSTICK.H"
#include "STM32F10x_BitBand.H"


#include	"stdio.h"			//����printf
#include	"string.h"		//����printf
#include	"stdarg.h"		//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�

unsigned char api_bq26100_crc=0;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void BQ26100_TEST_Configuration(void)
{
  SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H	
	PWM_OUT(TIM2,PWM_OUTChannel1,0.5,500);	//PWM�趨-20161127�汾	ռ�ձ�1/1000
	
	api_bq26100_configuration();
	
	SysTick_Configuration(1000);
	
//	while(1)
//	{
//		api_bq26100_test_example_getkey();
//	}
}

/*******************************************************************************
* ������		:
* ��������	:
* ����		:
* ���		:
* ���� 		:
*******************************************************************************/
void BQ26100_TEST_Server(void)
{	
	static unsigned short time = 0;
	unsigned short i = 0;
	unsigned char buffer[128]={0};
	if(time++<500)
		return ;
	time = 0;
	//api_bq26100_read_id(buffer);	//��ID0x33
//	api_bq26100_skip_id();	//0xCC
	//api_bq26100_read_memory(buffer);
	//api_bq26100_read_status(buffer);
	api_bq26100_test_example();
	//api_bq26100_test_example_getkey();

}




#endif