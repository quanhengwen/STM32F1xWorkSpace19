#include "STM32_WOW.H"

#include "STM32_USART.H"
#include "STM32_WDG.H"


__weak void Hardware_Configuration(void){}
__weak void Software_Configuration(void){}
__weak void Data_Initialize(void){}
	
__weak void Data_Server(void){}
__weak void SYS_Server(void){}

/*******************************************************************************
* ������	:	WOW_Configuration
* ��������	:	���ú���	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void WOW_Configuration(void)
{
	Hardware_Configuration();
	Software_Configuration();
	Data_Initialize();
//	GPIO_DeInit(GPIOA);
//	GPIO_DeInit(GPIOB);
//	GPIO_DeInit(GPIOC);
//	GPIO_DeInit(GPIOD);
//	GPIO_DeInit(GPIOE);
//	GPIO_DeInit(GPIOF);
//	GPIO_DeInit(GPIOG);
	
//	IWDG_Configuration(5000);	//�������Ź����� 1000ms
	
//	SysTick_Configuration(1000);	//ϵͳ���ʱ������72MHz,��λΪuS
	

	
//***********************************CS5530_DEMO***********************************//
#ifdef SP_FB2	
	SP_FB2_Configuration();
#endif


}













/*******************************************************************************
* ������	:	WOW_Server
* ��������	:	������ 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void WOW_Server(void)
{
//	IWDG_Feed();								//�������Ź�ι��
  MainServer();          //

//***********************************���ֵ�����������***********************************//
#ifdef SP_FB2
	SP_FB2_Server();
#endif



//IWDG_Feed();								//�������Ź�ι��

}
/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void MainServer(void)
{
  IWDG_Feed();			  //�������Ź�ι��
  USART_Process();     //���ڷ������
}



