/******************************** User_library *********************************
* �ļ��� 	: STM32_MCO.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2016/01/01
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "STM32_MCO.H"
#include "stm32f10x_gpio.h"
//#include "STM32F10x_BitBand.H"


//RCC_ClocksTypeDef
/*******************************************************************************
*������		:	function
*��������	:	��ȡʱ��
*����			: 
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
void MCO_Initialize(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//--------------------------����PA.8Ϊ����Push-Pullģʽ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//--------------------------ѡ�����ʱ��Դ��
	//------ʱ�ӵ�ѡ����ʱ�����üĴ���(RCC_CFGR)�е�MCO[2:0]λ���ơ�
	//����RCC_MCOΪҪ������ڲ�ʱ�ӣ�
	//RCC_MCO_NoClock --- ��ʱ����� 
	//RCC_MCO_SYSCLK --- ���ϵͳʱ�ӣ�SysCLK�� 
	//RCC_MCO_HSI --- ����ڲ�����8MHz��RC������ʱ�ӣ�HSI�� 
	//RCC_MCO_HSE --- ��������ⲿʱ���źţ�HSE�� 
	//RCC_MCO_PLLCLK_Div2 --- ���PLL��Ƶ��Ķ���Ƶʱ�ӣ�PLLCLK/2�� 	
	RCC_MCOConfig(RCC_MCO_HSE); 
}

