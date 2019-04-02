/******************************** User_library *********************************
* �ļ��� 	: CD4051����8ͨ����·����
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2016/01/01
* ˵��   	: 
********************************************************************************
* CD4051�ǵ���8ͨ����·���أ�����3��ͨ��ѡ�������C��B��A ��һ����ֹ����
* ��INH��C��B��A ����ѡ��ͨ���ţ�INH ��������CD4051�Ƿ���Ч��INH=��1����
* ��INH=Vpʱ������ͨ�����Ͽ�����ֹģ��������;��INH=��O��,��INH=Vss ʱ��
* ͨ����ͨ������ģ�������롣���������źŵķ�Χ��Vop~-Vss(3-15V).
* ����ģ���źŵķ�Χ��LYm(-15~15V)�����ԣ��û����Ը����Լ��������źŷ�Χ
* �����ֿ����źŵ��߼���ƽ��ѡ��VP��VSS��VeE �ĵ�ѹֵ��
* ���磬���Vob=5V,Vss=0V,Y=-5v,��ʱ�����ź�Ϊ0~5V,ģ���źŵķ�ΧΪ-5V~+5V��
*******************************************************************************/



#include "CD4051.H"
#include "STM32_GPIO.H"
#include	"stdlib.h"		//malloc��̬�����ڴ�ռ�
//#include "STM32F10x_BitBand.H"

//CD4511BcdDef* CD4511Bcd;		//BCD��ṹ��
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void CD4051_Delay(u32 time)
{
	while(time--);
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void CD4051_Initialize(stCD4051Def *pInfo)
{
	GPIO_Configuration_OPP50	(pInfo->Port.A_PORT,		pInfo->Port.A_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
	GPIO_Configuration_OPP50	(pInfo->Port.B_PORT,		pInfo->Port.B_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
	GPIO_Configuration_OPP50	(pInfo->Port.C_PORT,		pInfo->Port.C_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
	GPIO_Configuration_OPP50	(pInfo->Port.EN_PORT,		pInfo->Port.EN_Pin);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�2MHz----V20170605
	
	GPIO_SetBits	(pInfo->Port.EN_PORT, pInfo->Port.EN_Pin);			//B/A1
}
/*******************************************************************************
*������			:	CD4511_Clear
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void CD4051_Clear(stCD4051Def *pInfo)		//��������bitA~bitG����͵�ƽ
{
	GPIO_ResetBits(pInfo->Port.A_PORT, pInfo->Port.A_Pin);			//A/A0
	GPIO_SetBits	(pInfo->Port.B_PORT, pInfo->Port.B_Pin);			//B/A1
	GPIO_ResetBits(pInfo->Port.C_PORT, pInfo->Port.C_Pin);			//C/A2
	GPIO_SetBits	(pInfo->Port.EN_PORT, pInfo->Port.EN_Pin);			//D/A3
}
/*******************************************************************************
*������			:	CD4051_WriteChannel
*��������		:	����Ӧͨ���ţ�ֻ�ܴ�һ��
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void CD4051_WriteChannel(stCD4051Def *pInfo,unsigned char nChannel)
{
	GPIO_ResetBits(pInfo->Port.A_PORT, pInfo->Port.A_Pin);			//A/A0
	GPIO_ResetBits(pInfo->Port.B_PORT, pInfo->Port.B_Pin);			//A/A0
	GPIO_ResetBits(pInfo->Port.C_PORT, pInfo->Port.C_Pin);			//A/A0
	
	if(nChannel&0x01)
	{
		GPIO_SetBits(pInfo->Port.A_PORT, pInfo->Port.A_Pin);			//A/A0
	}
	if(nChannel&0x02)
	{
		GPIO_SetBits(pInfo->Port.B_PORT, pInfo->Port.B_Pin);			//A/A0
	}
	if(nChannel&0x04)
	{
		GPIO_SetBits(pInfo->Port.C_PORT, pInfo->Port.C_Pin);			//A/A0
	}
}