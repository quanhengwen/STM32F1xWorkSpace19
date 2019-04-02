/******************************** User_library *********************************
* �ļ��� 	: STM32_TOOLS.H
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


#ifndef __STM32_TOOLS_H
#define __STM32_TOOLS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"



typedef struct _LINK_NODE
{	
	char* DataAddr;								//���ݴ洢��ַ
	struct _LINK_NODE* PrevNode;	//��һ���ڵ�
	struct _LINK_NODE* NextNode;	//��һ���ڵ�	
}LINK_LIST;














unsigned char BCC8(const unsigned char *buffer,unsigned short num);		//���У��
unsigned char CRC8(const unsigned char *buffer);						//ѭ������У��
unsigned char LRC8(const unsigned char *buffer);						//��������У��


#endif






