/******************************** User_library *********************************
* 文件名 	: STM32_TOOLS.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
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
	char* DataAddr;								//数据存储地址
	struct _LINK_NODE* PrevNode;	//上一个节点
	struct _LINK_NODE* NextNode;	//下一个节点	
}LINK_LIST;














unsigned char BCC8(const unsigned char *buffer,unsigned short num);		//异或校验
unsigned char CRC8(const unsigned char *buffer);						//循环冗余校验
unsigned char LRC8(const unsigned char *buffer);						//纵向冗余校验


#endif







