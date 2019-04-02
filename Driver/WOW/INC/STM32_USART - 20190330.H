/******************************** User_library *********************************
* �ļ��� 	: STM32_SDCard.H
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


#ifndef __STM32_USART_H
#define __STM32_USART_H

//#include	"stdio.h"			//����printf
//#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"

//typedef struct _uLink
//{	
//	unsigned short 	DataLen;			//�洢�����ݳ���
//	unsigned short	Serial;				//�洢�����
//	unsigned char* 	DataAddr;			//���ݶ�̬�洢�ռ�
//	struct _uLink* Next;					//��һ���ڵ�	:
//}uLinkDef;

typedef struct
{
	//USART:485�õ��Ĵ���;���ں�--USART1,USART2,USART3,UART4;//UART5��֧��DMA
	USART_TypeDef* 	USARTx;			//USART1,USART2,USART3,UART4;//UART5
	//RS485_CTL
	GPIO_TypeDef* 	RS485_CTL_PORT;				//GPIOX
	unsigned short 	RS485_CTL_Pin;				//GPIO_Pin_x
}RS485Def;

//#define RS485_TX_EN			(RS485_TypeDef->RS485_CTL_PORT->BSRR		= R61509V_sPinfo->R61509V_sCS_Pin)
//#define RS485_RX_EN			(R61509V_sPinfo->R61509V_sCS_PORT->BRR 		= R61509V_sPinfo->R61509V_sCS_Pin)


//---------------------------����
//*****************USART_DMA��ʼ��


void	USART_IT_Initialize(USART_TypeDef* USARTx);	                    //�����ж�����

void	USART_DMA_ConfigurationNR	(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
void	USART_DMA_ConfigurationIDLEIT	(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize);	//USART_DMA����--�����ж�
void 	USART_DMA_ConfigurationEV	(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�--żУ��
void 	USART_DMA_ConfigurationOD	(USART_TypeDef* USARTx,u32 USART_BaudRate,u16 BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�--��У��

void 	USART_DMA_ConfigurationIT	(USART_TypeDef* USARTx,u32 USART_BaudRate,u32 BufferSize);	//USART_DMA����--�жϷ�ʽ

void	USART_DMA_ConfigurationNRRemap	(USART_TypeDef* USARTx,u32 USART_BaudRate,u32 *RxdBuffer,u32 BufferSize);	//USART_DMA����(ӳ��)--��ѯ��ʽ�������ж�

//*****************���ڽ���
u16	API_USART_ReadBufferIDLE(USART_TypeDef* USARTx,u8 *RevBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ���

//*****************���ڷ���
u16	USART_DMAPrintf		  (USART_TypeDef* USARTx,const char *format,...);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����

u16	API_USART_DMA_Send	(USART_TypeDef* USARTx,u8 *tx_buffer,u16 BufferSize);		//����DMA���ͳ�����������Ѿ����뵽DMA������Buffer��С�����򷵻�0


//---------------------------RS485
//*****************RS485��ʼ��---ʹ��DMA���н��գ���־��ѯ��ʽ
void	RS485_DMA_ConfigurationNR			(RS485Def *pRS485,u32 USART_BaudRate,u16 BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
void	RS485_DMA_ConfigurationNRRemap(RS485Def *pRS485,u32 USART_BaudRate,u32 *RxdBuffer,u32 BufferSize);	//USART_DMA����(ӳ��)--��ѯ��ʽ�������ж�,������Ĭ��Ϊ����״̬
//*****************RS485����
u16		RS485_ReadBufferIDLE			(RS485Def *pRS485,u8 *RevBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer

//*****************RS485����
u16 	RS485_DMAPrintf						(RS485Def *pRS485,const char *format,...);					//�Զ���printf����DMA���ͳ���,��ߵ�ʡ�Ժž��ǿɱ����
u16 	RS485_DMASend							(RS485Def *pRS485,u8 *tx_buffer,u16 BufferSize);	//RS485-DMA���ͳ���


void 	USART_ConfigurationIT(USART_TypeDef* USARTx,u32 USART_BaudRate,u8 PreemptionPriority,u8 SubPriority);	//USART_����---�����жϷ�ʽ
void	USART_Send(USART_TypeDef* USARTx,u8* RevBuffer,u16 Lengh);
unsigned char UART5ReceiveData(unsigned char* RecByte);			//����5������,����0-���жϣ�����1���ж������ݣ������жϣ�������2-���ж��н��յ�����


void  USART_TxServer(USART_TypeDef* USARTx);
void  USART_RxServer(USART_TypeDef* USARTx);		//���ڽ��շ������
void	USART_Process(void);		//���ڷ������	

#endif
