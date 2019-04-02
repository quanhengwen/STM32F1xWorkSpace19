#ifndef __USART_ISP_H
#define __USART_ISP_H

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_spi.h"

//#include	"stdio.h"				//����printf
//#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�
//	
//#include	"stddef.h"
//#include	"stdint.h"

//#include "STM32F10x_BitBand.H"

//#include "STM32_SYSTICK.H"
//#include "STM32_SPI.H"

/*-------------------------ͨ�Ű�ȫ--------------------
��̹��� (PC) ������������ͨ�ž�ͨ�����·�ʽ��֤��
1. У��ͣ����յ��������ֽڿ����������㡣ÿ��ͨ�Ž�β����һ���ֽڣ�У����ֽڣ���
	����ǰ�������ֽ��������Ľ��������������н��յ����ֽڣ������ݰ�����У����ֽڣ��������Ϊ 0x00
2. ���ÿ������������ᷢ��һ���ֽڼ��䲹�루����� = 0x00��
3. UART��������żУ�飨żУ�飩
ÿ�����ݰ����߱����ܣ�ACK Ӧ�𣩻��߱�������NACK Ӧ�𣩣�
--------------------------------------------------------------------------------------*/
#define	ISP_BufferSize	300
	
typedef	enum _ISP_COMMAND			//USART �Ծٳ�������
{
	ISP_COMMAND_Get		=0x00,		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	ISP_COMMAND_GetVS	=0x01,		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	ISP_COMMAND_GetID	=0x02,		//Get ID:��ȡоƬ ID
	ISP_COMMAND_RM		=0x11,		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	ISP_COMMAND_Go		=0x21,		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	ISP_COMMAND_WM		=0x31,		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	ISP_COMMAND_Erase	=0x43,		//Erase:����һ����ȫ�� Flash ҳ��
	ISP_COMMAND_EE		=0x44,		//Extended Erase:ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾����
	ISP_COMMAND_WP		=0x63,		//Write Protect:ʹ��ĳЩ������д����
	ISP_COMMAND_WU		=0x73,		//Write Unprotect:��ֹ���� Flash ������д����
	ISP_COMMAND_RP		=0x82,		//Readout Protect:ʹ�ܶ�����
	ISP_COMMAND_RU		=0x92			//Readout Unprotect:��ֹ������
	//˵����
	//1)������յ��ܾ��������ִ�������ڼ���ִ����Ծٳ�����ᷢ�� NACK �ֽڲ����ؼ������״̬��
	//2)������ - ���� RDP����������ѡ���ֻ��ʹ����һ���޵������Ӽ�������������յ� NACK Ӧ�𣬲��Ҳ�������������á�ȡ�� RDP ���ɼ����������
	//3)Erase (x043) �� Extended Erase (0x44) ��Ϊ��ռ���һ��������֧�� Erase ����� Extended Erase ���������ͬʱ֧�����������
}ISP_COMMAND_TypeDef;

typedef	enum _ISP_Answer	//USART Ӧ��
{
	ISP_ANSWER_ACK		=0x79,		//ACK Ӧ��(�����ܣ�
	ISP_ANSWER_NACK		=0x1F			//NACK Ӧ�𣨲����ܻ��߲�ʶ��--����)
}ISP_ANSWER_TypeDef;
typedef	enum _ISP_FUN	//USART Ӧ��
{
	ISP_IDLE			=0xA0,	//ISP״̬Ϊ���У�����RESET�ӿ�״̬��BOOT0�ӿ�״̬�����л�ΪMASTER����SLAVE
	ISP_SLAVE,						//ISP��Ϊ�ӻ�
	ISP_MASTER,						//ISP��Ϊ����--������������
	ISP_WaitDeliver				//�ȴ��ͷŴӻ�
}ISP_FUN_TypeDef;

//typedef	enum _ISP_STEPS			//ISPִ�в���
//{
//	Step_Start				=	0,			//��ʼ���裬��ǰΪ����
//	Step_Get					=	1,			//�ȴ�д������ɣ�ֻ��д������ɺ�ſ��Խ���д���߲�������
//	Step_GetVS				=	2,			//дʹ��
//	Step_GetID				=	3,			//д״̬��		��ȡ״̬Ϊ1��ʾ����ִ��д�����������ʾд�����
//	Step_RM						=	4,			//��
//	Step_WaitRaddr		=	5,			//�ȴ�������Ҫ��ȡ�ĵ�ַ
//	Step_WaitRlen			=	6,			//�ȴ�������Ҫ��ȡ�����ݳ���
//	Step_ReadData			=	7,			//�ȴ�������Ҫ��ȡ�����ݳ���
//	Step_SendRData		=	8,			//���Ͷ�ȡ������
//	Step_Erase				=	9,			//��������
//	Step_WaitWaddr		=	10,			//�ȴ�������Ҫд���������ʼ��ַ
//	Step_WriteData		=	11,			//����д������
//	Step_EraseAll			=	12,			//ȫ������
//	Step_ErasePage		=	13,			//����ҳ
//	Step_WaitErasePageA	=	14,		//WaitErasePageAҪ������ҳ��
//	Step_WaitWlen			=	15,			//�ȴ�������Ҫд������ݳ���
//	Step_WaitWData		=	16			//�ȴ�����Ҫд�������
//}ISP_STEPS_TypeDef;

typedef enum _ISP_SLAVE_STATUS			//ISP״̬��־---�����ڴӻ�״̬ʱ
{
	ISP_STATUS_IDLE=0x00,				//ISP����״̬�����Զ�д
	ISP_STATUS_UnConnect,				//ISPδ���ӣ���Ϊ�ӻ�)
	ISP_STATUS_WaitConnect,			//ISP�ȴ����ӣ���Ϊ�ӻ�)
	ISP_STATUS_Connectting,			//ISP�������ӣ���Ϊ�ӻ�)---ִ�и�ʽ��
	ISP_STATUS_Connectted,			//ISP�Ѿ����ӣ���Ϊ�ӻ�)
	ISP_STATUS_WaitCommand,			//ISP�ȴ������Ϊ�ӻ�)
	ISP_STATUS_WaitEraseAddr,		//ISP�ȴ���������ַ
	ISP_STATUS_WaitErase,				//ISP�ȴ�����
	ISP_STATUS_Eraseing,				//ISP���ڲ���
	ISP_STATUS_WaitErased,			//ISP�ȴ��������
	ISP_STATUS_WaitWAddr,				//ISP�ȴ���д���ַ
	ISP_STATUS_WaitWData,				//ISP�ȴ����մ�д������
	ISP_STATUS_WaitWrite,				//ISP�ȴ�д����
	ISP_STATUS_WaitWrited,			//ISP�ȴ�д�����
	ISP_STATUS_WaitReadAddr,		//ISP�ȴ�����ȡ��ַ
	ISP_STATUS_WaitReadData,		//ISP�ȴ������ݲ���
	ISP_STATUS_WaitLengh,				//ISP�ȴ�����ȡ����
	ISP_STATUS_WaitSData,				//ISP�ȴ���������
	ISP_STATUS_WaitGoAddr,				//�ȴ�����ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó�������ַ//Go �������ڴ�Ӧ�ó���ָ���ĵ�ַ��ʼִ�������صĴ���������κδ���
	ISP_STATUS_WaitReset				//ISP�ȴ���������
}ISP_SLAVE_STATUS_TypeDef;
typedef enum _ISP_MASTER_STATUS		//ISP״̬��־---����������״̬ʱ
{
	ISP_MSTATUS_IDLE=0x1E,			//ISP��������״̬----��ʼдǰ����д��ɺ�״̬
	ISP_MSTATUS_ResetDevice,		//ISP����׼������----��λ�ӻ�
	ISP_MSTATUS_WriteConnect,		//ISP����׼������----��λ����ӻ�
//	ISP_MSTATUS_Connectting,	//ISP������ӻ����ڽ�������
	ISP_MSTATUS_Connectted,			//ISP������ӻ��Ѿ��������
	ISP_MSTATUS_WriteUnprotect,	//��ֹ����Flash������д����
	ISP_MSTATUS_WaitUnprotect,	//�ȴ�����Flash����д�����������
	ISP_MSTATUS_FlashUnprotected,	//�ѽ�ֹ����Flash������д���������Խ���д����,��һ������д����
	ISP_MSTATUS_GetFirmwareInf,	//��ȡ�̼���Ϣ
	ISP_MSTATUS_WaitFirmwareInf,	//�ȴ��̼���Ϣ
	ISP_MSTATUS_GetedFirmwareInf,	//�ѻ�ȡ�̼���Ϣ
	ISP_MSTATUS_WriteGet,				//ISP������ȡ�ӻ��豸��Ϣ---��ҪΪЭ����Ϣ
	ISP_MSTATUS_WaitAck,				//ISP�����ȴ��ӻ�Ӧ��
	ISP_MSTATUS_WriteEraseCMD,	//ISP����д��������---��ҪӦ��
	ISP_MSTATUS_WriteEraseConf,	//ISP����д��������---��Ƭ����(0xFF 0x00)����ѡ���ַ������Ȼ��ȴ�������ɣ���ɺ�ӻ�����һ��ACKӦ��
	ISP_MSTATUS_EraseDone,	//ISP����д��������---��Ƭ����(0xFF 0x00)����ѡ���ַ������Ȼ��ȴ�������ɣ���ɺ�ӻ�����һ��ACKӦ��
	ISP_MSTATUS_ReadFirmware,		//���̼�
	ISP_MSTATUS_WaitFirmware,		//�ȴ��̼�
	ISP_MSTATUS_GettedFirmware,	//�Ѿ���ȡ�̼�
	ISP_MSTATUS_WriteWM,				//ISP��������д���Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash---��ҪӦ��
	ISP_MSTATUS_WriteAddr,			//ISP��������Ҫд�����ݵ���ʼ��ַ---��ҪӦ��
	ISP_MSTATUS_WriteData,			//ISP��������ִ�в�������
	ISP_MSTATUS_WriteRM,				//ISP�������Ͷ�����--��ҪΪУ��д������ݣ�Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	ISP_MSTATUS_WriteRMLengh,		//ISP����������Ҫ��ȡ�����ݳ���
	ISP_MSTATUS_Verify,					//����У�飬����д��ӻ���У������
	ISP_MSTATUS_WriteGo,				//ISP��������Go����---ִ����Goȫ������������ӻ�����
	ISP_MSTATUS_WriteGoAddr,		//д���е�ַ
	ISP_MSTATUS_ResetDevice2,		//ISP�������ӻ�д��ȫ�����ݺ�Դӻ����и�λ����
	ISP_MSTATUS_WaitDeliver			//�ȴ��ͷŴӻ�
}ISP_MASTER_STATUS_TypeDef;

typedef	struct _ISP_Version				//USART �Ծٳ���汾---����ͨ����λ�ӻ����ȡ
{
	unsigned char ISP_VS				;		//�Ծٳ���汾��0 < �汾 < 255����ʾ����0x10 = �汾 1.0
	unsigned char ISP_VS_Get		;		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	unsigned char ISP_VS_GetVS	;		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	unsigned char ISP_VS_GetID	;		//Get ID:��ȡоƬ ID
	unsigned char ISP_VS_RM			;		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	unsigned char ISP_VS_Go			;		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	unsigned char ISP_VS_WM			;		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	unsigned char ISP_VS_Erase	;		//Erase:����һ����ȫ�� Flash ҳ��
	unsigned char ISP_VS_WP			;		//Write Protect:ʹ��ĳЩ������д����
	unsigned char ISP_VS_WU			;		//Write Unprotect:��ֹ���� Flash ������д����
	unsigned char ISP_VS_RP			;		//Readout Protect:ʹ�ܶ�����
	unsigned char ISP_VS_RU			;		//Readout Unprotect:��ֹ������
}ISP_VERSION_TypeDef;

typedef	struct _ISP_DATA				//ISP���ݴ洢����
{
	unsigned int OffsetAddr				;		//д�ӻ�ʱ�ĵ�ַƫ��
	unsigned int StartAddr				;		//��ʼ��ַ
	unsigned int GoAddr						;		//��λ������ʼ��ַ
	unsigned int FirmwareLen			;		//�̼�����---��ҪISP���ص�������
	
	unsigned int ReceivedLen			;		//���ڽ��յ����ݸ���
	unsigned int USARTSendLen			;		//��Ҫ�����ڷ��͵����ݸ���---//����⵽SendLen��Ϊ0ʱ�Զ���������
	
	unsigned int FLASHNumToSave		;		//��Ҫ��FLASH�ﱣ������ݸ���---���˲�Ϊ0ʱSPI-FLASH�Զ��洢����
	unsigned int FLASHNumToRead		;		//��Ҫ��FLASH���ȡ�����ݸ���---���˲�Ϊ0ʱ��SPI-FLASH�Զ�ͨ��WriteAddr��ַ��SPI-FLASH��ȡ����
	
	unsigned int ReadAddr					;		//��������ʼ��ַ
	unsigned int ReadLen					;		//��Ҫ��ȡ�ĳ���
	
	unsigned int WriteAddr				;		//Ҫд��Flash��������ʼ��ַ--ISP�ӿ�
	unsigned int WriteLen					;		//Ҫд��Flash�����ݳ��ȣ��ֽ���)--ISP�ӿ�
	

	unsigned int 	SumHaveReceived		;		//��Ϊ�ӻ�ʱ�ܹ����յ������ݸ���
	unsigned int 	SumHaveWritten		;		//�����ܹ����ӻ�д������ݸ���
		
	unsigned char Command[2]				;		//�Ծٳ���������У����
	
	unsigned char ISP_RxBuffer[ISP_BufferSize]	;		//���ջ�����
	unsigned char ISP_RvBuffer[ISP_BufferSize]	;		//���ջ�����--������
	unsigned char ISP_TxBuffer[ISP_BufferSize]	;		//���ͻ�����
	unsigned char ISP_TvBuffer[ISP_BufferSize]	;		//���ͻ�����--������

}ISP_DATA_TypeDef;

typedef	struct _ISP_Conf					//USART ���߱�������ļ�
{
	USART_TypeDef* 	USARTx															;		//USART1,USART2,USART3,UART4;//UART5
	//RESET���ƽ�
	GPIO_TypeDef* 	RESET_CTL_PORT;				//GPIOX
	unsigned short 	RESET_CTL_Pin;				//GPIO_Pin_x
	//BOOT0���ƽ�
	GPIO_TypeDef* 	BOOT0_CTL_PORT;				//GPIOX
	unsigned short 	BOOT0_CTL_Pin;				//GPIO_Pin_x
	
	ISP_SLAVE_STATUS_TypeDef	ISP_SLAVE_STATUS					;		//ISP״̬��־---�����ڴӻ�״̬ʱ
	ISP_SLAVE_STATUS_TypeDef	SlaveLastStatus					;			//�洢�ӻ���һ��״̬��������һ��״̬������һ������
	
	ISP_MASTER_STATUS_TypeDef	ISP_MASTER_STATUS					;		//ISP״̬��־---����������״̬ʱ
	ISP_MASTER_STATUS_TypeDef	MasterLastStatus					;		//�洢������һ��״̬��������һ��״̬�ȵ��ӻ�Ӧ��ʱ������һ������
	
	ISP_VERSION_TypeDef				SlaveVersion							;		//�ӻ��Ծٳ���汾��Ϣ
	ISP_DATA_TypeDef					ISP_DATA									;		//ISP���ݴ洢����---�ӻ�
	ISP_DATA_TypeDef					ISP_MDATA									;		//ISP���ݴ洢����---����
	ISP_FUN_TypeDef						ISP_FUN										;		//ISPģ�鹤��״̬������-��¼/�ӻ�-������������)
	
	unsigned int 	OverRunTime														;		//��ʱʱ��
	unsigned int 	TimeCount															;		//��ʱʱ��
	unsigned char Connected															;		//����״̬--�ӻ����¹̼�ʱʹ�ã����Connected==0����ʾδ���ӣ���Ҫ��ʽ�������Connected==1����ʾ�Ѿ�����	

}ISP_Conf_TypeDef;


void Usart_ISP_Cof(ISP_Conf_TypeDef *ISP_Conf);
void Usart_ISP_ConfigurationNR(ISP_Conf_TypeDef *ISP_Conf,u32 USART_BaudRate,u32 *RXDBuffer,u32 BufferSize);	//
void Usart_ISP_Process(ISP_Conf_TypeDef *ISP_Conf);

void Usart_ISP_SlaveProcess(ISP_Conf_TypeDef *ISP_Conf);		//ģ����Ϊ�ӻ�ʱ�Ĵ�������
void Usart_ISP_MasterProcess(ISP_Conf_TypeDef *ISP_Conf);		//ģ����Ϊ����ʱ�Ĵ�������
void Usart_ISP_CheckFun(ISP_Conf_TypeDef *ISP_Conf);				//���ISPģ�鹤��ģʽ---����ʱ���

void Usart_ISP_ACK(ISP_Conf_TypeDef *ISP_Conf);	//ISPӦ��
void Usart_ISP_NACK(ISP_Conf_TypeDef *ISP_Conf);		//ISP��Ӧ��

void Usart_ISP_CommandRead(ISP_Conf_TypeDef *ISP_Conf);			//���ڽ����������->�ӻ�)
void Usart_ISP_GetAddr(ISP_Conf_TypeDef *ISP_Conf);					//ISP��ȡд������ʼ��ַ(����->�ӻ�)

void Usart_ISP_GetLengh(ISP_Conf_TypeDef *ISP_Conf);				//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)��������ʱ�������ַ���ٴ������ȡ�����ݳ���

void Usart_ISP_ReadMemory(ISP_Conf_TypeDef *ISP_Conf);			//ISP�����ݣ��ӻ�->������
void Usart_ISP_WriteMemory(ISP_Conf_TypeDef *ISP_Conf);			//ISPд���ݣ�����->�ӻ���д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
void Usart_ISP_SendBuffer(ISP_Conf_TypeDef *ISP_Conf);			//ISP�ϴ����ݣ��ӻ�->����)
void Usart_ISP_Erase(ISP_Conf_TypeDef *ISP_Conf);						//ISP�������������յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ�������ȴ�������ɣ���ɺ�Ӧ��
u8 Usart_ISP_GetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf);	//���شӻ�״ֵ̬
void Usart_ISP_SetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf,ISP_SLAVE_STATUS_TypeDef Status);	//���ôӻ�״̬
//------------------------------------�ӻ�ר�к���

//------------------------------------����ר�к���
void Usart_MISP_StatusProcess(ISP_Conf_TypeDef *ISP_Conf);			//�����ӻ��豸ʹ�ӻ�����
u8 Usart_MISP_GetStatus(ISP_Conf_TypeDef *ISP_Conf);	//���شӻ�״ֵ̬
void Usart_MISP_GetFirmwareInf(ISP_Conf_TypeDef *ISP_Conf);			//��Flash�л�ȡ�̼���Ϣ
void Usart_MISP_GetDevice(ISP_Conf_TypeDef *ISP_Conf);					//��ȡ�ӻ�оƬ��Ϣ
void Usart_MISP_WriteUnprotect(ISP_Conf_TypeDef *ISP_Conf);			//Write Unprotect�������ڽ�ֹ����Flash������д������
void Usart_MISP_Connect(ISP_Conf_TypeDef *ISP_Conf);						//�������Ӵӻ�����
void Usart_MISP_ResetDevice(ISP_Conf_TypeDef *ISP_Conf);				//��λ�ӻ�
void Usart_MISP_SetDevice(ISP_Conf_TypeDef *ISP_Conf);					//�����ӻ�����BOOT0ʹ�ӻ���������״̬
void Usart_MISP_RunDevice(ISP_Conf_TypeDef *ISP_Conf);					//�����ӻ��豸ʹ�ӻ���������
void Usart_MISP_CommandSend(ISP_Conf_TypeDef *ISP_Conf,unsigned char Command);	//���ڱ�̷����������
bool Usart_MISP_ReadAck(ISP_Conf_TypeDef *ISP_Conf);						//������ȡ�ӻ�Ӧ��:��Ӧ�𷵻�TRUE�����򷵻�FALSE


//------------------------------------��������
void Usart_ISP_Send(ISP_Conf_TypeDef *ISP_Conf);						//���ͺ��������ӻ�����Ӧ�𡢷��͵�ַ�����������������
void Usart_ISP_Reset(ISP_Conf_TypeDef *ISP_Conf);						//���ñ����---�ָ����в���ΪĬ��ֵ






#define	BSP_BufferSize	300

typedef	struct _BSP_Port					//USART�˿ڽṹ��
{
	//USART�ӿں�
	USART_TypeDef* 	USARTx															;		//USART1,USART2,USART3,UART4;//UART5
	//RESET���ƽ�
	GPIO_TypeDef* 	RESET_CTL_PORT;				//GPIOX
	unsigned short 	RESET_CTL_Pin;				//GPIO_Pin_x
	//BOOT0���ƽ�
	GPIO_TypeDef* 	BOOT0_CTL_PORT;				//GPIOX
	unsigned short 	BOOT0_CTL_Pin;				//GPIO_Pin_x
}BSP_Port_TypeDef;


typedef	enum _BSP_COMMAND			//USART �Ծٳ�������
{
	BSP_COMMAND_Get		=0x00,		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	BSP_COMMAND_GetVR	=0x01,		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	BSP_COMMAND_GetID	=0x02,		//Get ID:��ȡоƬ ID
	BSP_COMMAND_RM		=0x11,		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	BSP_COMMAND_Go		=0x21,		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	BSP_COMMAND_WM		=0x31,		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	BSP_COMMAND_Erase	=0x43,		//Erase:����һ����ȫ�� Flash ҳ��
	BSP_COMMAND_EE		=0x44,		//Extended Erase:ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾����
	BSP_COMMAND_WP		=0x63,		//Write Protect:ʹ��ĳЩ������д����
	BSP_COMMAND_WU		=0x73,		//Write Unprotect:��ֹ���� Flash ������д����
	BSP_COMMAND_RP		=0x82,		//Readout Protect:ʹ�ܶ�����
	BSP_COMMAND_RU		=0x92			//Readout Unprotect:��ֹ������
	//˵����
	//1)������յ��ܾ��������ִ�������ڼ���ִ����Ծٳ�����ᷢ�� NACK �ֽڲ����ؼ������״̬��
	//2)������ - ���� RDP����������ѡ���ֻ��ʹ����һ���޵������Ӽ�������������յ� NACK Ӧ�𣬲��Ҳ�������������á�ȡ�� RDP ���ɼ����������
	//3)Erase (x043) �� Extended Erase (0x44) ��Ϊ��ռ���һ��������֧�� Erase ����� Extended Erase ���������ͬʱ֧�����������
}BSP_COMMAND_TypeDef;

typedef	struct _BSP_Version				//USART �Ծٳ���汾---����ͨ����λ�ӻ����ȡ
{
	unsigned char BSP_VS				;		//�Ծٳ���汾��0 < �汾 < 255����ʾ����0x10 = �汾 1.0
	unsigned char BSP_VS_Get		;		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	unsigned char BSP_VS_GetVS	;		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	unsigned char BSP_VS_GetID	;		//Get ID:��ȡоƬ ID
	unsigned char BSP_VS_RM			;		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	unsigned char BSP_VS_Go			;		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	unsigned char BSP_VS_WM			;		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	unsigned char BSP_VS_Erase	;		//Erase:����һ����ȫ�� Flash ҳ��
	unsigned char BSP_VS_WP			;		//Write Protect:ʹ��ĳЩ������д����
	unsigned char BSP_VS_WU			;		//Write Unprotect:��ֹ���� Flash ������д����
	unsigned char BSP_VS_RP			;		//Readout Protect:ʹ�ܶ�����
	unsigned char BSP_VS_RU			;		//Readout Unprotect:��ֹ������
}BSP_VERSION_TypeDef;

typedef	enum _BSP_Answer	//USART Ӧ��
{
	BSP_ACK		=0x79,		//ACK Ӧ��(�����ܣ�
	BSP_NACK		=0x1F			//NACK Ӧ�𣨲����ܻ��߲�ʶ��--����)
}BSP_ANSWER_TypeDef;
typedef	enum _BSP_FUN	//USART Ӧ��
{
	BSP_IDLE			=0xA0,	//ISP״̬Ϊ���У�����RESET�ӿ�״̬��BOOT0�ӿ�״̬�����л�ΪMASTER����SLAVE
	BSP_SLAVE,						//ISP��Ϊ�ӻ�
	BSP_MASTER,						//ISP��Ϊ����--������������
	BSP_WaitDeliver				//�ȴ��ͷŴӻ�
}BSP_FUN_TypeDef;
typedef enum _WORK_STEEPS			//ISP״̬��־---�����ڴӻ�״̬ʱ
{
	WORK_STEEPS_IDLE=0x00,		//ISP����״̬�����Զ�д
	WORK_STEEPS_RESET,				//��λ
	WORK_STEEPS_VS,						//�Ծٳ���汾��0 < �汾 < 255����ʾ����0x10 = �汾 1.0
	WORK_STEEPS_GET,					//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	WORK_STEEPS_GETVS,				//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	WORK_STEEPS_GETID,				//Get ID:��ȡоƬ ID
	WORK_STEEPS_RM,						//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�	
	WORK_STEEPS_WM,						//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	WORK_STEEPS_ERASE,				//Erase:����һ����ȫ�� Flash ҳ��
	WORK_STEEPS_EERASE,				//ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾��
	WORK_STEEPS_WP,						//Write Protect:ʹ��ĳЩ������д����
	WORK_STEEPS_WU,						//Write Unprotect:��ֹ���� Flash ������д����
	WORK_STEEPS_RP,						//Readout Protect:ʹ�ܶ�����
	WORK_STEEPS_RU,						//Readout Unprotect:��ֹ������
	WORK_STEEPS_GO						//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
}WORK_STEEPS_TypeDef;
typedef enum _BSPS_STATUS			//ISP״̬��־---�����ڴӻ�״̬ʱ
{
	BSP_STATUS_IDLE=0x3E,				//ISP����״̬�����Զ�д
	BSP_STATUS_UnConnect,				//ISPδ���ӣ���Ϊ�ӻ�)
	BSP_STATUS_WaitConnect,			//ISP�ȴ����ӣ���Ϊ�ӻ�)
	BSP_STATUS_Connectting,			//ISP�������ӣ���Ϊ�ӻ�)---ִ�и�ʽ��
	BSP_STATUS_Connectted,			//ISP�Ѿ����ӣ���Ϊ�ӻ�)
	BSP_STATUS_WaitCommand,			//ISP�ȴ������Ϊ�ӻ�)
	BSP_STATUS_WaitEraseAddr,		//ISP�ȴ���������ַ
	BSP_STATUS_WaitErase,				//ISP�ȴ�����
	BSP_STATUS_Eraseing,				//ISP���ڲ���
	BSP_STATUS_WaitErased,			//ISP�ȴ��������
	BSP_STATUS_WaitWAddr,				//ISP�ȴ���д���ַ
	BSP_STATUS_WaitWData,				//ISP�ȴ����մ�д������
	BSP_STATUS_WaitWrite,				//ISP�ȴ�д����
	BSP_STATUS_WaitWrited,			//ISP�ȴ�д�����
	BSP_STATUS_WaitReadAddr,		//ISP�ȴ�����ȡ��ַ
	BSP_STATUS_WaitReadData,		//ISP�ȴ������ݲ���
	BSP_STATUS_WaitLengh,				//ISP�ȴ�����ȡ����
	BSP_STATUS_WaitSData,				//ISP�ȴ���������
	BSP_STATUS_WaitGoAddr,				//�ȴ�����ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó�������ַ//Go �������ڴ�Ӧ�ó���ָ���ĵ�ַ��ʼִ�������صĴ���������κδ���
	BSP_STATUS_WaitReset				//ISP�ȴ���������
}BSPS_STATUS_TypeDef;
typedef enum _BSP_MASTER_STATUS		//BSP״̬��־---����������״̬ʱ
{
	BSP_MSTATUS_IDLE=0x5E,			//BSP��������״̬----��ʼдǰ����д��ɺ�״̬
	BSP_MSTATUS_ResetDevice,		//BSP����׼������----��λ�ӻ�
	BSP_MSTATUS_WriteConnect,		//BSP����׼������----��λ����ӻ�
	BSP_MSTATUS_WriteGet,				//Get����,��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	BSP_MSTATUS_WriteGetVR,			//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	BSP_MSTATUS_WriteGetID,			//Get ID:��ȡоƬ ID
	BSP_MSTATUS_WriteRM,				//BSP�������Ͷ�����--��ҪΪУ��д������ݿɿ�����д��ǰ������޶�д������Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	BSP_MSTATUS_WriteErase,			//BSP����д��������---��ҪӦ��
	BSP_MSTATUS_WriteWM,				//BSP��������д���Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash---��ҪӦ��
	BSP_MSTATUS_WriteAddr,			//BSP��������Ҫд�����ݵ���ʼ��ַ---��ҪӦ��
	BSP_MSTATUS_WriteData,			//BSP����д���ݣ���� 256 ���ֽ�
	BSP_MSTATUS_WriteGo,				//BSP��������Go����---ִ����Goȫ������������ӻ�����
	BSP_MSTATUS_WriteGoAddr,		//д���е�ַ
	BSP_MSTATUS_WriteUnprotect,	//��ֹ����Flash������д����
	BSP_MSTATUS_WriteEraseConf,		//BSP����д��������---��Ƭ����(0xFF 0x00)����ѡ���ַ������Ȼ��ȴ�������ɣ���ɺ�ӻ�����һ��ACKӦ��
	BSP_MSTATUS_WriteRMLengh,			//BSP����������Ҫ��ȡ�����ݳ���
	
	BSP_MSTATUS_WaitUnprotect,		//�ȴ�����Flash����д�����������
	BSP_MSTATUS_FlashUnprotected,	//�ѽ�ֹ����Flash������д���������Խ���д����,��һ������д����
	BSP_MSTATUS_GetFirmwareInf,		//��ȡ�̼���Ϣ
	BSP_MSTATUS_WaitFirmwareInf,	//�ȴ��̼���Ϣ
	BSP_MSTATUS_GetedFirmwareInf,	//�ѻ�ȡ�̼���Ϣ
	BSP_MSTATUS_WaitAck,					//BSP�����ȴ��ӻ�Ӧ��	
	BSP_MSTATUS_EraseDone,				//BSP����д��������---��Ƭ����(0xFF 0x00)����ѡ���ַ������Ȼ��ȴ�������ɣ���ɺ�ӻ�����һ��ACKӦ��
	BSP_MSTATUS_ReadFirmware,			//���̼�
	BSP_MSTATUS_WaitFirmware,			//�ȴ��̼�
	BSP_MSTATUS_GettedFirmware,		//�Ѿ���ȡ�̼�	
	BSP_MSTATUS_Verify,						//����У�飬����д��ӻ���У������	
	BSP_MSTATUS_StartDevice,			//BSP�������ӻ�д��ȫ�����ݺ�Դӻ����и�λ����
	BSP_MSTATUS_ERROR,						//����---���������Ӵ�����Ϣ
	BSP_MSTATUS_TIMEOUT,					//����---��ʱ
	BSP_MSTATUS_WaitDeliver,			//�ȴ��ͷŴӻ�
	BSP_MSTATUS_Connectted				//BSP������ӻ��Ѿ��������
}BSPM_STATUS_TypeDef;


//------------------ģ����Ϊ����ʱ����Ϣ�ṹ��
typedef	struct _BSPM_Info					
{
	BSPM_STATUS_TypeDef		BSP_MASTER_STATUS		;		//ISP״̬��־---����������״̬ʱ
	BSPM_STATUS_TypeDef		MasterLastStatus		;		//�洢������һ��״̬��������һ��״̬�ȵ��ӻ�Ӧ��ʱ������һ������
	
	BSP_VERSION_TypeDef		SlaveVersion				;		//�ӻ��Ծٳ���汾��Ϣ
	
	WORK_STEEPS_TypeDef		WORK_STEEPS					;
	
	BSP_ANSWER_TypeDef		ACK;										//�ӻ���Ӧ��״̬	BSP_ACK-0x79,BSP_NACK-0x1F
	BSP_ANSWER_TypeDef		ACK2;										//�ӻ���Ӧ��״̬2	BSP_ACK-0x79,BSP_NACK-0x1F
	
}BSPM_Info_TypeDef;
//------------------ģ����Ϊ�ӻ�ʱ����Ϣ�ṹ��
typedef	struct _BSPS_Info
{
	BSPS_STATUS_TypeDef		BSP_SLAVE_STATUS		;		//ISP״̬��־---�����ڴӻ�״̬ʱ
	BSPS_STATUS_TypeDef		SlaveLastStatus			;			//�洢�ӻ���һ��״̬��������һ��״̬������һ������
}BSPS_Info_TypeDef;
//------------------��Ϊ�������ߴӻ�ʱ�Ĺ������ݽṹ��
typedef	struct _BSPM_Public					
{
	unsigned int 	OverRunTime				;		//��ʱʱ��
	unsigned int 	TimeCount					;		//��ʱʱ��
	unsigned int	RetryTimes				;		//���Դ���
	unsigned char Connected					;		//����״̬--�ӻ����¹̼�ʱʹ�ã����Connected==0����ʾδ���ӣ���Ҫ��ʽ�������Connected==1����ʾ�Ѿ�����
	
	unsigned int 	OffsetAddr				;		//д�ӻ�ʱ�ĵ�ַƫ��
	unsigned int 	StartAddr					;		//��ʼ��ַ
	unsigned int 	StepLen						;		//ÿ��д����ֽڳ�������ַ������
	unsigned int 	GoAddr						;		//��λ������ʼ��ַ
	unsigned int 	FirmwareLen				;		//�̼�����---��ҪISP���ص�������	
	
	unsigned int 	FLASHNumToSave		;		//��Ҫ��FLASH�ﱣ������ݸ���---���˲�Ϊ0ʱSPI-FLASH�Զ��洢����
	unsigned int 	FLASHNumToRead		;		//��Ҫ��FLASH���ȡ�����ݸ���---���˲�Ϊ0ʱ��SPI-FLASH�Զ�ͨ��WriteAddr��ַ��SPI-FLASH��ȡ����
	
	unsigned int 	ReadAddr					;		//��������ʼ��ַ
	unsigned int 	ReadLen						;		//��Ҫ��ȡ�ĳ���
	
	unsigned int 	WriteAddr					;		//Ҫд��Flash��������ʼ��ַ--ISP�ӿ�
	unsigned int 	WriteLen					;		//Ҫд��Flash�����ݳ��ȣ��ֽ���)--ISP�ӿ�
	
	unsigned int 	ReceivedLen				;		//���ڽ��յ����ݸ���
	unsigned int	USARTSendLen			;		//��Ҫ�����ڷ��͵����ݸ���---//����⵽SendLen��Ϊ0ʱ�Զ���������	

	unsigned int 	SumHaveReceived		;		//��Ϊ�ӻ�ʱ�ܹ����յ������ݸ���
	unsigned int 	SumHaveWritten		;		//�����ܹ����ӻ�д������ݸ���
		
	unsigned char Command[2]				;		//�Ծٳ���������У����
	
	unsigned char BSP_RxBuffer[BSP_BufferSize]	;		//���ջ�����
	unsigned char BSP_RvBuffer[BSP_BufferSize]	;		//���ջ�����--������
	unsigned char BSP_TxBuffer[BSP_BufferSize]	;		//���ͻ�����
	unsigned char BSP_TvBuffer[BSP_BufferSize]	;		//���ͻ�����--������	
	unsigned char BSP_FlashBuffer[BSP_BufferSize]	;		//��ȡ��FLASH����--������	
	
}BSPM_Public_TypeDef;

typedef	struct _BSP_Conf					//USART ���߱�������ļ�
{
	BSP_Port_TypeDef		BSP_Port			;			//USART�˿ڽṹ��-----�û�Ψһ��Ҫ���õ�����
	
	BSPM_Info_TypeDef		BSPM_Info			;			//ģ����Ϊ����ʱ����Ϣ�ṹ��	
	
	BSPS_Info_TypeDef		BSPS_Info			;			//ģ����Ϊ�ӻ�ʱ����Ϣ�ṹ��
	
	BSPM_Public_TypeDef	BSP_DATA			;			//��Ϊ�������ߴӻ�ʱ�Ĺ������ݽṹ��
	
	BSP_FUN_TypeDef			BSP_FUN				;			//ISPģ�鹤��״̬������-��¼/�ӻ�-������������)
	
	WORK_STEEPS_TypeDef	STEEPS				;			//ִ�н׶�

}BSP_Conf_TypeDef;










//---------------------��������
void BSP_Configuration(BSP_Conf_TypeDef *BSP_Conf);									//���ú���
void BSP_Process(BSP_Conf_TypeDef *BSP_Conf);								//�¼���������
void BSP_Reset(BSP_Conf_TypeDef *BSP_Conf);									//���ñ����---�ָ����в���ΪĬ��ֵ
void BSP_SendBuffer(BSP_Conf_TypeDef *BSP_Conf);						//ͨ�����ڷ������ݣ�����⵽USARTSendLen��Ϊ0ʱ�Զ���BSP_TvBuffer����USARTSendLen�������ݵ�BSP_TxBuffer������



//---------------------����ר�ú���
void USM_BSP_PROCESS(BSP_Conf_TypeDef *BSP_Conf);					//�����¼���������
void USM_BSP_RESET(BSP_Conf_TypeDef *BSP_Conf);						//��λ�ӻ�
void USM_BSP_ResetToBSP(BSP_Conf_TypeDef *BSP_Conf);			//��λ�ӻ��豸--ʹ�ӻ�����ISPģʽ
void USM_BSP_ResetToRUN(BSP_Conf_TypeDef *BSP_Conf);			//�����ӻ�����BOOT0ʹ�ӻ���������
void USM_BSP_SetStatus(BSP_Conf_TypeDef *BSP_Conf,BSPM_STATUS_TypeDef BSPM_STATUS);		//����/��������״̬
BSPM_STATUS_TypeDef USM_BSP_GetStatus(BSP_Conf_TypeDef *BSP_Conf);		//��ȡ������ǰ״̬
void USM_BSP_Connect(BSP_Conf_TypeDef *BSP_Conf);					//���Ӵӻ�---���5ms���ӷ���0x7F���ôӻ�ʶ������
void USM_BSP_Get(BSP_Conf_TypeDef *BSP_Conf);							//��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
void USM_BSP_GetVR(BSP_Conf_TypeDef *BSP_Conf);						//��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
void USM_BSP_GetId(BSP_Conf_TypeDef *BSP_Conf);						//��ȡоƬ ID
void USM_BSP_ReadMemory(BSP_Conf_TypeDef *BSP_Conf);			//��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
void USM_BSP_Go(BSP_Conf_TypeDef *BSP_Conf);							//��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
void USM_BSP_WriteMemory(BSP_Conf_TypeDef *BSP_Conf);			//��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
void USM_BSP_WriteAddr(BSP_Conf_TypeDef *BSP_Conf);				//д��ַ----������ַ�Զ����Ӽ�����ж�
void USM_BSP_WriteData(BSP_Conf_TypeDef *BSP_Conf);				//д����----��ӻ�д��FLASH����
void USM_BSP_EraseMemory(BSP_Conf_TypeDef *BSP_Conf);			//����һ����ȫ�� Flash ҳ�棨����FLASH��ʼд���ַ��
void USM_BSP_EEraseMemory(BSP_Conf_TypeDef *BSP_Conf);		//ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾��
void USM_BSP_WriteProtect(BSP_Conf_TypeDef *BSP_Conf);		//ʹ��ĳЩ������д����
void USM_BSP_WriteUnProtect(BSP_Conf_TypeDef *BSP_Conf);	//��ֹ���� Flash ������д����
void USM_BSP_ReadProtect(BSP_Conf_TypeDef *BSP_Conf);			//ʹ�ܶ�����
void USM_BSP_ReadUnProtect(BSP_Conf_TypeDef *BSP_Conf);		//��ֹ������
void USM_BSP_GetAck(BSP_Conf_TypeDef *BSP_Conf);					//��ȡ�ӻ�Ӧ��
void USM_BSP_SendCommand(BSP_Conf_TypeDef *BSP_Conf,BSP_COMMAND_TypeDef Command);	//���ڱ�̷����������


//---------------------�ӻ�ר�ú���
void USS_BSP_PROCESS(BSP_Conf_TypeDef *BSP_Conf);					//�ӻ��¼���������
void USS_BSP_RESET(BSP_Conf_TypeDef *BSP_Conf);
void USS_BSP_Get(BSP_Conf_TypeDef *BSP_Conf);							//��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
void USS_BSP_GetVR(BSP_Conf_TypeDef *BSP_Conf);						//��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
void USS_BSP_GetId(BSP_Conf_TypeDef *BSP_Conf);						//��ȡоƬ ID
void USS_BSP_ReadMemory(BSP_Conf_TypeDef *BSP_Conf);			//��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
void USS_BSP_Go(BSP_Conf_TypeDef *BSP_Conf);							//��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
void USS_BSP_WriteMemory(BSP_Conf_TypeDef *BSP_Conf);			//��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
void USS_BSP_EraseMemory(BSP_Conf_TypeDef *BSP_Conf);			//����һ����ȫ�� Flash ҳ��
void USS_BSP_EEraseMemory(BSP_Conf_TypeDef *BSP_Conf);		//ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾��
void USS_BSP_WriteProtect(BSP_Conf_TypeDef *BSP_Conf);		//ʹ��ĳЩ������д����
void USS_BSP_WriteUnProtect(BSP_Conf_TypeDef *BSP_Conf);	//��ֹ���� Flash ������д����
void USS_BSP_ReadProtect(BSP_Conf_TypeDef *BSP_Conf);			//ʹ�ܶ�����
void USS_BSP_ReadUnProtect(BSP_Conf_TypeDef *BSP_Conf);		//��ֹ������
void USS_BSP_ACK(BSP_Conf_TypeDef *BSP_Conf);							//����������Ӧ��(0x79)
void USS_BSP_NACK(BSP_Conf_TypeDef *BSP_Conf);						//���������ͷ�Ӧ��(0x1F)



















































#endif
