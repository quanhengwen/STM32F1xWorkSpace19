#ifndef __AMP_CABV11_H
#define __AMP_CABV11_H

#include	"AMP_PHY.H"

//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_nvic.h"
//#include "stm32f10x_spi.h"

//#include "STM32_GPIO.H"


//#include	"stdio.h"			//����printf
//#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

//------------------ͨѶ�ӿ�--PC
#define CommPcPort        USART2
//------------------ͨѶ�ӿ�--���
#define CommCbPort       USART1
#define CommCbCTLPort    GPIOA         //�����ƽӿڣ��ߵ�ƽ����
#define CommCbCTLPin     GPIO_Pin_8
//------------------ͨѶ�ӿ�--���ӿ�
#define CommLayPort       UART4
#define CommLayCTLPort    GPIOA         //�����ƽӿڣ��ߵ�ƽ����
#define CommLayCTLPin     GPIO_Pin_12
//------------------ͨѶ�ӿ�--�������ӿ�
#define CommCardPort      USART3
#define CommCardCTLPort   GPIOC         //�����ƽӿڣ��ߵ�ƽ����
#define CommCardCTLPin    GPIO_Pin_8
#define CommCardBaudRate  19200         //������ͨѶ������

//------------------���ӿ�J10
#define LockDrPort    GPIOB         //�����ƽӿڣ��ߵ�ƽ����
#define LockDrPin     GPIO_Pin_1
#define LockSiPort    GPIOC         //���źţ���⵽�͵�ƽ��ʾ���ѿ�
#define LockSiPin     GPIO_Pin_12
#define UnLock        LockDrPort->BSRR  = LockDrPin    //����
#define ResLock       LockDrPort->BRR   = LockDrPin    //�ͷ�������
#define GetLockSts    LockSiPort->IDR & LockSiPin      //��ȡ����״̬�����Ϊ0����ʾ���ѿ�
//------------------����ӿ�J11��VCC��EN��
#define BackLightPort GPIOB         //�ߵ�ƽ�رգ��͵�ƽ����
#define BackLightPin  GPIO_Pin_0
#define BackLightOn   BackLightPort->BSRR   = BackLightPin
#define BackLightOff  BackLightPort->BRR    = BackLightPin
//------------------����Դ����J5��J6��J9����һ�����Ƶ�Դ
#define LayPowerPort  GPIOB         //�ߵ�������Դ���͵�ƽ�رյ�Դ
#define LayPowerPin   GPIO_Pin_2
#define LayPowerOn    LayPowerPort->BSRR = LayPowerPin   //������Դ
#define LayPowerOff   LayPowerPort->BRR  = LayPowerPin   //�رյ�Դ


extern unsigned char MainFlag; //0--����1--����

void AMP_CABV11_Configuration(void);
void AMP_CABV11_Server(void);
void AMP_CABV11_Loop(void);
void CardReaderInitLoop(void);

void AMPCAB_Receive(void);
void AMPCAB_SwitchIDServer(void);
void AMPCAB_SYSLED(void);
void AMPCAB_BackLight(void);
void LockServer(void);


void AMPCABSwitchID_Configuration(void);
void AMPCAB_GenyConfiguration(void);
void AMPCABCOMM_Configuration(void);


unsigned short AMPCAB_SendBuff(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);


unsigned short PCnet_Send(unsigned char* pBuffer,unsigned short length);      //��PC������Ϣ
unsigned short Cabinet_Send(unsigned char* pBuffer,unsigned short length);    //����������Ϣ
unsigned short Laynet_Send(unsigned char* pBuffer,unsigned short length);     //����巢����Ϣ
unsigned short CardPort_Send(unsigned char* pBuffer,unsigned short length);   //���������ӿڷ�����




void Msg_Process(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);        //��Ϣ����


void Msg_ProcessCB(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);      //����Ϣ����
void Msg_ProcessPcPort(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
void Msg_ProcessCbPort(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
void Msg_ProcessCaPort(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
void Msg_ProcessLyPort(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
void CardDataSendUp(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);




void RequestServer(void);   //���������
void LockStatusUpdata(eucmddef Cmd,eLockStsdef std);
void CommTimeOutUpdata(enCCPortDef Port,stampaddrdef address);

#endif