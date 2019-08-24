#ifndef __AMP01V14A2_H
#define __AMP01V14A2_H

#include	"AMP_Protocol.H"

//#include	"AMP_PHY.H"

#define	ampAckSize    	10       		//ÿ�����Ͷ˿ڴ����ͻ����������
#define	ampArrySize    	20       		//ÿ�����Ͷ˿ڴ����ͻ����������

#define	ampledtime			10			//ms

//	��ʼ��		����		����	��ַ1			��ַ2			��ַ3			����0~N		BCC16У��	������
//	START		Length	CMD		Address1	Address2	Address3			


typedef struct _ampAck       //�������ݣ�
{
  unsigned char head;     	//0x7E
  unsigned char length;   	//����λ������head,leng,crcl,crch,end����	
	ampcmddef			cmd;				//����/״̬
	unsigned char address1;		//���ַ
  unsigned char address2;		//���ַ
  unsigned char address3;		//λ��ַ
	unsigned char status;			//״̬-Ĭ��0
	unsigned char Crc16L;
	unsigned char Crc16H;
	unsigned char end;				//0x7F
	
	unsigned char flag;				//��־:0-��Ӧ��1-��Ҫ�ϱ�Ӧ��
}ampAckDef;
//---------------------------------------------------------------------
typedef struct _ampCache       //�������ݣ�
{
  unsigned char arry;           		//��������0ʱΪ�գ��ӵ�1����ʼ���ͣ�������һ���������1��ֱ��0������ʱ������һ�������ͻ��棬�»��������
  unsigned char size;           		//��ǰ������Ч���ݸ���(�������ֽ���)
  unsigned char data[maxFramesize];	//����
}ampCachedef;
//---------------------------------------------------------------------
typedef struct _ampLight       //�ƹ��־
{
  struct
	{
		unsigned char	SetLevel			:2;	//�������ȵȼ�,�����������ȵȼ�:0-��������1-�������/��ƣ���ʱ������Ҫ���ʱ���󣬿���ʱLCD�������ȣ�2-�е����ȣ�����ʱ���屳�����ȣ�3-������ȣ�����ʱ��Ҫ����ʱ������
		unsigned char	WorkLevel			:2;	//��ǰ���ȵȼ�,�����������ȵȼ�:0-��������1-�������/��ƣ���ʱ������Ҫ���ʱ���󣬿���ʱLCD�������ȣ�2-�е����ȣ�����ʱ���屳�����ȣ�3-������ȣ�����ʱ��Ҫ����ʱ������
	}flag;	
	unsigned long	Time;       	//����ʱ�䵹��ʱ
}ampLightDef;
//---------------------------------------------------------------------
typedef struct _ampTims       //�������ݣ�
{
  unsigned short swicthidtime;    //���м�����
  unsigned short CardSendTime;    //���м�����
	unsigned short LayPowerOnTime; 	//����ƽӿ��ϵ�ʱ��
}ampTimesdef;
//---------------------------------------------------------------------
typedef struct _ampdata       					//�������ݣ�
{
  unsigned char Cab_Addr;         	//���ַ 6λ���� 345678
  unsigned char MB_Flag;          	//�����־:	//0--����1--����
	unsigned char PowerOn_Flag;       //�ϵ��־:0--���ϵ�δ��ɣ�1--�ϵ����
	unsigned char Display_Flag;       //͸������־://0--��ͨ/������1--����͸����
	unsigned char LayPowerOnFlag;   	//��幩��://0--δ���磬1--�ѹ���
	unsigned char SysLedFastFlag;   	//ϵͳ����ָʾ�ƿ�����˸��־://0--��ͨѶ����1����˸��1--��ͨѶ���ݣ�0.2����˸
}ampdatadef;
//---------------------------------------------------------------------
typedef struct _ampCommLed       				//ͨѶLEDָʾ��
{
	union
	{
		unsigned char code;
		struct
		{
			unsigned char pc_tx:1;		//0-����͵�ƽ/���ƣ�1-����ߵ�ƽ/���
			unsigned char pc_rx:1;
			
			unsigned char cb_tx:1;
			unsigned char cb_rx:1;
			
			unsigned char ly_tx:1;
			unsigned char ly_rx:1;
			
			unsigned char rf_tx:1;
			unsigned char rf_rx:1;
		}seg;
	}displaycode;
	unsigned char display;
  struct
	{
		unsigned char pc_tx:1;
		unsigned char pc_rx:1;
		
		unsigned char cb_tx:1;
		unsigned char cb_rx:1;
		
		unsigned char ly_tx:1;
		unsigned char ly_rx:1;
		
		unsigned char rf_tx:1;
		unsigned char rf_rx:1;
	}flag;
	struct
	{
		unsigned short pc_tx;
		unsigned short pc_rx;
		
		unsigned short cb_tx;
		unsigned short cb_rx;
		
		unsigned short ly_tx;
		unsigned short ly_rx;
		
		unsigned short rf_tx;
		unsigned short rf_rx;
	}time;		//��˸����
	unsigned char	 ClearFlag;		//0-������ʾ��1-�����ʾ
	unsigned short FlashTime;		//ˢ��ʱ��
}ampCommLedDef;
//---------------------------------------------------------------------
typedef struct _ampLock       					//��
{
  struct
	{
		unsigned char	LockStatus	:1;	//��״̬��0-��״̬��1-��״̬
		unsigned char	LockUpdata	:1;	//�����£�0-�޸��£�1-��״̬����
	}flag;
	unsigned short LockTime;        //��������
}ampLockDef;
//---------------------------------------------------------------------
typedef struct _ampMagneticLock       	//������
{
  struct
	{
		unsigned char	LockStatus	:1;	//��״̬��0-��״̬��1-��״̬
		unsigned char	LockUpdata	:1;	//�����£�0-�޸��£�1-��״̬����
	}flag;
	unsigned short LockTime;        //��������
}ampMagneticLockDef;

//---------------------------------------------------------------------
typedef struct _ampCommPort        			//ͨѶ�˿�
{
	unsigned short 	SendWaitTime;      		//���͵ȴ�ʱ��--Ϊ0ʱ����������
	unsigned char 	SlaveSendEnable;    	//�ӻ��������ͱ�־:0-�ȴ���1-�����ϴ�
	unsigned char 	WaitAckFlag;      		//�ȴ�Ӧ���־:0--����ȴ�Ӧ��1--��ȴ�Ӧ��	
	unsigned char 	ReSendCount;      		//�ϴ��ط�����
	unsigned char		GetSlaveAddress;			//��ǰ��ѯ�Ĵӻ���ַ
	ampAckDef				AckFrame;							//Ӧ��֡
  ampCachedef			Buffer[ampArrySize];	//����
}ampCommPortDef;
//---------------------------------------------------------------------
typedef struct
{
	float temperature;		//�¶�--��ת��
	float humidity;  			//ʪ��--��ת��
	unsigned short SegTemperature;	//�¶�--δת��
	unsigned short SegHumidity;  	//ʪ��--δת��
	unsigned short Time;        //��������
}ampSHT20Def;
//---------------------------------------------------------------------
//=====================================================================
typedef struct _amp
{
	ampdatadef					sysdata;			//�������ݺͱ�ʶ
	ampTimesdef					time;	
	
	ampCommLedDef				CommLed;			//ͨѶLEDָʾ��	
	
	ampLockDef					Lock;					//�����	
	ampMagneticLockDef	MagneticLock;	//������
	
	ampLightDef					BackLight;		//���屳��
	ampLightDef					LcdLight;			//LCD����
	ampSHT20Def					SHT20;				//��ʪ�ȴ�����
	
	ampCommPortDef			Pc;						//PC�˿�����
	ampCommPortDef			Cab;					//��˿�����
	ampCommPortDef			Lay;					//��ӿ�����
}ampdef;
//---------------------------------------------------------------------





void API_AMP01V14A2_Configuration(void);
void API_AMP01V14A2_Server(void);
static void AMP01V14A2_HIGH_Process(void);
static void AMP01V14A2_LOW_Process(void);


static void SysLed_server(void);
//--------------------------------static-cache_buffer
static ampCachedef* get_cache_addr(ampPortDef port);
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame);
static ampCachedef* get_cache_data(ampPortDef port);
static unsigned char del_cache_data(ampPortDef port);
//--------------------------------static-ack_flag
static unsigned char get_ack_wait_flag(ampPortDef port);
static void set_ack_wait_flag(ampPortDef port);
//--------------------------------static-port_server
static void Pc_Server(void);
static void Cab_Server(void);
static void Lay_Server(void);
static void CardReader_Server(void);
//--------------------------------static-port_data_process
static void pc_receive_data_process(void);
static void pc_send_data_process(void);
static void cab_receive_data_process(void);
static void cab_send_data_process(void);
static void lay_receive_data_process(void);
static void lay_send_data_process(void);
static void cmd_process(ampphydef* frame);
//--------------------------------static-hardware_control
static void set_door(unsigned char flag);
static void set_backlight(unsigned char flag);
static void set_layer_power(unsigned char flag);
//--------------------------------static-set_frame
static void set_ackup_frame(ampPortDef Port);
static void set_ackdown_frame(ampPortDef Port);
static void set_lock_status_frame(void);
static void set_comm_time_out_frame(ampPortDef Port,ampphydef* frame);
//--------------------------------static-Server
static void SwitchID_Server(void);
static void Lock_server(void);
static void Magnetic_Lock_server(void);		//������
static void CommLed_Server(void);
static void BackLight_Server(void);
static void LcdLight_Server(void);
static void Tim_Server(void);
static void sht20_server(void);
//--------------------------------static-configuration
static void Hardware_Configuration(void);
static void Communication_Configuration(void);
static void SwitchID_Configuration(void);
static void BackLight_Configuration(void);
static void LayPower_Configuration(void);
static void Lock_Configuration(void);
static void Led_Configuration(void);
static void sht20_configuration(void);
//--------------------------------static-test
static void test_cabinet_address_none_process(void);		//δ�����ӿڷ�LCD���Գ���
static void set_time_sync_frame(void);		//δ�����ӿڷ�LCD���Գ���

#endif