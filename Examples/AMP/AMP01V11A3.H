#ifndef __AMP01V11A3_H
#define __AMP01V11A3_H

#include	"AMP_Protocol.H"

//#include	"AMP_PHY.H"

#define 	ampArrySize    	30       		//ÿ�����Ͷ˿ڴ����ͻ����������


//typedef struct _ampdata       //�������ݣ�
//{
//  unsigned char Cab_Addr;         	//���ַ
//  unsigned char MB_Flag;          	//�����־://0--����1--����
//}ampdatadef;
typedef struct _ampCache       //�������ݣ�
{
  unsigned char arry;           		//��������0ʱΪ�գ��ӵ�1����ʼ���ͣ�������һ���������1��ֱ��0������ʱ������һ�������ͻ��棬�»��������
  unsigned char size;           		//��ǰ������Ч���ݸ���(�������ֽ���)
  unsigned char data[maxFramesize];	//����
}ampCachedef;
typedef struct _ampCC       		//ͨѶ����
{
	ampCachedef	PcAck;						//Ӧ��
	ampCachedef	CbAck;						//Ӧ��
	ampCachedef	LyAck;						//Ӧ��
  ampCachedef	pc[ampArrySize];
	ampCachedef	cab[ampArrySize];
	ampCachedef	lay[ampArrySize];
}ampCCdef;

typedef struct _ampRequest
{
  unsigned short  unlock  		:1;//��������0-������1-������
  unsigned short  relslock		:1;//ִ�п�����0-�ޣ�1-����ִ��  
  unsigned short  reslock     :1;//�ͷ�����������0-������1-������
  unsigned short  reslockqust :1;//�ͷ�������0-������1-�����󣬼�⵽��״̬Ϊ��ʱ����ʱһ��ʱ���ٹر�����
  unsigned short  BLon        :1;//����������0-������1-������
  unsigned short  BLoff       :1;//�ر�������0-������1-������
  unsigned short  PLon        :1;//��幩�翪����0-������1-������
  unsigned short  PLoff       :1;//��幩�������0-������1-������
}ampReqdef;

typedef struct _ampWaitAck       //ͨѶ����
{
  unsigned char PC:1;			//0--����ȴ�Ӧ��1--��ȴ�Ӧ��
	unsigned char CB:1;			//0--����ȴ�Ӧ��1--��ȴ�Ӧ��
	unsigned char LY:1;			//0--����ȴ�Ӧ��1--��ȴ�Ӧ��
	unsigned char RF:1;			//0--����ȴ�Ӧ��1--��ȴ�Ӧ��
}ampWaitAckdef;

typedef struct _ampTims       //�������ݣ�
{
  unsigned short runningtime;     //���м�����
  unsigned short swicthidtime;    //���м�����
  unsigned short SYSLEDTime;      //LED���м�����
  unsigned short LockTime;        //LED���м�����
  unsigned short PcSendTime;      //���м�����
  unsigned short CabSendTime;     //���м�����
  unsigned short LaySendTime;     //���м�����
  unsigned short CardSendTime;    //���м����� 
	unsigned short BackLightTime;  	//���м�����
}ampTimesdef;

typedef struct _ampResend   //�ط�������
{
  unsigned char Pc;      //PC�ϴ��ط�����
  unsigned char Cab;     //��ӿ��ط�����
  unsigned char Lay;     //��ӿ��ط�����
  unsigned char Card;    //�������ӿ��ط�����
}ampResenddef;

typedef struct _ampdata       //�������ݣ�
{
  unsigned char Cab_Addr;         	//���ַ
  unsigned char MB_Flag;          	//�����־://0--����1--����
}ampdatadef;

typedef struct _ampStatus        //״̬��־��
{
  unsigned short lock    		:1;  //��״̬��0-��״̬��1-��״̬
  unsigned short unlockerr  :1;  //��������/ʧ��:0-�ޣ�1-����ʧ��
  unsigned short Online     :1;  //�豸����:0-���ߣ�1-����
}ampStadef;
typedef struct _amp
{
//  stTimesdef        Time;         //���ʱ�����
//  stSWdef           SwData;       //������־
//  stReqdef          Req;         //�������־
  
//  stFlgdef          Flag;         //��ر�־λ
//  stAckQdef         AckQ;         //��־Ӧ���־��0-����Ҫ��1-��ҪӦ����Ӧ�ӿ�
	
	ampReqdef				request;
	ampStadef     	status;          //״̬��
	ampWaitAckdef		AckW;				//�ȴ�Ӧ���־
  ampResenddef    ReSend;	
	ampCCdef				commdata;			//ͨѶ����
	ampdatadef			sysdata;		//�������ݺͱ�ʶ
	ampTimesdef			time;
}ampdef;


//extern stAMPProdef   AMPPro;

void AMP01V11A3_Configuration(void);
void AMP01V11A3_Server(void);




static ampCachedef* get_cache_addr(ampPortDef port);
static unsigned short set_cache_data(ampPortDef port,ampphydef* frame);
static ampCachedef* get_cache_data(ampPortDef port);
static unsigned char del_cache_data(ampPortDef port);


static unsigned char get_ack_wait_flag(ampPortDef port);
static void set_ack_wait_flag(ampPortDef port);
static void del_ack_wait_flag(ampPortDef port);


static void Hardware_Configuration(void);
static void Communication_Configuration(void);
static void SwitchID_Configuration(void);
static void BackLight_Configuration(void);
static void LayPower_Configuration(void);
static void Lock_Configuration(void);
static void Led_Configuration(void);



static void pc_receive_data_process(void);
static void pc_send_data_process(void);

static void cab_receive_data_process(void);
static void cab_send_data_process(void);

static void lay_receive_data_process(void);
static void lay_send_data_process(void);



static void lay_data_process(unsigned char* pBuffer,unsigned short len);
static void cmd_process(ampphydef* frame);
static void received_ack_process(ampPortDef port);

static void status_server(void);
static void lock_status_updata(void);



static void set_door(unsigned char flag);
static void set_backlight(unsigned char flag);
static void set_layer_power(unsigned char flag);


void set_ackup_frame(ampPortDef Port);
void set_ackdown_frame(ampPortDef Port);

void set_comm_time_out_msg(ampPortDef Port,ampphydef* frame);


static void door_server(void);
static void Led_Server(void);
void Pc_Server(void);
void Cab_Server(void);
void Lay_Server(void);



void Tim_Server(void);


#endif