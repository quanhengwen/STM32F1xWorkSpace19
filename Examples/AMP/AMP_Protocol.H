#ifndef __AMP_Protocol_H
#define __AMP_Protocol_H


#define headcode  (unsigned char)0x7E
#define endcode   (unsigned char)0x7F
	
#define minFrameSize 7   //��С֡����---7Ӧ��֡

#define maxFramesize 256  		//5+128	//ͨѶ���֡�ֽ���

#define DspMaxNameLen		64		//���ֳ���--�ֽ�
  
#define gDatasize       		maxmsgsize	//����������ݰ�����
#define arrysize        		10       		//ÿ�����Ͷ˿ڴ����ͻ����������
#define MaxSlave        		30        	//���ӻ�����(�񡢲㡢λ)
#define ampReSendTime  			100 	      	//�ط����ʱ�䵥λms
#define SendNopTime     		10  	      	//���͵ȴ�ʱ�䵥λms---Ӧ��ȴ�ʱ��
#define ampMaxResendCount  		5         	//����ʹ���
#define ampUnlockOuttime   		2000      	//��������ʱ��ʱʱ��
#define reslockdelaytime   	200   			//��⵽���Ѵ򿪺���ʱ�ر�����ʱ��ms
#define checkswitchidtime 	2000    		//��鲦�뿪��ʱ����
#define GetOnlinetime   		5000      	//��ȡ�����豸ʱ����

extern unsigned  char  ackupfarme[minFrameSize];
extern unsigned  char  ackdownfarme[minFrameSize];

//-----------------------------------------------------������
typedef enum  _ampPort
{
  NonPort     = 0,
  PcPort      = 1,  //PCͨѶ�ӿ�
  CabPort     = 2,  //��ͨѶ�ӿ�
  LayPort     = 3,  //��ͨѶ�ӿ�
  CardPort    = 4   //������ͨѶ�ӿ�
}ampPortDef;
typedef enum _ampStype		//״̬����
{
  ampStypeNone      = 0,    //��֧�ֵ�����/������
  ampStypeLock      = 1,    //��״̬,0--
  ampStypeComm      = 2,    //����״̬
}ampStypedef;
typedef enum _ampLockSts
{
  AmpLockOpen       = 0,    //����  
  AmpLockOn         = 1,    //������
  AmpLockOpenFailed = 2,    //����ʧ��
}ampLockStsdef;
typedef enum _amp_comm_sts
{
  AmpCommOK       = 0,    //�������� 
  AmpCommTimeOut  = 1,    //���ӳ�ʱ/ʧ��
}ampCommStsdef;



//-----------------------------------------------------ͨѶЭ����
typedef enum _ampecmd
{
  ampCmdNone      = (unsigned char)0x00,    	//��֧�ֵ�����/������
  ampCmdAck       = (unsigned char)0x01,    	//Ӧ�����ͣ�ֻ��һ�����ݣ�������ַ�����ݱ�ʾ״̬
  ampCmdLed       = (unsigned char)0x02,    	//LED/LCD���ƣ����������ݣ�����ַ��0-��ʾ�أ�1-��ʾ��
  ampCmdLock      = (unsigned char)0x03,    	//�������ֻ��һ�����ݣ�����ַ��0-��ʾ����״̬��1��ʾ����
  ampCmdCard      = (unsigned char)0x04,    	//������������ַ����һ������Ϊ0��ʾ������Ϊ1��ʾд����������濨����
  ampCmdPwr       = (unsigned char)0x05,    	//��幩�����
  ampCmdSta       = (unsigned char)0x06,    	//״̬����ҪΪ�ϱ�״̬
  ampCmdGetOnline = (unsigned char)0x07,    	//��������豸�����յ���Ӧ���ʾ����
  ampCmdBKligth   = (unsigned char)0x08,    	//����ƿ���
	ampCmdLcdData  	= (unsigned char)0x09,    	//LCD������ʾ���ݴ�������--��Ʒ���Ƽ������ʾ����
	ampCmdLcdConf  	= (unsigned char)0x0A,			//LCD��������޸ı���ɫ����,����ɫ,�����С
	ampCmdLcdPwr  	= (unsigned char)0x0B,			//LCD��Դ�������ָ��
}ampecmddef;
typedef enum _ampedir
{
	ampDirUp  			= (unsigned char)0x80,			//�ϴ�����
	ampDirDown  		= (unsigned char)0x00,			//�·�����
}ampedirdef;
typedef struct _ampcmd
{
	unsigned char cmd:6;
	unsigned char rv	:1;		//����
	unsigned char dir	:1;		//����0���¼�������Ӧ��1���ϼ�������Ӧ��
}ampcmddef;
typedef struct _ampaddr
{
  unsigned char address1;
  unsigned char address2;		//���ַ
  unsigned char address3;
}ampaddrdef;
typedef struct _ampcrc16
{
  unsigned char crcl;
  unsigned char crch;
}ampcrc16def;
typedef struct _ampmsg
{
  unsigned char length;   //��������λ
  ampcmddef 		cmd;
	ampaddrdef		addr;
	unsigned char data[maxFramesize-5];		//������head,leng,crcl,crch,end����
}ampmsgdef;
typedef struct _ampphy
{
  unsigned char head;
  ampmsgdef    	msg;
  ampcrc16def 	crc16;
  unsigned char end;
}ampphydef;





//------------------------------------------------------------------ͷ�ļ�
ampphydef* api_get_frame(unsigned char* pbuffer,unsigned short len);
unsigned short api_set_frame(ampphydef* frame,ampecmddef cmd,ampedirdef dir);




#endif


