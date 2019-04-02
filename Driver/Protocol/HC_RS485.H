#ifndef __HC_RS485_H
#define __HC_RS485_H




#define	HC485BuffSize	16


//==============================��ʼ��ʶ��&������ʶ��
typedef enum	_HC485Head
{
	eHead		=	0x7E,
	eEnd		=	0x7F
}eRS485HeadDef;

//==============================������
typedef enum	_HC485Cmd
{
	eR,
}eRS485CmdDef;
//==============================״̬��/������
typedef enum	_HC485ErrCode
{
	eGetStatus	=	1,
	eGetData		=	2,
	eSetStatus	=	3,
	eSetData		=	4,
}eRS485ErrDef;
//==============================ͨѶЭ��ṹ��
typedef	struct	_sHC485Farme			//485ͨѶ��Ϣ֡�ṹ��MessageFarme  
{
	eRS485HeadDef		eHead;			//b0ʶ���	0x7E
	unsigned char		dAdd;				//b1Ŀ���ַ����ʱ����Ϊ��Ԫ���ַ���·�Ϊ���յ�ַ���ϴ�Ϊ0x00
	unsigned char		sAdd;				//b2Դ��ַ �·�Ϊ0x00���ϴ�Ϊ����ַ
	unsigned char		nSerial;		//b3��ˮ�� 0x01~0xFF ÿ��ʹ�ú��1
	eRS485CmdDef		Cmd;				//b4����ţ�0x12-��ȡָ��IC���ţ�0x13ȡҩ,�ϱ�����Ҳ�ڴ�λ
	unsigned char		Ucode;			//b5�û��룺��������ԭ������
	unsigned char		Length;			//b6��ַ+�쳣��̶�����Ϊ4�������ݳ���Ϊ3
	unsigned char		Addr1;			//��Ԫ���ַ
	unsigned char		Addr2;			//���ַ
	unsigned char		Addr3;			//�˿ڵ�ַ/�۵�ַ

	eRS485ErrDef		ErrCode;		//״̬��/������
	
	unsigned char		data[HC485BuffSize];		//BusDataSizeΪ������ݽ��ջ��壬ʵ�����ݴ�С����Length-4(4Ϊ��ַ��״̬�볤��)
	
	unsigned char		Bcc8;				//���У��	//���㷶ΧΪ��ʼ�ε����ݶ�(dAdd~data[n])��������
	eRS485HeadDef		eEnd;				//������ 0x7F
}sHC485FrmDef;













#endif


