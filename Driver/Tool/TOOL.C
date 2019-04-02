/************************************ TOOLS ************************************
* �ļ��� 	: LinkedList����
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/09/11
* ˵��   	: 
********************************************************************************
����˵��:
*
*
*
*
*
*
*
*
*******************************************************************************/
#include 	"TOOL.H"

#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�


#define NULL 0

#define LEN sizeof(struct student)

struct student
{
	long num; 						/*ѧ�� */
	float score; 					/*������������Ϣ���Լ��������������ֶ� */
	struct student *next; /*ָ����һ����ָ�� */
};
int n; /*������� */


/////////////////////////////////CRC//////////////////////////////////////////
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
BuildTimeDef* GetBuildTime(void* pDate, void*	pTime)
{
	static BuildTimeDef	BuildTime;
	//=============================================ת������
	BuildTime.year		=	GetBuildYear((char*)pDate);
	BuildTime.month		=	GetBuildMonth((char*)pDate);
	BuildTime.day			=	GetBuildDay((char*)pDate);
	//=============================================ת��ʱ��
	BuildTime.hour		=	GetBuildHour((char*)pTime);
	BuildTime.minute	=	GetBuildMinute((char*)pTime);
	BuildTime.second	=	GetBuildSecond((char*)pTime);
	return &BuildTime;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned short GetBuildYear(const char* DataStr)
{
	unsigned	short Year	=	0;
//	unsigned	char DataStr[]=__DATE__;
	if(NULL	==DataStr)
		return 0;
	Year	=	(unsigned short)((DataStr[7]-'0')*1000+(DataStr[8]-'0')*100+(DataStr[9]-'0')*10+(DataStr[10]-'0'));
	return	Year;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetBuildMonth(const char* DataStr)
{
	unsigned	char Month	=	0;
	if(NULL	==DataStr)
		return 0;
	Month	=	(DataStr[2] == 'c' ? 0\
					:DataStr[2] == 'b' ? 1\
					:DataStr[2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \
					:DataStr[2] == 'y' ? 4 \
					:DataStr[2] == 'n' ? 5 \
					:DataStr[2] == 'l' ? 6 \
					:DataStr[2] == 'g' ? 7 \
					:DataStr[2] == 'p' ? 8 \
					:DataStr[2] == 't' ? 9 \
					:DataStr[2] == 'v' ? 10 : 11)+1;
//	Month	=	(unsigned char)((DataStr[7]-'0')*1000+(DataStr[8]-'0')*100+(DataStr[9]-'0')*10+(DataStr[10]-'0'));
	return	Month;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetBuildDay(const char* DataStr)
{
	unsigned	char Day	=	0;
	if(NULL	==DataStr)
		return 0;

	Day	=	(unsigned char)((DataStr[4] == ' ' ? 0 : DataStr[4] - '0') * 10+ (DataStr[5] - '0'));

	return	Day;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetBuildHour(const char* TimeStr)
{
	unsigned	char Hour	=	0;

	if(NULL	==TimeStr)
		return 0;

	Hour	=	(unsigned char)((TimeStr[0]-'0')*10+(TimeStr[1]-'0'));
	return	Hour;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetBuildMinute(const char* TimeStr)
{
	unsigned	char Minute	=	0;
	if(NULL	==TimeStr)
		return 0;

	Minute	=	(unsigned char)((TimeStr[3]-'0')*10+(TimeStr[4]-'0'));
	return	Minute;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetBuildSecond(const char* TimeStr)
{
	unsigned	char Second	=	0;
	if(NULL	==TimeStr)
		return 0;
	Second	=	(unsigned char)((TimeStr[6]-'0')*10+(TimeStr[7]-'0'));
	return	Second;
}
  
/*******************************************************************************
* ������		:	8λ���У��
* ��������	: 
* ����			: buffer��������ݻ�����ʼ��ַ
              numb���ݳ��ȣ�������
* ���			: ������
* ����			: ��
*******************************************************************************/
unsigned char BCC8(const unsigned char *buffer,unsigned short num)			//���У��
{
	u16 i=0;
	u16 temp=0;
	if(NULL	==	buffer)
	{
		return 0;
	}
	if(1	>=	num)
	{
		return 0;
	}
	for(i=0;i<num;i++)
	{
		temp=temp^buffer[i];
	}
	return temp;
}
/*******************************************************************************
* ������		:	���У��
* ��������	: 
* ����			: 
* ���			: ������
* ����			: ��
*******************************************************************************/
unsigned char CRC8(const unsigned char *buffer)			//ѭ������У��
{
	return 0;
}
/*******************************************************************************
* ������		:	��������У��
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
unsigned char LRC8(const unsigned char *buffer)		//��������У��
{
	return 0;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetMaxChar(unsigned char *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned char TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData<pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned short GetMaxShort(unsigned short *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned short TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData<pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned long GetMaxLong(unsigned long *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned long TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData<pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned char GetMinChar(unsigned char *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned char TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData>pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned short GetMinShort(unsigned short *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned short TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData>pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned long GetMinLong(unsigned long *pBuffer,unsigned short Length)
{
	unsigned short i	=	0;
	unsigned long TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		if(TempData>pBuffer[i])
		{
			TempData	=	pBuffer[i];
		}
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned long GetAveLong(unsigned long *pBuffer,unsigned short Length)	//��ȡunsigned long��ƽ��ֵ
{
	unsigned short i	=	0;
	unsigned long TempData=0;
	
	TempData	=	pBuffer[0];
	for(i=1;i<Length;i++)
	{
		TempData	=	(TempData	+	pBuffer[i])/2;
	}
	return TempData;
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
unsigned long GetVarLong(unsigned long *pBuffer,unsigned short Length)	//��ȡunsigned long����
{
	unsigned short i	=	0;
	unsigned long TempData=0;
	long VarData=0;
	
	//��ȡƽ��ֵ
	TempData	=	GetAveLong(pBuffer,Length);	//��ȡunsigned long��ƽ��ֵ
		
	for(i=0;i<Length;i++)
	{
		if(pBuffer[i]>=TempData)
		{
			VarData+=(pBuffer[i]-TempData)*(pBuffer[i]-TempData);
		}
		else
		{
			VarData-=(TempData-pBuffer[i])*(TempData-pBuffer[i]);
		}
	}
	if(VarData<0)
	{
		VarData	=	0	-	VarData;
	}
	return VarData;
}
