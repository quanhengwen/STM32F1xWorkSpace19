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
#include 	"LinkedList.H"

#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�

//LINK_NODE** pNODE;
//===============================================================================
//����:	CreateNode
//����:	�����ڵ�
//����:	DataLenth-���洢�����ݳ���
//����:	NULL--�յ�ַ,��ʾ����ʧ��;���򷵻ش����Ľڵ��ַ
//===============================================================================
static LINK_NODE *CreateNode(char* DataAddr,unsigned long DataLength)
{
	LINK_NODE *NewNode; 		//NewNode���洴�����½��ĵ�ַ
  char *Addr;
	if(DataLength==0	||	DataAddr==NULL)				//���ݳ���Ϊ0,��ִ��
	{
		return	NULL;
	}
	//==================Ϊ�˽�����붯̬�ռ�
	NewNode = (LINK_NODE *) malloc (sizeof(LINK_NODE)); 	//����һ���½��(���붯̬�洢�ռ�)

	if (NULL == NewNode)	//����ʧ��
	{
		return NULL;				//���ؿյ�ַ
	}
	else									//����ɹ�:��һ���������ݿռ�
	{
		//==================Ϊ�˽���������ݴ洢�ռ�----ʹ�ö�̬�洢�ռ䷽ʽ
		Addr	= (char *) malloc (DataLength);			//�������ݴ洢��̬�洢�ռ�
		if(NULL	== Addr)	//���ݿռ�����ʧ��:�ͷŽ��,����NULL
		{
			free(NewNode);								//�ͷŽ������½��
			return NULL;									//���ؿյ�ַ
		}
    NewNode->DataAddr = Addr;
		NewNode->NextNode = NULL;
		NewNode->DataLen  = DataLength;											//�˽��洢�����ݴ�С
		memcpy(NewNode->DataAddr,DataAddr,DataLength);		//��������		
	}
	return	NewNode;					//����β���ĵ�ַ:׼����β�����������
}
//===============================================================================
//����:	InsertNode
//����:	��ͷ���ǰ������½��
//����:	HeadNODEx-ͷ���,NewNODEx-������Ľ��
//����:	����Ľ���ַ
//===============================================================================
static LINK_NODE *InsertNode(LINK_NODE* HeadNODEx,LINK_NODE* NewNODEx)
{
	if(HeadNODEx==NULL)						//ͷ���Ϊ�ս��----������
	{
		NewNODEx->PrevNode	=	NULL;	//��һ�������Ͻ��Ϊ�ս��
		NewNODEx->NextNode	=	NULL;	//�½�������������һ���Ϊ�ս��
		return	NewNODEx;
	}
	//==================�ǿս��
	else													//ͷ���ǿս��
	{		
		if(HeadNODEx->PrevNode	==NULL)			//ͷ�����Ͻ��Ϊ��
		{
			NewNODEx->PrevNode	=	NULL;				//�½����Ͻ��Ϊ��
			NewNODEx->NextNode	=	HeadNODEx;	//�½����½�㵽ͷ���
			HeadNODEx->PrevNode	=	NewNODEx;		//ͷ�����Ͻ�����ӵ��½��
		}
		else																											//ͷ�����Ͻ��ǿ�
		{
			NewNODEx->PrevNode	=	HeadNODEx->PrevNode->NextNode;		//�½��	���Ͻ�����ӵ�ͷ�����Ͻ��
			NewNODEx->NextNode	=	HeadNODEx;												//�½����½�����ӵ�ͷ���
			HeadNODEx->PrevNode->NextNode=NewNODEx;									//ͷ�����Ͻ����½�����ӵ��½��
			HeadNODEx->PrevNode=NewNODEx;														//ͷ�����Ͻ�����ӵ��½��
		}
		return	NewNODEx;
	}
}
//===============================================================================
//����:	InsertNode
//����:	��β���������½�㣬�����¸���β��㣬�����µ�β����ַ
//����:	EndNODEx-β���,NewNODEx-�����ӵĽ��
//����:	�µ�ͷ����ַ
//===============================================================================
static LINK_NODE *AddNode(LINK_NODE* EndNODEx,LINK_NODE* NewNODEx)
{
	if(EndNODEx==NULL)						//ͷ���Ϊ�ս��----������
	{
		NewNODEx->PrevNode	=	NULL;	//��һ�������Ͻ��Ϊ�ս��
		NewNODEx->NextNode	=	NULL;	//�½�������������һ���Ϊ�ս��
		EndNODEx	=	NewNODEx;				//
		return	NewNODEx;
	}
	//==================�ǿս��
	else													//ͷ���ǿս��
	{
    LINK_NODE* TempNODEx  = EndNODEx;   //�����½��
    TempNODEx->NextNode   = NewNODEx;   //β�����½�����ӵ��½��
    NewNODEx ->PrevNode   = EndNODEx;   //�½����Ͻ��ָ��β���
    
    NewNODEx->NextNode    = NULL;       //�½���β���Ϊ��
    EndNODEx =  NewNODEx;               //����β���
		return	EndNODEx;
	}
}
//===============================================================================
//����:	DeleteNode
//����:	ɾ�������
//����:	HeadNODEx-ɾ���Ľ��
//����:	
//===============================================================================
static LINK_NODE *DeleteNode(LINK_NODE* DelNODEx)
{
	LINK_NODE* pNODE;
	
	pNODE = DelNODEx;
	//==================�жϽ���Ƿ�Ϊ��
	if(pNODE	==	NULL)
	{
		return NULL;
	}
//	//==================��̬���ݷ�ʽ:��Ҫ�ͷ����ݿռ�
//	free(pNODE->DataAddr);										//�ͷ����ݿռ�
	//==================���ͷβ�ж�
	if(DelNODEx->PrevNode==NULL)			//----------------��һ�����Ϊ��
	{
		if(DelNODEx->NextNode==NULL)										//��һ�����Ϊ��,��һ�����Ϊ��
		{
      free(DelNODEx->DataAddr);									  //�ͷŽ�������ݻ���
			free(DelNODEx);															//�ͷ���ɾ�����
			DelNODEx	=	NULL;													//ɾ����ַ
		}
		else																				//��һ�����Ϊ��,��һ�����ǿ�
		{
			DelNODEx	=	DelNODEx->NextNode;				    //��ɾ������ַ����Ϊ��һ����ַ
			DelNODEx->PrevNode		=	NULL;	            //��һ����ͷ������Ϊ�ս��
      free(pNODE->DataAddr);									  //�ͷŽ�������ݻ���
			free(pNODE);															//�ͷ���ɾ�����
		}
	}
	else													//----------------��һ�����ǿ�
	{
		if(DelNODEx->NextNode==NULL)										//��һ�����ǿ�,��һ�����Ϊ��
		{
			DelNODEx->PrevNode->NextNode	=	NULL;		//�Ͻ���β�������Ϊ��
			DelNODEx	=	DelNODEx->PrevNode;			//�˽���ַ����Ϊ��һ����ַ
      free(pNODE->DataAddr);									  //�ͷŽ�������ݻ���
			free(pNODE);															//�ͷ���ɾ�����
		}
		else											//----------------��һ�����ǿ�,��һ�����ǿ�
		{
      DelNODEx->PrevNode->NextNode=DelNODEx->NextNode;	//�Ͻ��β������ӵ��˽����½��
			DelNODEx->NextNode->PrevNode=DelNODEx->PrevNode;	//Щ�����½���ͷ������ӵ�Щ�����Ͻ��
      DelNODEx  = DelNODEx->PrevNode;			              //�˽���ַ����Ϊ��һ����ַ
      free(pNODE->DataAddr);									  //�ͷŽ�������ݻ���
			free(pNODE);															//�ͷ���ɾ�����
		}
	}
	return DelNODEx;						//�����½���ַ
}
//===============================================================================
//����:	DeleteNode
//����:	ɾ�������
//����:	HeadNODEx-ͷ���,NewNODEx-������Ľ��
//����:	ͷ���
//===============================================================================
static LINK_NODE *GetEndNode(LINK_NODE* DelNODEx)
{
	LINK_NODE* pNODE;
	pNODE	=	DelNODEx;
	if(pNODE	==	NULL)
	{
		return NULL;
	}
	if(pNODE->NextNode	==NULL)
	{
		return pNODE;
	}
	else
	{
		GetEndNode(pNODE->NextNode);
	}
	return pNODE;
}
//===============================================================================
//����: FindData
//����: ��ͷ��㿪ʼ����������ݽ��
//����: HeadNODE-ͷ���,DataAddr-�����ҵ�����,DataLength-���ݳ���
//����: �ҵ����ݣ����ؽ���ַ�����򷵻�NULL
//===============================================================================
LINK_NODE *FindData(LINK_NODE	*HeadNODE,char* DataAddr,unsigned long DataLength)
{
	unsigned short i	=0;
	LINK_NODE* TempNODEx	=	NULL;
	
	if(HeadNODE	==	NULL)   //�ս��
	{
		return NULL;
	}
	TempNODEx	=	HeadNODE;
	for(i=0;i<0xFF;i++)
	{
		if(TempNODEx  ==	NULL)
		{
			return NULL;
		}
		else if(memcmp(TempNODEx->DataAddr,DataAddr,DataLength)	==	0)				//���ҵ�����//�Ƚ��ڴ��������
		{
			return	TempNODEx;
		}
		else
		{
			TempNODEx	=	TempNODEx->NextNode;
		}
	}
	return NULL;
}
//===============================================================================
//����:	GetListLength
//����:	��ȡ��������
//����:	HeadNODEx-����ͷ���:��һ�����
//����:	�����ĳ���
//===============================================================================
unsigned long GetListLength(LINK_NODE* HeadNODEx)
{
	if(HeadNODEx==NULL)			//���Ϊ��
	{
		return 0;					//
	}
	return	1+GetListLength(HeadNODEx->NextNode);
}
//===============================================================================
//����:	FIFO_IN--����:FIFO�洢����---First in, First out
//����:	�洢���ݣ������ݴ��뵽β��㣬������β����ַ�����ش洢���
//����:	EndNode-β���
//����:	DataAddr-���洢�����ݵ�ַ
//����:	DataLenth-���洢�����ݳ���
//����:	���н��0--ʧ��,SaveLength--�Ѿ���������ݴ�С
//===============================================================================
unsigned short  FIFO_IN(LINK_NODE	**EndNode,char* SaveAddr,unsigned short SaveLength)
{
	//==================��ʱ����
	LINK_NODE *NewNode = NULL; 											//�½���ʱ���
  LINK_NODE *TempNode = *EndNode; 							  //��ʱ���
	//==================�������:���ؽ���ַ
	NewNode	=	CreateNode(SaveAddr,SaveLength);			//�õ�������Ľ���ַ
	//==================�ж���������			
	if(NewNode==NULL)		//����ʧ��
	{
		return 0;	        //�˳�,����0
	}
  //==================������
  if(NULL ==  TempNode)
  {
    NewNode->NextNode = NULL;
    NewNode->PrevNode = NULL;
    *EndNode = NewNode;
    return  SaveLength;
  }
  else
  {
    //===================����β��㲢����β��������½��
    LINK_NODE*  TempNode  = *EndNode;
    unsigned char i=0;
    for(i=0;i<0xFF;i++)
    {
      if(NULL ==  TempNode->NextNode)
      {
        break;
      }
      TempNode  = TempNode->NextNode;
    }
    TempNode->NextNode  = NewNode;
    NewNode->PrevNode   = TempNode;
    NewNode->NextNode   = NULL;
    *EndNode = NewNode;
  }
	//==================�жϴ������Ƿ�Ϊ������(ͷ����Ƿ�Ϊ��)
	
	return	SaveLength;
}

//===============================================================================
//����:	FIFO_OUT--����:FIFO�������---First in, First out
//����:	�������ݴ洢��ReadAddr--��HeadNode��ȡ,��ȡ��,HeadNodeָ����һ����ַ���ͷ��Ѷ�ȡ��Ľ�㣬���ؽڵ������ݳ���
//����:	ReadAddr--�洢�ڵ��ȡ�����ݵ�ַ
//����:	DataLenth---�洢�����ݴ�С
//===============================================================================
unsigned short FIFO_OUT(LINK_NODE	**HeadNode,char* ReadAddr)
{
  LINK_NODE *TempNode = *HeadNode; 							  //��ʱ���
  unsigned short DataLen = 0;			//�洢�����ݳ���
	//==================�ս��
	if(TempNode==NULL)							//����ͷ���Ϊ��---������
	{
		return NULL;												  //�˳�
	}
	//==================�ǿս��
	else		
	{
    //===================����ͷ��㲢�Ҹ���ͷ����ڴ����ݣ�Ȼ�������޸�ͷ����ַ���ͷ�ԭͷ����ڴ�
    unsigned char i = 0;
    for(i=0;i<0xFF;i++)
    {
      if(NULL ==  TempNode->PrevNode) //�˽���Ͻ��Ϊ�ձ�ʾ��Ϊͷ���
      {
        break;
      }
      TempNode  = TempNode->PrevNode;
    }
		//==================��������
    DataLen = TempNode->DataLen;
    if(DataLen)
    {
      if(NULL !=  ReadAddr)
      {
        free(ReadAddr);        
      }
      ReadAddr  = (char*)malloc((unsigned int)DataLen);
      if(NULL ==  ReadAddr)   //��̬�ڴ�����ʧ��
      {
        return 0;
      }
      memcpy(ReadAddr,TempNode->DataAddr,TempNode->DataLen);	          //���ƽ�������ݵ���������
    }
    //===================���¸�������ͷ��ַ���ͷ������ռ䣬��������һ��������ɾ������
    if(NULL !=  TempNode->NextNode)                           //�������н��
    {
      TempNode ->NextNode->PrevNode = NULL;
      *HeadNode  = TempNode  ->NextNode;
      free(TempNode->DataAddr);                               //�ͷŽ�������ݻ���
      free(TempNode);                                         //�ͷ���ɾ�����
    }
    else
    {
      free(TempNode->DataAddr);                               //�ͷŽ�������ݻ���
      free(TempNode);                                         //�ͷ���ɾ�����
      *HeadNode  = NULL;
    }
		return DataLen;					//�������ݿ���
	}
}
//===============================================================================
//����:	FIFO_DEL
//����:	FIFOɾ����Ӧ���ݵĽ��
//����:	HeadNODEx-ͷ���,NewNODEx-������Ľ��
//����:	ͷ���
//===============================================================================
LINK_NODE* FIFO_DEL(LINK_NODE	*DelNode)
{
	if(NULL ==  DelNode)
	{
		return 0;
	}
  if(NULL ==  DelNode->PrevNode)    //ͷ���Ϊ��
  {
    if(NULL ==  DelNode->NextNode)  //β���Ϊ��
    {
      free(DelNode->DataAddr);      //�ͷŽ�������ݻ���
      free(DelNode);                //�ͷ���ɾ�����
      DelNode = NULL;
    }
    else
    {
      LINK_NODE* pNODE	=	DelNode;
      DelNode = DelNode->NextNode;
      free(pNODE->DataAddr);        //�ͷŽ�������ݻ���
      free(pNODE);                  //�ͷ���ɾ�����
    }
  }
  
  else                              //ͷ���ǿ�
  {
    if(NULL ==  DelNode->NextNode)  //β���Ϊ��
    {
      LINK_NODE* pNODE	=	DelNode;
      DelNode = DelNode->NextNode;
      DelNode->PrevNode = NULL;
      free(pNODE->DataAddr);        //�ͷŽ�������ݻ���
      free(pNODE);                  //�ͷ���ɾ�����
    }
    else
    {
      LINK_NODE* pNODE	=	DelNode;
      DelNode->PrevNode->NextNode = DelNode->NextNode;
      DelNode->NextNode->PrevNode = DelNode->PrevNode;
      DelNode = DelNode->PrevNode;
      free(pNODE->DataAddr);        //�ͷŽ�������ݻ���
      free(pNODE);                  //�ͷ���ɾ�����
    }
  }
	return DelNode;	
}
//===============================================================================
//����:	LIFO_IN--��ջ:��������:LIFO�洢����---Last in, First out
//����:	��FIFO_IN��֮ͬ��Ϊ������ͷ�������ݣ����Ҹ���ͷ����ַΪ�½��
//����:	DataAddr-���洢�����ݵ�ַ
//����:	DataLenth-���洢�����ݳ���
//����:	���н��0--ʧ��,SaveLength--�Ѿ���������ݴ�С
//===============================================================================
unsigned long LIFO_IN(LINK_NODE	*HeadNode,char* SaveAddr,unsigned long SaveLength)
{
	LINK_NODE *NewNode = NULL; 											//��ʱ����
//	HeadNode->DataLen	=	SaveLength;							    //������洢�����ݴ�С
	NewNode	=	CreateNode(SaveAddr,SaveLength);		  //�õ�������Ľ���ַ
						
	if(NewNode==NULL)		//����ʧ��
	{
		return 0;					//
	}
  //��ͷ���ǰ�����½��
  if(NULL ==  HeadNode)                     //ͷ���Ϊ�ս��
  {
    NewNode->NextNode = NULL;
    NewNode->PrevNode = NULL;
    HeadNode  = NewNode;
  }
  else
  {
    LINK_NODE*  TempNode  = HeadNode;
    unsigned char i=0;
    for(i=0;i<0xFF;i++)
    {
      if(NULL ==  TempNode->PrevNode)
      {
        break;
      }
      TempNode  = TempNode->PrevNode;
    }
    NewNode->NextNode = TempNode;
    NewNode->PrevNode = NULL;
    HeadNode  = NewNode;                    //����ͷ���
    
  }
	return	SaveLength;
}

//===============================================================================
//����:	LIFO_OUT--��ջ:��������:LIFO�������---Last in, First out
//����:	��ջ,�ҵ�β��㣬�����ݿ�����ReadAddr���ͷ�β��㣬�����¸���β����ַ
//����:	DataAddr--���ݵ�ַ
//����:	DataLenth---�洢�����ݴ�С
//===============================================================================
LINK_NODE* LIFO_OUT(LINK_NODE	*EndNode,char* ReadAddr)
{
	//==================�ս��
	if(EndNode==NULL)							//����ͷ���Ϊ��---������
	{
		return 0;														//�˳�
	}
	//==================�ǿս��
	else		
	{
    LINK_NODE*  TempNode  = EndNode;
    do
    {
      TempNode  = TempNode->NextNode;
    }
    while(NULL ==  TempNode->NextNode);
    TempNode->PrevNode->NextNode  = NULL;
    EndNode = TempNode->PrevNode;
    memcpy(ReadAddr,TempNode->DataAddr,TempNode->DataLen);                //��������
    free(TempNode->DataAddr);                               //�ͷŽ�������ݻ���
    free(TempNode);                                         //�ͷ���ɾ�����
    
		return EndNode;																			    //�����µ�β����ַ
	}
}
//===============================================================================
//����:	ListTest
//����:	
//����:	
//����:	
//===============================================================================
unsigned long LinkListTest(LINK_NODE	*LISTx,char* DataAddr,unsigned long SaveLength,unsigned char CMD)
{
	unsigned long	RevLength	=	0;
//	//===========================FIFO����
//	//--------------------����			----����0x01;
//	if(CMD	==	0x01)
//	{
//		RevLength	=	FIFO_IN(LISTx,DataAddr,SaveLength);		//FIFO�洢����---First in, First out
//	}
//	//--------------------����			----����0x02
//	else if(CMD	==	0x02)
//	{
//		RevLength	=	FIFO_OUT(LISTx,DataAddr);							//FIFO�������---First in, First out
//	}	
//	//--------------------ɾ�����		----����0x03
//	else if(CMD	==	0x03)
//	{
//		RevLength	=	FIFO_DEL(LISTx,DataAddr,SaveLength);	//FIFOɾ����Ӧ���ݵĽ��
//	}
//	//--------------------��������		----����0x04
//	else if(CMD	==	0x04)
//	{
//	}
//	//--------------------�������		----����0x05
//	else if(CMD	==	0x05)
//	{
//	}
//	
//	
//	//===========================LIFO����
//	//--------------------����			----����0x11
//	else if(CMD	==	0x11)
//	{
//	}
//	//--------------------����			----����0x12
//	else if(CMD	==	0x11)
//	{
//	}
//	//--------------------ɾ�����		----����0x13
//	else if(CMD	==	0x11)
//	{
//	}
//	//--------------------��������		----����0x14
//	else if(CMD	==	0x11)
//	{
//	}
//	//--------------------�������		----����0x15
//	else if(CMD	==	0x11)
//	{
//	}
//	return RevLength;
}


