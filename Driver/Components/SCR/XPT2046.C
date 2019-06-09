

/********************************************************************************
***XPT2046��������IC
********************************************************************************/
#include "XPT2046.H"
#include "stdint.h"



//��ȡһ������ֵ
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
#define READ_TIMES 15 //��ȡ����
#define LOST_VAL 5	  //����ֵ


//***���������β�ͬ��ԭ��Ĭ�ϵ�У׼����ֵ���ܻ�������ʶ��׼������У׼����ʹ�ã�������ʹ�ù̶���Ĭ��У׼����
unsigned short vx=4809,vy=7382;  //�������ӣ���ֵ����1000֮���ʾ���ٸ�ADֵ����һ�����ص�
unsigned short chx=102,chy=296;//Ĭ�����ص�����Ϊ0ʱ��AD��ʼֵ
//***���������β�ͬ��ԭ��Ĭ�ϵ�У׼����ֵ���ܻ�������ʶ��׼������У׼����ʹ�ã�������ʹ�ù̶���Ĭ��У׼����

struct tp_pix_  tp_pixad,tp_pixlcd;	 //��ǰ���������ADֵ,ǰ�������������ֵ

spi_def* pXpt2046;

/*******************************************************************************
*������			:	function
*��������		:	function
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
void api_xpt2046_configuration(spi_def* pInfo)
{
	pXpt2046	=	pInfo;
  //api_spi_configuration_gpio(pInfo);				//��ͨSPIͨѶ��ʽ����
	api_spi_configurationNR(pInfo);				//SPI�ӿ�����

}
//-----------------------------------------------------------------------------


/*******************************************************************************
*������			:	api_xpt2046_get_coordinate
*��������		:	���˲��������ȡ,��Сֵ��������100
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
unsigned short api_xpt2046_get_coordinate(unsigned short *x,unsigned short *y)
{
	unsigned short xtemp,ytemp;	
	xtemp=xpt2046_get_ad_filter(CMD_RDX);		//��ȡX��λ��
	ytemp=xpt2046_get_ad_filter(CMD_RDY);		//��ȡY��λ��
	if(xtemp<100||ytemp<100)
		return 0;//����ʧ��
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}
//-----------------------------------------------------------------------------


/*******************************************************************************
*������			:	xpt2046_get_ad
*��������		:	��7846/7843/XPT2046/UH7843/UH7846��ȡadcֵ	  0x90=y   0xd0-x
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static unsigned short xpt2046_get_ad(unsigned char CMD)          
{
	unsigned short l;
	spi_set_nss_low(pXpt2046);
	/* �ڲ��ģʽ�£�XPT2046 ת����Ҫ24��ʱ�ӣ�8��ʱ���������֮��1��ʱ��ȥ�� */
	//1)--------------------����ת������
	api_spi_ReadWrite_byte(pXpt2046,CMD);        				//�Ϳ����ּ��ò�ַ�ʽ��X���� ��ϸ����й�����
	//2)--------------------��ȡ���ݣ����е�һ��ʱ��Ҫȥ��
	xpt2046_DelayNop(50);
	l	=	api_spi_ReadWrite_byte(pXpt2046,0xFF)&0x7F;    	//��һ��ʱ��ȥ����//�Ϳ����ּ��ò�ַ�ʽ��X���� ��ϸ����й�����
	l<<=8;
	l	|=	api_spi_ReadWrite_byte(pXpt2046,0xFF);        //�Ϳ����ּ��ò�ַ�ʽ��X���� ��ϸ����й�����
	//3)--------------------ֻ��12λ��Чֵ����3λ��Ч
	l>>=3;
	spi_set_nss_high(pXpt2046);
	return l;
}
/*******************************************************************************
*������			:	xpt2046_get_ad_filter
*��������		:	�˲�����
*����				: 
*����ֵ			:	��
*�޸�ʱ��		:	��
*�޸�˵��		:	��
*ע��				:	wegam@sina.com
*******************************************************************************/
static unsigned short xpt2046_get_ad_filter(unsigned char xy)
{
	/* ȥ�����ֵ��ȥ����Сֵ����ƽ��ֵ */
	unsigned short i, j;
	unsigned short buf[READ_TIMES];
	unsigned short sum=0;
	unsigned short temp;
	//1)-------------------------��������
	for(i=0;i<READ_TIMES;i++)
	{				 
		buf[i]=xpt2046_get_ad(xy);	    
	}
	//2)-------------------------��С��������
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
	//3)-------------------------ȥ������������Сֵ
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
		sum+=buf[i];
	//4)-------------------------��ƽ��ֵ
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
}

//------------------------------------------------------------------------------

   


unsigned char tpstate(void)
{
//	return 	Penirq;
}
//**********************************************************
void spistar(void)                                     //SPI��ʼ
{
//	CS=1;
//	DCLK=1;
//	DIN=1;
//	DCLK=1;
}

//2�ζ�ȡADS7846,������ȡ2����Ч��ADֵ,�������ε�ƫ��ܳ���
//50,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
#define ERR_RANGE 20 //��Χ 
unsigned char Read_ADS2(unsigned short *x,unsigned short *y) 
{
	unsigned short x1,y1;
 	unsigned short x2,y2;
 	unsigned char flag;    
    flag=api_xpt2046_get_coordinate(&x1,&y1);   
    if(flag==0)return(0);
    flag=api_xpt2046_get_coordinate(&x2,&y2);	
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-ERR_RANGE��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)>>1;
        *y=(y1+y2)>>1;		
        return 1;
    }else return 0;	  
} 
//��ȷ��ȡһ������,У׼��ʱ���õ�	   
unsigned char Read_TP_Once(void)
{
	unsigned char re=0;
	unsigned short x1,y1;
	unsigned short x2,y2;
	while(re==0)
	{
		while(!api_xpt2046_get_coordinate(&x1,&y1));
//		delayms(10);
		while(!api_xpt2046_get_coordinate(&x2,&y2));
		if((x2<=x1&&x1<x2+3)||(x1<=x2&&x2<x1+3))
		{
			tp_pixad.x=(x1+x2)/2;
			tp_pixad.y=(y1+y2)/2;
			re=1; 
		}	
	} 
	return re;
}


//////////////////////////////////////////////////
//��LCD�����йصĺ���  
//��һ��������
//����У׼�õ�
void Drow_Touch_Point(unsigned short x,unsigned short y)
{
//	LCD_DrawLine(x-12,y,x+13,y);//����
//	LCD_DrawLine(x,y-12,x,y+13);//����
//	LCD_DrawPoint(x+1,y+1);
//	LCD_DrawPoint(x-1,y+1);
//	LCD_DrawPoint(x+1,y-1);
//	LCD_DrawPoint(x-1,y-1);
}	  
//ת�����
//���ݴ�������У׼����������ת����Ľ��,������X0,Y0��
unsigned char Convert_Pos(void)
{		 	 
	unsigned char l=0; 
	if(api_xpt2046_get_coordinate(&tp_pixad.x,&tp_pixad.y))
	{
		l=1;
		tp_pixlcd.x=tp_pixad.x>chx?((unsigned long)tp_pixad.x-(unsigned long)chx)*1000/vx:((unsigned long)chx-(unsigned long)tp_pixad.x)*1000/vx;
		tp_pixlcd.y=tp_pixad.y>chy?((unsigned long)tp_pixad.y-(unsigned long)chy)*1000/vy:((unsigned long)chy-(unsigned long)tp_pixad.y)*1000/vy;
	}
	return l;
}	   
//������У׼����
//�õ��ĸ�У׼����
#define tp_pianyi 80   //У׼����ƫ����	
#define tp_xiaozhun 1000   //У׼����
void Touch_Adjust(void)
{	
	float vx1,vx2,vy1,vy2;  //�������ӣ���ֵ����1000֮���ʾ���ٸ�ADֵ����һ�����ص�
	unsigned short chx1,chx2,chy1,chy2;//Ĭ�����ص�����Ϊ0ʱ��AD��ʼֵ
	unsigned short lx,ly;				 
	struct tp_pixu32_ p[4];
	unsigned char  cnt=0;	 
	cnt=0;				
//	POINT_COLOR=BLUE;
//	BACK_COLOR =WHITE;
//	LCD_Clear(WHITE);//����   
//	POINT_COLOR=RED;//��ɫ 
//	LCD_Clear(WHITE);//���� 
//	Drow_Touch_Point(tp_pianyi,tp_pianyi);//����1 
//	while(1)
//	{
//		if(Penirq==0)//����������
//		{
//			if(Read_TP_Once())//�õ����ΰ���ֵ
//			{  								   
//				p[cnt].x=tp_pixad.x;
//				p[cnt].y=tp_pixad.y;
//				cnt++; 
//			}			 
//			switch(cnt)
//			{			   
//				case 1:
//					LCD_Clear(WHITE);//���� 
//					while(!Penirq)  //�ȴ�����
//					{
//					}
//					Drow_Touch_Point(LCD_W-tp_pianyi-1,tp_pianyi);//����2
//					break;
//				case 2:
//					LCD_Clear(WHITE);//���� 
//					while(!Penirq)  //�ȴ�����
//					{
//					}
//					Drow_Touch_Point(tp_pianyi,LCD_H-tp_pianyi-1);//����3
//					break;
//				case 3:
//					LCD_Clear(WHITE);//���� 
//					while(!Penirq)  //�ȴ�����
//					{
//					}
//					Drow_Touch_Point(LCD_W-tp_pianyi-1,LCD_H-tp_pianyi-1);//����4
//					break;
//				case 4:	 //ȫ���ĸ����Ѿ��õ�
//	    		   	LCD_Clear(WHITE);//���� 
//				   	while(!Penirq)  //�ȴ�����
//					{
//					}
//			   		vx1=p[1].x>p[0].x?(p[1].x-p[0].x+1)*1000/(LCD_W-tp_pianyi-tp_pianyi):(p[0].x-p[1].x-1)*1000/(LCD_W-tp_pianyi-tp_pianyi);
//				 	chx1=p[1].x>p[0].x?p[0].x-(vx1*tp_pianyi)/1000:p[0].x+(vx1*tp_pianyi)/1000;
//				   	vy1=p[2].y>p[0].y?(p[2].y-p[0].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi):(p[0].y-p[2].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi);
//					chy1=p[2].y>p[0].y?p[0].y-(vy1*tp_pianyi)/1000:p[0].y+(vy1*tp_pianyi)/1000; 
//					
//					vx2=p[3].x>p[2].x?(p[3].x-p[2].x+1)*1000/(LCD_W-tp_pianyi-tp_pianyi):(p[2].x-p[3].x-1)*1000/(LCD_W-tp_pianyi-tp_pianyi);
//					chx2=p[3].x>p[2].x?p[2].x-(vx2*tp_pianyi)/1000:p[2].x+(vx2*tp_pianyi)/1000;
//				   	vy2=p[3].y>p[1].y?(p[3].y-p[1].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi):(p[1].y-p[3].y-1)*1000/(LCD_H-tp_pianyi-tp_pianyi);
//					chy2=p[3].y>p[1].y?p[1].y-(vy2*tp_pianyi)/1000:p[1].y+(vy2*tp_pianyi)/1000; 


//					if((vx1>vx2&&vx1>vx2+tp_xiaozhun)||(vx1<vx2&&vx1<vx2-tp_xiaozhun)||(vy1>vy2&&vy1>vy2+tp_xiaozhun)||(vy1<vy2&&vy1<vy2-tp_xiaozhun))
//					{
//						cnt=0;
//						LCD_Clear(WHITE);//���� 
//						Drow_Touch_Point(tp_pianyi,tp_pianyi);//����1 
//						continue;
//					}
//					vx=(vx1+vx2)/2;vy=(vy1+vy2)/2;
//					chx=(chx1+chx2)/2;chy=(chy1+chy2)/2;	
//																
//					//��ʾУ׼��Ϣ
//					LCD_Clear(WHITE);//���� 
//					POINT_COLOR=BLACK;
//					BACK_COLOR=BLUE;	
//			
//					lx=0;ly=50;			
//					LCD_ShowString(lx,ly,"VX1:");lx+=40;LCD_ShowNum(lx,ly,vx1,4);					
//					lx=0;ly+=20;
//					LCD_ShowString(lx,ly,"Vy1:");lx+=40;LCD_ShowNum(lx,ly,vy1,4);					
//					lx=0;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX1:");lx+=40;LCD_ShowNum(lx,ly,chx1,4);					
//				    lx=0;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY1:");lx+=40;LCD_ShowNum(lx,ly,chy1,4);

//					lx=100;ly=50;			
//					LCD_ShowString(lx,ly,"VX2:");lx+=40;LCD_ShowNum(lx,ly,vx2,4);					
//					lx=100;ly+=20;
//					LCD_ShowString(lx,ly,"Vy2:");lx+=40;LCD_ShowNum(lx,ly,vy2,4);					
//					lx=100;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX2:");lx+=40;LCD_ShowNum(lx,ly,chx2,4);					
//				    lx=100;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY2:");lx+=40;LCD_ShowNum(lx,ly,chy2,4);
//				
//					lx=50;ly=150;			
//					LCD_ShowString(lx,ly,"VX:");lx+=40;LCD_ShowNum(lx,ly,vx,4);					
//					lx=50;ly+=20;
//					LCD_ShowString(lx,ly,"Vy:");lx+=40;LCD_ShowNum(lx,ly,vy,4);					
//					lx=50;ly+=20; 
//					LCD_ShowString(lx,ly,"CHX:");lx+=40;LCD_ShowNum(lx,ly,chx,4);					
//				    lx=50;ly+=20; 
//					LCD_ShowString(lx,ly,"CHY:");lx+=40;LCD_ShowNum(lx,ly,chy,4);

//					lx=30;ly+=30;
//					LCD_ShowString(lx,ly,"Adjust OK!  Touch Anywhere To Continue");										  
//					Read_TP_Once(); //�ȴ�����������

//					LCD_Clear(WHITE);//����
//					return;//У�����				 
//			}
//		}
//	} 
}
void point(void) //��ͼ����
{
	double t=0;
	
//    while(1)
//	{  	
//		if(Penirq==0)
//		{
//			t=0;
//			if(Convert_Pos())	//�õ�����ֵ
//			{
//			//	LCD_ShowString(10,250,"X:");LCD_ShowNum(30,250,(u32)tp_pixad.x,6);	
//				//LCD_ShowString(180,250,"Y:");LCD_ShowNum(200,250,(u32)tp_pixad.y,6);	
//				LCD_ShowString(10,250,"X:");LCD_ShowNum(30,250,tp_pixad.x,4);
//				LCD_ShowString(180,250,"Y:");LCD_ShowNum(200,250,tp_pixad.y,4);					
//				LCD_DrawPoint_big(tp_pixlcd.x,tp_pixlcd.y);   
//			 }
//			
//		}
//		else
//		{	
//			t++;		
//			if(t>65000)
//			{
//				return;
//			}
//		}		

//		}
}	    
//-----------------------------------------------------------------------------









/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
* �޸�ʱ��		: ��
* �޸�����		: ��
* ����			: wegam@sina.com
*******************************************************************************/
void xpt2046_DelayNop(unsigned	short Time)
{
	while(Time--)
	{
		__nop();
	}
}

