#include "MFRC522.H"

#include "string.h"
#define MAXRLEN 18

//unsigned char mfrc522_UID[5],mfrc522_Temp[4];

/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_mfrc522_configuration(mfrc522def* pMfrc522)
{
	mfrc522_port_configuration(pMfrc522);
	mfrc522_set_reset(pMfrc522);					//复位RC522
	mfrc522_set_AntennaOff(pMfrc522);
	mfrc522_set_AntennaOn(pMfrc522);			//开启天线发射 
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_mfrc522_self_test(mfrc522def* pInfo)
{
	unsigned char i = 0;
	unsigned char fifo[128]={0};
	//------------------------reset
	mfrc522_set_rst_low(pInfo);
	mfrc522_delay_ms(5);
	mfrc522_set_rst_high(pInfo);
	mfrc522_delay_ms(5);
	
	//------------------------Clear the internal buffer by writing 25 bytes of 00h and implement the Config command.
	mfrc522_set_clk_low(pInfo);		//MF522_SCK = 0;
	mfrc522_set_nss_low(pInfo);		//MF522_NSS = 0;
	for(i=0;i<25;i++)
	{
		mfrc522_write_byte(pInfo,0x00);
	}
	mfrc522_set_nss_high(pInfo);		//MF522_NSS = 1
	mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
	//------------------------Enable the self test by writing 09h to the AutoTestReg register
	mfrc522_write_data(pInfo,AutoTestReg,0x09);
	//------------------------Write 00h to the FIFO buffer 64byte
	memset(fifo,0x00,128);
	mfrc522_write_buffer(pInfo,FIFODataReg,fifo,DEF_FIFO_LENGTH);
	//------------------------Start the self test with the CalcCRC command
	mfrc522_write_data(pInfo,AutoTestReg,0x09);
	//------------------------The self test is initiated
	mfrc522_delay_ms(10);
	//------------------------When the self test has completed, the FIFO buffer contains the following 64 bytes
	mfrc522_read_buffer(pInfo,FIFODataReg,fifo,DEF_FIFO_LENGTH);
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	function
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
void api_mfrc522_Auto_Reader(mfrc522def* pInfo)
{
	
	
//  while(1)
//  {
//		
//    if(PcdRequest(pInfo,0x52,mfrc522_Temp)==MI_OK)
//    {
//      if(PcdAnticoll(pInfo,mfrc522_UID)==MI_OK)
//      { 
//				
//      }
//    }
//  } 
}

/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//          pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(mfrc522def* pInfo,unsigned char req_code,unsigned char *pTagType)
{
   char status;  
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(pInfo,Status2Reg,0x08);						//寄存器包含接收器和发送器和数据模式检测器的状态标志
   mfrc522_write_data(pInfo,BitFramingReg,0x07);	//不启动数据发送
   SetBitMask(pInfo,TxControlReg,0x03);						//TX1、TX2输出信号将传递经发送数据调制的13.56MHz的能量载波信号。
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);		//通过RC522和ISO14443卡通讯,读取数据

   if ((status == MI_OK) && (unLen == 0x10))		//读取到数据--寻卡成功
   {    
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号缓存地址，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(mfrc522def* pInfo,unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(pInfo,Status2Reg,0x08);					//清除相关状态寄存器
    mfrc522_write_data(pInfo,BitFramingReg,0x00);	//
    ClearBitMask(pInfo,CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;		//防冲撞
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);	//通过RC522和ISO14443卡通讯,读取数据

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
			 {   
					 *(pSnr+i)  = ucComMF522Buf[i];
					 snr_check ^= ucComMF522Buf[i];
			 }
			 if (snr_check != ucComMF522Buf[i])
			 {   status = MI_ERR;    }
    }    
    SetBitMask(pInfo,CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(mfrc522def* pInfo,unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(pInfo,ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(pInfo,Status2Reg,0x08);

    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(mfrc522def* pInfo,unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(pInfo,PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(mfrc522_read_data(pInfo,Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(mfrc522def* pInfo,unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(mfrc522def* pInfo,unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(pInfo,ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}





/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(mfrc522def* pInfo,unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(pInfo,DivIrqReg,0x04);
    mfrc522_write_data(pInfo,CommandReg,PCD_IDLE);
    SetBitMask(pInfo,FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   mfrc522_write_data(pInfo,FIFODataReg, *(pIndata+i));   }
    mfrc522_write_data(pInfo,CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = mfrc522_read_data(pInfo,DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = mfrc522_read_data(pInfo,CRCResultRegL);
    pOutData[1] = mfrc522_read_data(pInfo,CRCResultRegM);
}


//////////////////////////////////////////////////////////////////////
//设置RC632的工作方式 
//////////////////////////////////////////////////////////////////////
char M500PcdConfigISOType(mfrc522def* pInfo,unsigned char type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       ClearBitMask(pInfo,Status2Reg,0x08);

 /* 	mfrc522_write_byte(CommandReg,0x20);    //as default   
			mfrc522_write_byte(ComIEnReg,0x80);     //as default
			mfrc522_write_byte(DivlEnReg,0x0);      //as default
			mfrc522_write_byte(ComIrqReg,0x04);     //as default
			mfrc522_write_byte(DivIrqReg,0x0);      //as default
			mfrc522_write_byte(Status2Reg,0x0);//80    //trun off temperature sensor
			mfrc522_write_byte(WaterLevelReg,0x08); //as default
			mfrc522_write_byte(ControlReg,0x20);    //as default
			mfrc522_write_byte(CollReg,0x80);    //as default
*/
       mfrc522_write_data(pInfo,ModeReg,0x3D);//3F
/*	   mfrc522_write_byte(TxModeReg,0x0);      //as default???
	   mfrc522_write_byte(RxModeReg,0x0);      //as default???
	   mfrc522_write_byte(TxControlReg,0x80);  //as default???

	   mfrc522_write_byte(TxSelReg,0x10);      //as default???
   */
       mfrc522_write_data(pInfo,RxSelReg,0x86);//84
 //      mfrc522_write_byte(RxThresholdReg,0x84);//as default
 //      mfrc522_write_byte(DemodReg,0x4D);      //as default

 //      mfrc522_write_byte(ModWidthReg,0x13);//26
       mfrc522_write_data(pInfo,RFCfgReg,0x7F);   //4F
	/*   mfrc522_write_byte(GsNReg,0x88);        //as default???
	   mfrc522_write_byte(CWGsCfgReg,0x20);    //as default???
       mfrc522_write_byte(ModGsCfgReg,0x20);   //as default???
*/
   	   mfrc522_write_data(pInfo,TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   mfrc522_write_data(pInfo,TReloadRegH,0);
       mfrc522_write_data(pInfo,TModeReg,0x8D);
	   mfrc522_write_data(pInfo,TPrescalerReg,0x3E);
	   

  //     PcdSetTmo(106);
	    		mfrc522_delay_ms(10);
       mfrc522_set_AntennaOn(pInfo);
   }
   else{ return -1; }
   
   return MI_OK;
}


/////////////////////////////////////////////////////////////////////
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(mfrc522def* pInfo,unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = mfrc522_read_data(pInfo,reg);
    mfrc522_write_data(pInfo,reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(mfrc522def* pInfo,unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = mfrc522_read_data(pInfo,reg);
    mfrc522_write_data(pInfo,reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char PcdComMF522(mfrc522def* pInfo,
									unsigned char Command, 
									unsigned char *pInData, 
									unsigned char InLenByte,
									unsigned char *pOutData, 
									unsigned int  *pOutLenBit)
{
	char status = MI_ERR;
	unsigned char irqEn   = 0x00;
	unsigned char waitFor = 0x00;
	unsigned char lastBits;
	unsigned char n;
	unsigned int i;

	switch (Command)
	{
		case PCD_AUTHENT:			//验证密钥
			irqEn   = 0x12;
			waitFor = 0x10;
			break;
		case PCD_TRANSCEIVE:	//发送并接收数据
			irqEn   = 0x77;
			waitFor = 0x30;
			break;
		default:
		 break;
	}
	//-------------------------------------
	mfrc522_write_data(pInfo,ComIEnReg,irqEn|0x80);		//使能相关中断
	ClearBitMask(pInfo,ComIrqReg,0x80);								//清除相关中断标志
	mfrc522_write_data(pInfo,CommandReg,PCD_IDLE);		//取消当前命令
	SetBitMask(pInfo,FIFOLevelReg,0x80);							//清除当前缓存中的字节数

	for (i=0; i<InLenByte; i++)
	{
		mfrc522_write_data(pInfo,FIFODataReg, pInData[i]);	//往FIFO缓存写入数据
	}

	mfrc522_write_data(pInfo,CommandReg, Command);		//启动发送


	if (Command == PCD_TRANSCEIVE)
	{
		SetBitMask(pInfo,BitFramingReg,0x80);		//全部发送
	}
    
//    i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
	i = 2500;	//根据时钟频率调整，操作M1卡最大等待时间25ms
	do 
	{
		n = mfrc522_read_data(pInfo,ComIrqReg);		//读取中断寄存器
		mfrc522_delay_us(10);
		i--;
	}
	while ((i!=0) && !(n&0x01) && !(n&waitFor));

	ClearBitMask(pInfo,BitFramingReg,0x80);		//清除
	
	//---------------------------检查接收状态并读取数据
	if (i!=0)
	{    
		if(!(mfrc522_read_data(pInfo,ErrorReg)&0x1B))		//读错误状态，检查有无错误
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
			{	
				status = MI_NOTAGERR;
			}
			if (Command == PCD_TRANSCEIVE)	//发送并接收数据命令
			{
				n = mfrc522_read_data(pInfo,FIFOLevelReg);				//读取FIFO中数据个数
				lastBits = mfrc522_read_data(pInfo,ControlReg) & 0x07;		//读取最后接收字节有效位个数
				if (lastBits)
				{
					*pOutLenBit = (n-1)*8 + lastBits;
				}
				else
				{
					*pOutLenBit = n*8;
				}
				if (n == 0)
				{
					n = 1;
				}
				if (n > MAXRLEN)
				{
					n = MAXRLEN;
				}
				for (i=0; i<n; i++)
				{
					pOutData[i] = mfrc522_read_data(pInfo,FIFODataReg);	//从FIFO缓存读取数据
				}
			}
		}
		else
		{
			status = MI_ERR;
		}			
	}
	//---------------------------
	SetBitMask(pInfo,ControlReg,0x80);           		// stop timer now
	mfrc522_write_data(pInfo,CommandReg,PCD_IDLE);	//进入空闲模式
	return status;
}



/////////////////////////////////////////////////////////////////////
//功    能：扣款和充值
//参数说明: dd_mode[IN]：命令字
//               0xC0 = 扣款
//               0xC1 = 充值
//          addr[IN]：钱包地址
//          pValue[IN]：4字节增(减)值，低位在前
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                 
char PcdValue(mfrc522def* pInfo,unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        memcpy(ucComMF522Buf, pValue, 4);
 //       for (i=0; i<16; i++)
 //       {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(pInfo,ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：备份钱包
//参数说明: sourceaddr[IN]：源地址
//          goaladdr[IN]：目标地址
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdBakValue(mfrc522def* pInfo,unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(pInfo,ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}

///////////////////////////////////////////////////////////////////////
// Delay us
///////////////////////////////////////////////////////////////////////
static void mfrc522_delay_us(unsigned short us)
{
	#include "STM32_SYSTICK.H"
	SysTick_DeleyuS(us);				//SysTick延时nmS
}
///////////////////////////////////////////////////////////////////////
// Delay ms
///////////////////////////////////////////////////////////////////////
static void mfrc522_delay_ms(unsigned short ms)
{
	#include "STM32_SYSTICK.H"
	SysTick_DeleymS(ms);				//SysTick延时nmS
}
//------------------------------------------------------------------












//-----------------------------------------------static-hardware
/*******************************************************************************
*函数名			:	function
*功能描述		:	配置读卡器接口
*输入				: 
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_port_configuration(mfrc522def* pInfo)
{  
	#include "STM32_GPIO.H"
	GPIO_Configuration_OPP50	(pInfo->port.nss_port,pInfo->port.nss_pin);		//NSS
	GPIO_Configuration_OPP50	(pInfo->port.clk_port,pInfo->port.clk_pin);		//CLK
	GPIO_Configuration_OPP50	(pInfo->port.mosi_port,pInfo->port.mosi_pin);	//MOSI
	GPIO_Configuration_IPU	(pInfo->port.miso_port,pInfo->port.miso_pin);			//MISO
	
	GPIO_Configuration_OPP50	(pInfo->port.rst_port,pInfo->port.rst_pin);	//RESET
}
//------------------------------------------------------------------


//-----------------------------------------------static-hardware-reset
static void mfrc522_set_rst_high(mfrc522def* pInfo)
{  
	pInfo->port.rst_port->BSRR=pInfo->port.rst_pin;
}
static void mfrc522_set_rst_low(mfrc522def* pInfo)
{  
	pInfo->port.rst_port->BRR=pInfo->port.rst_pin;
}
//-----------------------------------------------static-hardware-nss
static void mfrc522_set_nss_high(mfrc522def* pInfo)
{  
	pInfo->port.nss_port->BSRR=pInfo->port.nss_pin;
}
static void mfrc522_set_nss_low(mfrc522def* pInfo)
{  
	pInfo->port.nss_port->BRR=pInfo->port.nss_pin;
}
//-----------------------------------------------static-hardware-clk
static void mfrc522_set_clk_high(mfrc522def* pInfo)
{  
	pInfo->port.clk_port->BSRR=pInfo->port.clk_pin;
}
static void mfrc522_set_clk_low(mfrc522def* pInfo)
{  
	pInfo->port.clk_port->BRR=pInfo->port.clk_pin;
}
//-----------------------------------------------static-hardware-mosi
static void mfrc522_set_mosi_high(mfrc522def* pInfo)
{  
	pInfo->port.mosi_port->BSRR=pInfo->port.mosi_pin;
}
static void mfrc522_set_mosi_low(mfrc522def* pInfo)
{  
	pInfo->port.mosi_port->BRR=pInfo->port.mosi_pin;
}
//-----------------------------------------------static-hardware-mosi
static unsigned short mfrc522_get_miso(mfrc522def* pInfo)
{ 
	return pInfo->port.miso_port->IDR&pInfo->port.miso_pin;
}
//------------------------------------------------------------------


/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_byte(mfrc522def* pInfo,unsigned char value)
{
	unsigned char i=0;

	//-----------------------------发送地址
	for(i=8;i>0;i--)
	{
		if(0x80	==	(value&0x80))
		{
		 mfrc522_set_mosi_high(pInfo);	//mosi=1
		}
		else
		{
		 mfrc522_set_mosi_low(pInfo);		//mosi=	0
		}
		mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;		
		mfrc522_set_clk_low(pInfo);			//MF522_SCK = 0;
		value <<= 1;
	}
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_data(mfrc522def* pInfo,unsigned char Address,unsigned char value)
{
	unsigned char i, ucAddr;

	mfrc522_set_clk_low(pInfo);		//MF522_SCK = 0;
	mfrc522_set_nss_low(pInfo);		//MF522_NSS = 0;
	ucAddr = ((Address<<1)&0x7E);
	//ucAddr = (Address&0x7F);			//BIT7==0;write
	
	//-----------------------------发送地址--写数据
	mfrc522_write_byte(pInfo,ucAddr);
	//-----------------------------发送数据
	mfrc522_write_byte(pInfo,value);
	//-----------------------------结束
	mfrc522_set_nss_high(pInfo);		//MF522_NSS = 1
	mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	写RC632寄存器
*输入				: Address[IN]:寄存器地址
							value[IN]:写入的值
*返回值			:	无
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static void mfrc522_write_buffer(mfrc522def* pInfo,unsigned char Address,unsigned char* tx_buffer,unsigned char len)
{
	unsigned char i, ucAddr;

	mfrc522_set_clk_low(pInfo);		//MF522_SCK = 0;
	mfrc522_set_nss_low(pInfo);		//MF522_NSS = 0;
	ucAddr = ((Address<<1)&0x7E);
	//ucAddr = (Address&0x7F);			//BIT7==0;write
	
	//-----------------------------发送地址--写数据
	mfrc522_write_byte(pInfo,ucAddr);
	//-----------------------------发送数据
	for(i=0;i<len;i++)
	{
		mfrc522_write_byte(pInfo,tx_buffer[i]);
	}	
	//-----------------------------结束
	mfrc522_set_nss_high(pInfo);		//MF522_NSS = 1
	mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
}

/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_byte(mfrc522def* pInfo)
{
	unsigned char i	=	0;
	unsigned char ucResult=0;
	//-----------------------------读取数据
	for(i=8;i>0;i--)
	{
		mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
		ucResult <<= 1;
		if(mfrc522_get_miso(pInfo))			//miso==1
		{
			ucResult|=0x01;
		}
		mfrc522_set_clk_low(pInfo);			//MF522_SCK = 0;
	}	
	return ucResult;
}

/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_data(mfrc522def* pInfo,unsigned char Address)
{
	unsigned char i	=	0;
	unsigned char ucAddr	=	0;
	unsigned char ucResult=0;

	mfrc522_set_clk_low(pInfo);		//MF522_SCK = 0;
	mfrc522_set_nss_low(pInfo);		//MF522_NSS = 0;
   
	ucAddr = ((Address<<1)&0x7E)|0x80;
	
	//ucAddr = (Address|0x80);			//BIT7==1;read
	//-----------------------------发送地址
	mfrc522_write_byte(pInfo,ucAddr);
	//-----------------------------读取数据
	ucResult	=	mfrc522_read_byte(pInfo);
	//-----------------------------结束
	mfrc522_set_nss_high(pInfo);		//MF522_NSS = 1
	mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
	
	return ucResult;
}
/*******************************************************************************
*函数名			:	mfrc522_read_byte
*功能描述		:	读RC632寄存器
*输入				: Address[IN]:寄存器地址
*返回值			:	读出的值
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
static unsigned char mfrc522_read_buffer(mfrc522def* pInfo,unsigned char Address,unsigned char* rx_buffer,unsigned char len)
{
	unsigned char i	=	0;
	unsigned char ucAddr	=	0;
	unsigned char ucResult=0;

	mfrc522_set_clk_low(pInfo);		//MF522_SCK = 0;
	mfrc522_set_nss_low(pInfo);		//MF522_NSS = 0;
   
	ucAddr = ((Address<<1)&0x7E)|0x80;	
	//ucAddr = (Address|0x80);			//BIT7==1;read
	
	//-----------------------------发送地址
	mfrc522_write_byte(pInfo,ucAddr);
	//-----------------------------读取数据
	for(i=0;i<len;i++)
	{
		rx_buffer[i]=mfrc522_read_byte(pInfo);
	}
	//-----------------------------结束
	mfrc522_set_nss_high(pInfo);		//MF522_NSS = 1
	mfrc522_set_clk_high(pInfo);		//MF522_SCK = 1;
	
	return ucResult;
}
//------------------------------------------------------------------


/*******************************************************************************
*函数名			:	mfrc522_set_reset
*功能描述		:	RC522
*输入				: 
*返回值			:	成功返回MI_OK
*修改时间		:	无
*修改说明		:	无
*注释				:	wegam@sina.com
*******************************************************************************/
char mfrc522_set_reset(mfrc522def* pInfo)
{
	mfrc522_set_rst_high(pInfo);	//MF522_RST=1;
	mfrc522_delay_us(1);
	mfrc522_set_rst_low(pInfo);		//MF522_RST=0;
	mfrc522_delay_us(1);
	mfrc522_set_rst_high(pInfo);	//MF522_RST=1;
	mfrc522_delay_us(10);
	
	mfrc522_write_data(pInfo,CommandReg,PCD_RESETPHASE);   
	mfrc522_write_data(pInfo,ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
	mfrc522_write_data(pInfo,TReloadRegL,30);           
	mfrc522_write_data(pInfo,TReloadRegH,0);
	mfrc522_write_data(pInfo,TModeReg,0x8D);
	mfrc522_write_data(pInfo,TPrescalerReg,0x3E);
	mfrc522_write_data(pInfo,TxAutoReg,0x40);     
	return MI_OK;
}
/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天线发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void mfrc522_set_AntennaOn(mfrc522def* pInfo)
{
    unsigned char i;
    i = mfrc522_read_data(pInfo,TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(pInfo,TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void mfrc522_set_AntennaOff(mfrc522def* pInfo)
{
    ClearBitMask(pInfo,TxControlReg, 0x03);
}
/////////////////////////////////////////////////////////////////////
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char mfrc522_set_Halt(mfrc522def* pInfo)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(pInfo,ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(pInfo,PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}
//------------------------------------------------------------------




