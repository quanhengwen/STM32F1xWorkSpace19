#ifndef __AMP01V11A3_H
#define __AMP01V11A3_H


#include	"AMP_PHY.H"

#define AmpArrySize    	30       		//每个发送端口待发送缓存排序个数
#define AmpFramesize   	256       		//每个发送端口待发送缓存排序个数

typedef struct _ampCache       //拨码数据：
{
  unsigned char arry;           		//发送排序：0时为空，从第1个开始发送，发送完一个，排序减1，直到0，增加时，增加一个待发送缓存，新缓存排最后
  unsigned char size;           		//当前缓存有效数据个数(待发送字节数)
  unsigned char data[AmpFramesize];	//缓存
}ampCachedef;
typedef struct _ampCC       //拨码数据：
{
  ampCachedef	pc[AmpArrySize];
	ampCachedef	cab[AmpArrySize];
	ampCachedef	lay[AmpArrySize];
}ampCCdef;
typedef struct _amp
{
  stTimesdef        Time;         //相关时间变量
//  stSWdef           SwData;       //主机标志
  stReqdef          Req;         //请求类标志
  stStadef          Sta;          //状态码
  stFlgdef          Flag;         //相关标志位
  stAckQdef         AckQ;         //标志应答标志，0-不需要，1-需要应答相应接口
  stBufferdef       buffer;       //串口数据缓存
  stSendCountdef    ReSend;
	ampCCdef					CCdata;
}ampdef;


extern stAMPProdef   AMPPro;

void AMP01V11A3_Configuration(void);
void AMP01V11A3_Server(void);

//void AMP01V11_Loop(void);


unsigned short HW_SendBuff(enCCPortDef Port,unsigned char* pBuffer,unsigned short length);
//=================================时间函数ST==============================================================
void Tim_Server(void);


#endif
