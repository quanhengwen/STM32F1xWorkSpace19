#include "WDH320S.H"

#include "stdio.h"
#include "string.h"

/*通信命令的枚举量*/
enum MESSAGE_CMD
{
    CMD_ONE_VS_N  = 0,                //采集特征并1：N比对
    CMD_ONE_VS_G,                     //采集特征并1：G比对
    CMD_ONE_VS_ONE,                   //采集特征并1：1比对
    CMD_REGISTER,                     //注册手指静脉
    CMD_REG_END,                      //结束注册
    CMD_DELETE_ONE,                   //删除单个手指
    CMD_DELETE_ALL,                   //删除所有信息
    CMD_UPLOAD_ALL_ID,                //上传所有手指ID
    CMD_UPLOAD_INFOR,                 //上传指定手指信息
    CMD_UPLOAD_TEMPLATE,              //上传指定手指模板
    CMD_UPLOAD_INFOR_TEMPLATES,       //上传指定手指以及对应模板
    CMD_CREATE_TEMPLATE,              //采集并上传模板
    CMD_DOWNLOAD_INFOR_TEMPLATES,     //下载手指信息头和所有模板
    CMD_UPLOAD_VERSION,               //获取固件版本号
    CMD_UPLOAD_COUNT,                 //获取手指注册数量
    CMD_CHK_FINGER,                   //检查手指状态
    CMD_UPLOAD_SEQUENCE,              //获取序列号
    CMD_SET_BAUD,                     //设置波特率
    CMD_SET_DEVID,                    //设置设备编号
};

//通讯特殊字节
#define ASCII_XON           (0x40)
#define ASCII_XON_DATA      (0x3e)
#define ASCII_XOFF          (0x0D)
#define BROADCASTADDR       (0xff)

//发送命令包
typedef struct
{
	unsigned char Start;	//帧头
	unsigned char Cmd;      //指令码
	unsigned char Devid;	//设备号
	unsigned char P1;		//参数 1 或 数据长度低字节
	unsigned char P2;		//参数 2 或 数据长度高字节
	unsigned char P3;		//参数 3
	unsigned char CHK;      //校验和
	unsigned char End;      //帧尾
}*P_CMD_PACK_SEND;

//接收命令包
typedef struct
{
	unsigned char Start;	//帧头
	unsigned char Cmd;      //指令码
	unsigned char Devid;	//设备号
	unsigned char Q1;       //参数 1 或 数据长度低字节
	unsigned char Q2;       //参数 2 或 数据长度高字节
	unsigned char Q3;       //命令返回值
	unsigned char CHK;      //校验和
	unsigned char End;      //帧尾
}*P_CMD_PACK_RECV;

//数据包头部
typedef struct
{
	unsigned char Start;	//帧头
	unsigned char Cmd;	//指令码
	unsigned char Devid;	//设备号
	unsigned char *Data; //指向附加数据开始位置的指针
}*P_DATA_PACK_HEAD;

//数据包尾部
typedef struct
{
	unsigned char CHK;	//校验和
	unsigned char End;	//帧尾
}*P_DATA_PACK_TAIL;


//计算校验和
unsigned char api_wdh320s_get_check_xor(unsigned char *data,unsigned int len)
{
	unsigned char chk = *data;
	unsigned long i	=	0;
	for(i=1;i<len;i++)
	{
		chk ^= *(data+i);
	}
	return chk;
}
/*
 * @brief api_wdh320s_get_pack_len【得到发送包长度】
 * @param cmd(I)            包命令
 * @param Extra_DataLen(I)  包中携带的主要数据长度，参考通讯协议
 * @return 发送包长度
 */
int api_wdh320s_get_pack_len(int cmd, unsigned int Extra_DataLen)
{
    int retlen = 0;

    //所有发送包只有 CMD_DOWNLOAD_INFOR_TEMPLATES 有附加数据
    if(CMD_DOWNLOAD_INFOR_TEMPLATES == cmd)
    {
        //校验附加长度是否正确
        if((Extra_DataLen - 18) % 512 != 0)
        {
            //dbgInfoPrint("附加数据长度错误.");
            return -1;
        }
        else
        {
            retlen = CMD_PACK_SIZE + DATA_PACK_HEAD_SIZE + Extra_DataLen + DATA_PACK_TAIL_SIZE;
        }
    }
    else
    {
        retlen = CMD_PACK_SIZE;
    }
    //dbgInfoPrint("发送包长度应为 %d 字节.", retlen);
    return retlen;
}


/**
 * @brief api_wdh320s_get_send_pack【打包一个发送包】
 * @param p(O)             发送包缓冲区
 * @param _cmd(I)          包命令
 * @param _device(I)       设备ID
 * @param _p1(I)           参数1 由具体命令决定
 * @param _p2(I)           参数2 由具体命令决定
 * @param _p3(I)           参数3 由具体命令决定
 * @param Extra_Data(I)    附加数据
 * @param Extra_DataLen(I) 附加数据的长度
 * @return 发送包长度
 */
int api_wdh320s_get_send_pack(unsigned char * p,
                    unsigned char _cmd,
                    unsigned char _device,
                    unsigned char _p1,
                    unsigned char _p2,
                    unsigned char _p3,
                    unsigned char *Extra_Data,
                    unsigned int  Extra_DataLen)
{
    //返回值-发送包大小
    int retlen = 0;
	unsigned char *databuf	= NULL;
	P_DATA_PACK_TAIL datatail;
    //打包-命令包
    P_CMD_PACK_SEND head = (P_CMD_PACK_SEND)p;
    head->Start = ASCII_XON;	//帧头
    head->Cmd = _cmd;           //指令码
    head->Devid = _device;      //设备号
    head->P1 = _p1;             //参数 1
    head->P2 = _p2;             //参数 2
    head->P3 = _p3;             //参数 3
    head->CHK = api_wdh320s_get_check_xor((unsigned char *)head, 6);	//校验和
    head->End = ASCII_XOFF;     //帧尾

    //dbgInfoPrint("设置发送包的命令包成功.");
    retlen += CMD_PACK_SIZE;
    //打包-数据包
    if(NULL != Extra_Data && 0 != Extra_DataLen)
    {
        //附加数据头
        P_DATA_PACK_HEAD datahead = (P_DATA_PACK_HEAD)(p + CMD_PACK_SIZE);
        datahead->Start = ASCII_XON_DATA;
        datahead->Cmd = _cmd;
        datahead->Devid = _device;

        //附加数据
		databuf = p + CMD_PACK_SIZE +DATA_PACK_HEAD_SIZE;
        memcpy(databuf,Extra_Data,Extra_DataLen);

        //附加数据尾
		datatail = (P_DATA_PACK_TAIL)(p + CMD_PACK_SIZE + DATA_PACK_HEAD_SIZE + Extra_DataLen);
        datatail->CHK = api_wdh320s_get_check_xor((unsigned char *)datahead, DATA_PACK_HEAD_SIZE + Extra_DataLen);
        datatail->End = ASCII_XOFF;

        //dbgInfoPrint("设置发送包的数据包成功,附加数据共 %d 字节.", Extra_DataLen);
        retlen += DATA_PACK_HEAD_SIZE + Extra_DataLen + DATA_PACK_TAIL_SIZE;
    }
    //dbgInfoPrint("输出发送包，共 %d 字节.", retlen);
    return retlen;
}


/**
 * @brief api_wdh320s_get_pack_data【获得接收包中的主要数据】
 * @param pack(I)      接收到的数据包
 * @param ebuf(O)      主要数据内容
 * @param ebuflen(IO)  输入为缓冲区大小，输出为主要数据内容长度
 * @param special(O)   携带内容，针对通过命令包返回的数据，special通常为整形变量的地址
 *                     如以下几个命令：
 *                       CMD_ONE_VS_N
 *                       CMD_ONE_VS_ONE
 *                       CMD_ONE_VS_G
 *                       CMD_UPLOAD_COUNT
 *                     祥见通讯协议
 * @return 命令执行的结果 或 包错误码
 */
int api_wdh320s_get_pack_data(void *pack, void *ebuf, unsigned int *ebuflen, int *special)
{
    unsigned char *p = (unsigned char *)pack;                           //包数据按字节数组处理

    P_CMD_PACK_RECV cmdpack = (P_CMD_PACK_RECV)p;                       //指向-命令包
    unsigned char *datapack =  p + CMD_PACK_SIZE;                       //指向-数据包
    unsigned char *databuf = p + CMD_PACK_SIZE + DATA_PACK_HEAD_SIZE;   //指向-数据包中的数据

    //1. 计算包中的附加数据长度
    unsigned int extarlen = 0;
    //以下命令Q1和Q2不表示附加数据长度，且他们的附加数据为0
    if(	cmdpack->Cmd == CMD_ONE_VS_N 		||	//表示FID
		cmdpack->Cmd == CMD_ONE_VS_ONE 		|| 	//表示FID
		cmdpack->Cmd == CMD_ONE_VS_G 		||	//表示FID
		cmdpack->Cmd == CMD_UPLOAD_COUNT 	||	//表示注册个数
		cmdpack->Cmd == CMD_CHK_FINGER 		||	//表示手指状态
		cmdpack->Cmd == CMD_REGISTER)   		//320为表示附加数据长度0，330表示手指倾斜的角度
    {
        extarlen = 0;
    }
    //除以上命令外Q1和Q2表示附加数据长度
    else
    {
        extarlen = cmdpack->Q1 + cmdpack->Q2*256;
    }

    //2.校验用户缓冲区长度
    //dbgInfoPrint("0X%02X 附加数据主要内容长度：%d, 传入缓冲区为：%d",\
                 cmdpack->Cmd, extarlen, ebuflen?*ebuflen:0);
    if((ebuflen?*ebuflen:0) < extarlen)
    {
        //dbgInfoPrint("输入缓冲区太小.");
        return BUF_SMALL;
    }
    else
    {
        memset(ebuf, 0x00, ebuflen?*ebuflen:0);
    }

    //3. 校验-命令包是否正确
    if(api_wdh320s_get_check_xor((unsigned char *)cmdpack, 6) != cmdpack->CHK)
    {
        //dbgInfoPrint("数据包头已损坏.");
        return  PACK_ISBAD;
    }
    //dbgInfoPrint("数据包头校验通过.");

    //4. 校验-数据包是否正确
    if(	cmdpack->Cmd == CMD_ONE_VS_N 	||	//携带FID
		cmdpack->Cmd == CMD_ONE_VS_ONE	||	//携带FID
		cmdpack->Cmd == CMD_ONE_VS_G 	||	//携带FID
		cmdpack->Cmd == CMD_UPLOAD_COUNT||	//携带注册个数
		cmdpack->Cmd == CMD_CHK_FINGER	||	//表示手指状态
		cmdpack->Cmd == CMD_REGISTER)      	//320为表示附加数据长度0，330表示手指倾斜的角度
    {
        //dbgInfoPrint("数据包头携带数据.");
    }
    else if(extarlen != 0)
    {
        if(api_wdh320s_get_check_xor((unsigned char *)datapack, extarlen + 3) != *(datapack +extarlen + 3))
        {
            //dbgInfoPrint("数据包数据已损坏.");
            return PACK_ISBAD;
        }
        //dbgInfoPrint("数据包数据校验通过.");
    }
    else
    {
        //dbgInfoPrint("无附加数据.");
    }
    //5. 拷贝附加数据并返回附加数据的长度
    //以下三个个命令通过命令包传回用户ID
    if(	cmdpack->Cmd == CMD_ONE_VS_N 		||	//携带FID
		cmdpack->Cmd == CMD_ONE_VS_ONE 		||	//携带FID
		cmdpack->Cmd == CMD_ONE_VS_G 		||	//携带FID
		cmdpack->Cmd == CMD_UPLOAD_COUNT 	|| 	//携带注册个数
		cmdpack->Cmd == CMD_REGISTER)         	//320为表示附加数据长度0，330表示手指倾斜的角度
    {
        if(special != NULL)
        {
            *special = cmdpack->Q1 + cmdpack->Q2*256;
            //dbgInfoPrint("提取命令包附加数据成功，值=%d.", *special);
        }
    }
    //常规附加数据及长度
    else
    {
        if(ebuf != NULL)
        {
			memcpy(ebuf ,databuf, extarlen);
            //dbgInfoPrint("拷贝数据包附加数据成功.");
        }
        if(ebuflen != NULL)
        {
            *ebuflen = extarlen;
        }
    }
    //返回值-命令执行结果
    //dbgInfoPrint("返回值命令执行结果: %d", cmdpack->Q3);
    return cmdpack->Q3;
}
/////////////////////////////////////////////////////////////////////////


/* 使用方法 */
/*
void func()
{
    //1.定义一个发送包，并设置发送数据
    //  方法一（对于没有附加数据的发送包）
    unsigned char sendpack[CMD_PACK_SIZE] = {0};
    GetSendPack(sendpack, CMD_CHK_FINGER, ui->comboBox_Devid->currentText().toInt(), 0, 0, 0);

    //  方法二（适用所有发送包）
    int lenx = api_wdh320s_get_pack_len(CMD_CHK_FINGER, 0);                   //根据命令和所携带的附加数据获得发送包的大小
    if(lenx < 0)
    {
        return -1;
    }
    unsigned char * sendpack = (unsigned char *)calloc(1, lenx);    //分配发送包内存

    int len = api_wdh320s_get_send_pack(sendpack,                                 //设置发送包
                    CMD_CHK_FINGER,
                    ui->comboBox_Devid->currentText().toInt(),
                    0,
                    0,
                    0,
                    NULL,                                           //发送包中携带的数据
                    0);                                             //携带数据长度


    //2.定义一个接收缓冲区
    unsigned char *recvdata = ...;


    //3.使用你的方法接收返回的数据。
    you_recv(recvdata)


    //4.把接收的数据丢给此函数解析，来获取发送的命令操作结果，或者带回的附加数据内容。
    int rel = api_wdh320s_get_pack_data(recvdata, NULL, NULL, NULL);
    if(rel < 0)
    {
        cout << "数据包损坏";
    }

    //do what you want to do ...
}

*/


