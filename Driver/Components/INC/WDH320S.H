#ifndef WDH320S_H
#define WDH320S_H

#define BUF_SMALL               -1  //���뻺����̫С
#define PACK_ISBAD              -2  //���ݰ���������
#define CMD_PACK_SIZE           8   //命令包大�?
#define DATA_PACK_HEAD_SIZE     3   //数据包头部大�?
#define DATA_PACK_TAIL_SIZE     2   //数据包尾部大�?


//#include <QDebug>

//inline void dbgTxtPrint(const char *txt)
//{
//#if defined(_MSC_VER)
//    OutputDebugStringA(txt);
//#elif defined(QT_VERSION)
//    qDebug() << txt;
//#else
//    fprintf(stderr, "%s\r\n", txt);
//#ifdef ANDROID_DEBUG
//    #define  LOG_TAG    "WEDONE_NDK_BIOVEIN"
//    #define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//    LOGD("%s\n", txt);
//#endif
//#endif
//    return;
//}
//#define dbgInfoPrint(format, ...) {\
//        char LOG__LINE__[10240] = {0}; \
//        snprintf(LOG__LINE__, 10240, "[%s:%04d]: " format,  __FUNCTION__ , __LINE__, ##__VA_ARGS__);\
//        dbgTxtPrint(LOG__LINE__);}



//计算校验�?
unsigned char api_wdh320s_get_check_xor(unsigned char *data,unsigned int len);

//得到发送包长度
int api_wdh320s_get_pack_len(int cmd, unsigned int Extra_DataLen);

//得到接收包中主要数据长度
#define GetExtraDataLen(packlen) (packlen <= CMD_PACK_SIZE) ? 0 : \
        packlen - CMD_PACK_SIZE - DATA_PACK_HEAD_SIZE - DATA_PACK_TAIL_SIZE;

//设置发送包
//1.方法一(只适用于无附加数据的发送包�?
/**
 * @brief GetSendPack   【设置一个无附加数据的发送包�?
 * @param p(O)             发送包缓冲�?
 * @param _cmd(I)          包命�?
 * @param _device(I)       设备ID
 * @param _p1(I)           参数1 由具体命令决�?
 * @param _p2(I)           参数2 由具体命令决�?
 * @param _p3(I)           参数3 由具体命令决�?
 */
#define GetSendPack(_pack, _cmd, _device, _p1, _p2, _p3) \
    _pack[0] = ASCII_XON; \
    _pack[1] = _cmd; \
    _pack[2] = _device; \
    _pack[3] = _p1; \
    _pack[4] = _p2; \
    _pack[5] = _p3; \
    _pack[6] = api_wdh320s_get_check_xor(&_pack[0], 6); \
    _pack[7] = ASCII_XOFF


//2.方法�?适用于所有发送包�?
int api_wdh320s_get_send_pack(unsigned char * p, unsigned char _cmd, unsigned char _device,
                    unsigned char _p1, unsigned char _p2, unsigned char _p3,
                    unsigned char *Extra_Data, unsigned int  Extra_DataLen);


//获得接收包中的主要数�?
int api_wdh320s_get_pack_data(void *pack, void *ebuf, unsigned int *ebuflen, int *special);


#endif // PACK_H

