#ifndef __CRC_H
#define __CRC_H












unsigned short CRC16_CCITT(unsigned char *puchMsg, unsigned int usDataLen);				//����ʽx16+x12+x5+1��0x1021������ʼֵ0x0000����λ��ǰ����λ�ں󣬽����0x0000���
unsigned short CRC16_CCITT_FALSE(unsigned char *puchMsg, unsigned int usDataLen);	//����ʽx16+x12+x5+1��0x1021������ʼֵ0xFFFF����λ�ں󣬸�λ��ǰ�������0x0000���
unsigned short CRC16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen);				//����ʽx16+x12+x5+1��0x1021������ʼֵ0x0000����λ�ں󣬸�λ��ǰ�������0x0000���
unsigned short CRC16_X25(unsigned char *puchMsg, unsigned int usDataLen);					//����ʽx16+x12+x5+1��0x1021������ʼֵ0x0000����λ��ǰ����λ�ں󣬽����0xFFFF���
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen);				//����ʽx16+x15+x5+1��0x8005������ʼֵ0xFFFF����λ��ǰ����λ�ں󣬽����0x0000���
unsigned short CRC16_IBM(unsigned char *puchMsg, unsigned int usDataLen);					//����ʽx16+x15+x5+1��0x8005������ʼֵ0x0000����λ��ǰ����λ�ں󣬽����0x0000���
unsigned short CRC16_MAXIM(unsigned char *puchMsg, unsigned int usDataLen);				//����ʽx16+x15+x5+1��0x8005������ʼֵ0x0000����λ��ǰ����λ�ں󣬽����0xFFFF���
unsigned short CRC16_USB(unsigned char *puchMsg, unsigned int usDataLen);					//����ʽx16+x15+x5+1��0x8005������ʼֵ0xFFFF����λ��ǰ����λ�ں󣬽����0xFFFF���


unsigned char CRC8_8541_msb(unsigned char *puchMsg, unsigned int usDataLen);			//CRC8_8541_msb ��λ��ǰ
unsigned char CRC8_8541_lsb(unsigned char *puchMsg, unsigned int usDataLen);			//CRC8_8541_lsb��λ��ǰ
#endif


