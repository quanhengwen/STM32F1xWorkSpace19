#ifndef __BQ26100_H
#define __BQ26100_H 
#include "stdint.h"



typedef enum
{
	sdq_error		=	0,
	sdq_success	=	1,
}sdq_result;
typedef enum
{
	sdq_rom_readID		=	0x33,			//低位到高位传输
	sdq_rom_matchID		=	0x55,			//
	sdq_rom_skipID		=	0xCC,			//
	sdq_rom_searchID	=	0xF0			//查找ROM(F0H)
}sdq_rom_cmd;

typedef enum
{
	sdq_mem_read_memory		=	0xF0,			//
	sdq_mem_write_memory	=	0x0F,
	sdq_mem_read_page			=	0xC3,			//
	sdq_mem_read_digest		=	0xDD,			//
	sdq_mem_write_message	=	0x22,			//
	sdq_mem_read_status		=	0xAA,
	sdq_mem_write_status	=	0x55,
	sdq_mem_read_control	=	0x88,
	sdq_mem_write_control	=	0x77,
	sdq_mem_read_eeprom		=	0xE0,
	sdq_mem_write_eeprom	=	0x0E,
	sdq_mem_profile				=	0x99,
	sdq_mem_read_page4		=	0xFA,
	sdq_mem_write_page4		=	0xAF,
}sdq_mem_cmd;
//typedef struct
//{
//	//____________SPI_CS定义
//	GPIO_TypeDef* 	nss_port;		//GPIOX
//	unsigned short 	nss_pin;			//GPIO_Pin_x
//}bq26100_port;
typedef struct
{
	unsigned char uid[6];
}bq26100_def;

void api_bq26100_configuration(void);
sdq_result api_bq26100_read_id(unsigned char* pbuffer);	//读ID0x33
sdq_result api_bq26100_skip_id(void);	//0xCC
sdq_result api_bq26100_read_memory(unsigned char* pbuffer);
sdq_result api_bq26100_write_memory(unsigned char* pbuffer);
sdq_result api_bq26100_read_page(unsigned char* pbuffer);
sdq_result api_bq26100_read_status(unsigned char* pbuffer);
sdq_result api_bq26100_write_status(unsigned char* pbuffer);



sdq_result bq26100_step1_send_message(const unsigned char* message,unsigned char len);
sdq_result bq26100_step2_write_control(void);
sdq_result bq26100_step3_read_control(void);
sdq_result bq26100_step4_read_digest(unsigned char* digest);
void bq26100_step5_Auth(void);
sdq_result bq26100_step6_verify(unsigned char* digest);




void api_bq26100_test_example(void);
unsigned char api_bq26100_test_example_getkey(void);
unsigned long api_bq26100_test_example_getkey2(unsigned long* H,unsigned long* ARRY);

sdq_result bq26100_rest(void);	//复位Dallas,返回结果
void sdq_write_byte(unsigned char dat);
unsigned char sdq_read_byte(void);

static void bq26100_change_message(void);

void api_bq26100_set_message2(unsigned char* buffer);
void api_bq26100_test_example22(unsigned long* buffer);

unsigned char api_bq26100_get_digest(const unsigned char* message,unsigned char* digest);


#endif

