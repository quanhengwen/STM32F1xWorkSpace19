#ifndef __BQ26100Master_H
#define __BQ26100Master_H 
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





void api_bq26100Master_configuration(void);
void api_bq26100Master_server(void);


unsigned char api_bq26100Master_get_digest(const unsigned char* message,unsigned char* digest);
	
sdq_result api_bq26100Master_send_message(const unsigned char* message);
sdq_result api_bq26100Master_write_control(void);
sdq_result api_bq26100Master_read_control(void);
sdq_result api_bq26100Master_read_digest(unsigned char* digest);




static void bq26100Master_write_bit(unsigned char bit);
static unsigned char	bq26100Master_read_bit(void);
static void	bq26100Master_write_byte(unsigned char dat);
static unsigned char bq26100Master_read_byte(void);
sdq_result api_bq26100Master_rest(void);




#endif

