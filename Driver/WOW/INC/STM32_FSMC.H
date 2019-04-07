/******************************** User_library *********************************
* 文件名 	: STM32_SDCard.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __STM32_FSMC_H
#define __STM32_FSMC_H

void FSMC_Initialize(void);
//------------------------------api_configuration
void api_fsmc_sram_configuration(unsigned long address);

//------------------------------api_data
void api_fsmc_write_byte(unsigned long address,unsigned char Data);
unsigned char api_fsmc_read_byte(unsigned long address);
void api_fsmc_write_word(unsigned long address,unsigned short Data);
unsigned short api_fsmc_read_word(unsigned long address);

//------------------------------api_buffer
void api_fsmc_write_buffer_byte(unsigned long startaddress,unsigned char* pbuffer,unsigned short len);
unsigned short api_fsmc_read_buffer_byte(unsigned long startaddress,unsigned char* pbuffer,unsigned short len);
void api_fsmc_write_buffer_word(unsigned long startaddress,unsigned short* pbuffer,unsigned short len);
unsigned short api_fsmc_read_buffer_word(unsigned long startaddress,unsigned short* pbuffer,unsigned short len);

//------------------------------static
static void fsmc_gpio_initialize(unsigned long address);
static void fsmc_sram_initialize(unsigned long address);


#endif




