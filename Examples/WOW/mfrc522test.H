#ifndef __mfrc522test_H
#define __mfrc522test_H


void mfrc522test_Configuration(void);
void mfrc522test_Server(void);


static void hardware_configuration(void);


static void sys_led_conf(void);
static void sys_led_run(void);

void MFRC522_LOOP(void);

#endif
