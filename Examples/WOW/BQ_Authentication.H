#ifndef __BQ_Authentication_H
#define __BQ_Authentication_H


void BQ_Authentication_Configuration(void);
void BQ_Authentication_Server(void);



static void BQ_Authentication_SYS_Configuration(void);

static void BQ_Authentication_Test_Model_WorkAsSMT(void);
static void BQ_Authentication_Test_Model_OnLine(void);
static void BQ_Authentication_Auto_Reset_OnLine(void);		//认证失败自动重启
static void BQ_Authentication_Running_Model_OnLine(void);	//正常运行模式：认证失败自动重启




#endif

