#pragma once
/*����ʶ��*/
#include "include/qisr.h"
#include "include/msp_cmn.h"
#include "include/msp_errors.h"
#include "asr_record/include/speech_recognizer.h"
#include <conio.h>
#include <errno.h>
#include <process.h>

#define SAMPLE_RATE_16K     (16000)
#define SAMPLE_RATE_8K      (8000)
#define MAX_GRAMMARID_LEN   (32)
#define MAX_PARAMS_LEN      (1024)

typedef struct _UserData {
	int     build_fini;  //��ʶ�﷨�����Ƿ����
	int     update_fini; //��ʶ���´ʵ��Ƿ����
	int     errcode; //��¼�﷨��������´ʵ�ص�������
	char    grammar_id[MAX_GRAMMARID_LEN]; //�����﷨�������ص��﷨ID
}UserData;

class Voice_process
{
public:
	Voice_process();
	~Voice_process();

public:
	bool init();
	int run();

private:
	const char *login_config; //��¼����
	UserData asr_data;
	int ret = 0;

};

