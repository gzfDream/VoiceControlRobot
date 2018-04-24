#pragma once
/*语音识别*/
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
	int     build_fini;  //标识语法构建是否完成
	int     update_fini; //标识更新词典是否完成
	int     errcode; //记录语法构建或更新词典回调错误码
	char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
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
	const char *login_config; //登录参数
	UserData asr_data;
	int ret = 0;

};

