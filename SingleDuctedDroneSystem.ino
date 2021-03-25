#include "Model.h"
#include "Synchronizer.h"
#include "Storage.h"
#include "Beeper.h"


Model model;
Synchronizer synch;


//------------------------ Arduino Schedule Function -----------------------//

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	Beeper::Init();
	//Beeper::Beep(0b11101101, 8, true);

	bool bModel = model.Init();

	bool bSynch = synch.Init();

	if(!bModel || !bSynch) while(true);

	LOGF("Waiting for command...");
	Beeper::Beep(0b11, 2, true);

	synch.WaitConnect(model);
}



void loop()
{
	synch.Sync(model);  // (*)接收命令 => 发送之前请求的数据 => 处理接收的命令 => 发送当前命令执行结果/发送当前请求的数据

	model.Update();

	synch.LateSync(model); //处理延迟发送的回传

	Beeper::Update(model.status.delta_time_1024);
}
