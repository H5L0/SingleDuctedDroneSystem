#include "Model.h"
#include "Synchronizer.h"
#include "Storage.h"
#include "Beeper.h"


/*
class Drone
{

	void GetStatus();


	void SetControl();

};
*/

Model model;
Synchronizer synch;


//------------------------------- Init Function -------------------------------//

/*
void InitProperty()
{
	//...
	//Command
	//
	//Command.throttle = 0;


	//PID.X
	//PID.RX.kp = 0.1f;   //0.1
	//PID.RX.ki = 0;
	//PID.RX.kd = 0.05f;   //0.05
	//
	//PID.VX.kp = 2.0f;
	//PID.VX.ki = 1.0f;
	//PID.VX.kd = 0;
	//
	////PID.Y
	//PID.RY.kp = 0.1f;
	//PID.RY.ki = 0;
	//PID.RY.kd = 0.05f;
	//
	//PID.VY.kp = 2.0f;
	//PID.VY.ki = 1.0f;
	//PID.VY.kd = 0;

	PID.RX.kp = 0.2f;
	PID.RX.ki = 0.0f;  //=> 1-4
	PID.RX.kd = 0.0f;  //=> 0-1
	
	PID.VX.kp = 1.0f;
	PID.VX.ki = 0.0f;
	PID.VX.kd = 0.0;

	//PID.Y
	PID.RY.kp = 0.2f;
	PID.RY.ki = 0.0f;
	PID.RY.kd = 0.0f;

	PID.VY.kp = 1.0f;
	PID.VY.ki = 0.0f;
	PID.VY.kd = 0.0f;

	//PID.Z
	//PID.RZ.kp = FP16_1 * 2;
	//PID.RZ.ki = FP16_H5;
	//PID.RZ.kd = FP16_H5;
	//
	//PID.VZ.kp = FP16_1 * 2;
	//PID.VZ.ki = FP16_1;
	//PID.VZ.kd = 0;

	PID.RZ.kp = 0.2f;
	PID.RZ.ki = 0;
	PID.RZ.kd = 0;

	PID.VZ.kp = 2.0f;
	PID.VZ.ki = 1.0f;
	PID.VZ.kd = 0;

	// **DEBUG** //
	//DeleteStored({ 0xFF, 0xFF });

	//读取之前储存的数据
	LoadBytes(&StoreInfo, EADDR_INFO, ESIZE_INFO);

	//判断有无储存过数据
	if(StoreInfo.key == EKEY)
	{
		PersistParameters({ StoreInfo.flags, StoreInfo.pid_flags }, false);
	}
	else
	{
		StoreInfo.key = EKEY;
		StoreInfo.flags = 0;
		StoreInfo.pid_flags = 0;
	}

	//启动时电源默认关闭
	state = eState_PowerOff;
}
*/




//------------------------ Arduino Schedule Function -----------------------//

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	//设置参数初始值
	u8 cm_flags;
	u8 pid_flags;
	//检索有无储存值
	Storage::DataExists(cm_flags, pid_flags);
	//加载储存值
	Storage::LoadParameters(model, cm_flags);
	Storage::LoadPIDs(model.GetPIDs(), pid_flags);
	//加载默认值
	Storage::ResetParameters(model, ~cm_flags);
	Storage::ResetPIDs(model.GetPIDs(), ~pid_flags);

	Beeper::Init();

	bool bSynch = synch.Init();

	bool bModel = model.Init();

	if(!bSynch || !bModel) while(true);

	LOGF("Waiting for command...");

	synch.Connect(model);
}



void loop()
{
	synch.Sync(model);  //接收命令 => 发送之前请求的数据 => 处理接收的命令 => 发送当前命令执行结果/发送当前请求的数据

	model.Update();

	synch.LateSync(model); //延迟发送的回传

	Beeper::Update(model.status.delta_time_1024);
}
