#pragma once
#include "HL.Types.h"

#include "Pins.h"
#include "Beeper.h"
#include "RF24.h"

#include "Model.h"
#include "Storage.h"


#define BUFFER_SIZE 8

class Synchronizer
{

	//-------------------------------- Command Enum --------------------------------//

	enum PackageType
	{
		etPackage_None = 0b00,
		etPackage_Control = 0b01,
		etPackage_Config = 0b10,
		etPackage_Unknown = 0b11,
	};

	enum ConfigCommandType
	{
		etConfig_Hello = 0x01,
		etConfig_ByeBye = 0x02,
		etConfig_Get = 0x03,  //获取参数

		etConfig_StateToken = 0x05,  //无人机状态转移符
		etConfig_ActionAngle = 0x08,  //控制角度幅度
		etConfig_Calibrate = 0x0A,  //以当前姿态为基准校正MPU偏移

		etConfig_Throttle = 0x11,  //油门最大/最小/开始值
		etConfig_RudderBias = 0x21,
		etConfig_RudderRatio = 0x22,

		etConfig_EnablePID = 0x41,
		etConfig_DisablePID = 0x42,
		etConfig_SetCascade = 0x45,
		//etConfig_EnableCascade = 0x45,
		//etConfig_DisableCascade = 0x46,
		etConfig_ConfigPID = 0x4A,

		etConfig_SetPID = 0x50,  //设置PID的参数
		etConfig_SetPID_1 = 0x51,
		//etConfig_ZeroPID   = 0x52,  //清空PID的积分
		//etConfig_StorePID = 0x5A,
		//etConfig_LoadPID = 0x5B,
		//etConfig_ResetPID = 0x5E,
		//etConfig_DeletePID = 0x5F,

		etConfig_Store = 0x8A,  //储存某参数到EEPROM
		etConfig_Load = 0x8B,  //从EEPROM读取数据
		etConfig_Reset = 0x8E,  //重置所有参数
		etConfig_Delete = 0x8F,  //清除储存到EEPROM的参数

		etConfig_Log = 0xB1,  //控制打印的信息

	};


	enum RequestFeedbackType
	{
		etFeedback_None = 0x00,
		etFeedback_Ack = 0x01,

		etFeedback_Fail = 0xFF,
		etFeedback_Error = 0xFE,
		etFeedback_UnknownPackageType = 0xFD,
		etFeedback_UnknownConfigCommand = 0xFC,

		etFeedback_State = 0x10,
		etFeedback_Angles = 0x11,
		etFeedback_AngularVelocity = 0x12,
		etFeedback_Acceleration = 0x13,
		etFeedback_Battery = 0x1A,
		etFeedback_ReceiveCount = 0x1C,

		etFeedback_ThrottleProperty = 0x21,
		etFeedback_RuddersProperty = 0x22,
		etFeedback_Config = 0x23,
		etFeedback_LogSwitch = 0x24,

		etFeedback_PIDParameters = 0x31,

		etFeedback_StoreInfo = 0x41,

		etFeedback_RudderAngles = 0x61,
		etFeedback_PIDOutput = 0x62,
		etFeedback_TimeCycle = 0x6A,
	};


	//状态转移符:   PowerOn        Unlock       Launch
	//              0x01          0xFA         0x11
	//     PowerOff <=>  LockMode <=> SafeMode <=> FlightMode
	//        ^     0x88    |     0xF0   |     0xFE
	//        |   PowerOff  |     Lock   |    SafeMode
	//        \-------------/------------/ 0x88 PowerOff
	enum StateTransferToken
	{
		etStateToken_PowerOn = 0x01, //? => 锁定模式
		etStateToken_PowerOff = 0x88, //锁定模式/安全模式 => 

		etStateToken_Lock = 0xF0,   //安全模式 => 锁定模式
		etStateToken_Unlock = 0xFA,   //锁定模式 => 安全模式

		etStateToken_Launch = 0x11, //安全模式 => 飞行模式
		etStateToken_SafeMode = 0xFE, //飞行模式 => 安全模式
	};


	//---------------------------------- Package --------------------------------//

	struct PackageHead
	{
		PackageType package_type : 2;
		u8 need_ack : 1;
		u8 flag : 5;
	};


	struct ConfigCommandPackage
	{
		PackageHead head;
		u8 command;

		union
		{
			u8 value_u8;   //: StateToken, AngleShift, RudderRatio, RequestFeedbackType
			u16 value_u16;
			u32 value_u32;
			u8 bytes[6];
			s16 s16s[3];

			struct ThrottlePropertyStruct
			{
				u16 start;
				u16 range;
				//u16 start;
			}throttle;

			struct RudderCenterStruct
			{
				u8 index;
				u16 value;
			}rudder_center;

			struct PIDConfigureStruct
			{
				u8 index;
				u8 id;
				fp32 value;
			}pid;

			struct StoreStruct
			{
				u8 cm_flags;
				u8 pid_flags;
			}store;

			struct GetStruct
			{
				u8 target;
				u8 index;
			}get;
		};
	};


	struct ControlCommandPackage
	{
		PackageHead head;
		u8 request;

		ControlCommand command;
	};


	struct FeedbackPackage
	{
		u8 request;
		//u8 flag;
		u8 u8_flag;

		union
		{
			byte bytes[6];
			s16 s16s[3];
			u16 u16s[3];
			Vector3FP16 v3f16; //姿态角度/角速度/加速度
		};
	};



	//buffer共用体, 储存从radio读取的数据
	union
	{
		byte bytes[BUFFER_SIZE];
		PackageHead head;
		ConfigCommandPackage config_package;
		ControlCommandPackage control_package;
	}receive;



	RF24 radio;

	//无线电收发机的地址
	const u8 address[2][6] = { "S1234", "C5678" };

	//是否与发送机建立通信
	bool connected;

	//下次回传请求
	RequestFeedbackType mFeedbackType;

	//是否输出调试信息
	u8 bLog;

	u16 receive_count;


	//Model &model;

	public:
	Synchronizer() :
		radio(PIN_RADIO_CE, PIN_RADIO_CS),
		connected(false),
		mFeedbackType(etFeedback_None),
		bLog(0),
		receive_count(0)
	{

	}

	bool Init()
	{
		return InitRadio();
	}



	private:
	bool InitRadio()
	{
		LOGF("Initial RF24/Transimiter: ");

		if(!radio.begin())
		{
			LOGF("Failed\n");
			return false;
		}

		//xx 接收RF24中断 xx
		//不知道为什么一直触发
		//pinMode(PIN_IRQ_RF24, INPUT);
		//attachInterrupt(digitalPinToInterrupt(PIN_IRQ_RF24), OnRF24Interrtupt, FALLING);

		//关闭自动确认和重试
		//radio.setAutoAck(false);
		//radio.setRetries(0, 0);
		radio.setChannel(102);

		// RF24_PA_MAX is default.
		//radio.setPALevel(RF24_PA_MAX);

		radio.setPayloadSize(BUFFER_SIZE);

		// set the TX address of the RX node into the TX pipe
		radio.openWritingPipe(address[1]);

		// set the RX address of the TX node into a RX pipe
		radio.openReadingPipe(1, address[0]);

		//#=>发送模式
		radio.startListening();
		LOGF("#=> Receive Mode.\n");

		return true;
	}



	void Connect(Model &model)
	{
		connected = true;
	}

	void Disconnect(Model &model)
	{
		connected = false;
		SetState(model, etStateToken_PowerOff);
	}


	//设定无人机状态机
	void SetState(Model &model, StateTransferToken token)
	{
		if(token == etStateToken_PowerOn) model.PowerOn();
		else if(token == etStateToken_PowerOff) model.PowerOff();
		else if(token == etStateToken_Lock) model.Lock();
		else if(token == etStateToken_Unlock) model.Unlock();
		else if(token == etStateToken_Launch) model.Launch();
		else if(token == etStateToken_SafeMode) model.TurnToSafeMode();
	}



#define FB_RESULT(bSuccess) (bSuccess ? etFeedback_Ack : etFeedback_Fail)


	//读取并应用Configure数据
	RequestFeedbackType ExecuteConfigure(ConfigCommandPackage &config, Model &model)
	{
		if(bLog)
		{
			LOGF("Executing configure: ");
			LOGLN(config.command);
		}

		if((config.command & 0xF0) == etConfig_SetPID)
		{
			u8 index = config.command & 0x0F;
			model.SetPIDParameter(index, config.s16s[0], config.s16s[1], config.s16s[2]);
			return etFeedback_Ack;
		}

		switch(config.command)
		{
			//System
		case etConfig_Hello:       Connect(model);                        return etFeedback_Ack;
		case etConfig_ByeBye:      Disconnect(model);                     return etFeedback_Ack;
		case etConfig_Get:
		{
			//返回发送目标
			return (RequestFeedbackType)config.get.target;
		}

			//Config
		case etConfig_StateToken:
		{
			SetState(model, (StateTransferToken)config.value_u8);
			return etFeedback_Ack;
		}
		case etConfig_ActionAngle:
		{
			model.SetActionAngle(config.bytes[0], config.bytes[1], config.bytes[2]);
			return etFeedback_Ack;
		}
		case etConfig_Calibrate:
		{
			model.Calibrate();
			return etFeedback_Ack;
		}

			//Property
		case etConfig_Throttle:
		{
			model.SetThrottleProperty(config.throttle.start, config.throttle.range);
			return etFeedback_Ack;
		}
		case etConfig_RudderBias:
		{
			model.SetRudderCenter(config.rudder_center.index, config.rudder_center.value);
			return etFeedback_Ack;
		}
		case etConfig_RudderRatio:
		{
			model.SetRudderAngle(config.value_u8);
			return etFeedback_Ack;
		}

			//PID
		case etConfig_EnablePID:   model.config.use_PID = true;           return etFeedback_Ack;
		case etConfig_DisablePID:  model.config.use_PID = false;          return etFeedback_Ack;
		case etConfig_SetCascade:
		{
			model.config.use_cascade.value = config.value_u8;
			return etFeedback_Ack;
		}
		case etConfig_ConfigPID:
		{
			model.ConfigPID(config.pid.index, 0);
			return etFeedback_Ack;
		}
		//case etConfig_SetPID:
		{
			//model.SetPIDParameter(config.pid.index, config.pid.id, config.pid.value);
			//return etFeedback_Ack;
		}

			//Storage
		case etConfig_Store:
		{
			if(config.store.cm_flags) Storage::StoreParameters(model, config.store.cm_flags);
			if(config.store.pid_flags) Storage::StorePIDs(model.GetPIDs(), config.store.pid_flags);
			return etFeedback_Ack;
		}
		case etConfig_Load:
		{
			u8 fcm = config.store.cm_flags;
			u8 fpid = config.store.pid_flags;
			return (fcm && Storage::LoadParameters(model, fcm) != fcm)
				|| (fpid && Storage::LoadPIDs(model.GetPIDs(), fpid) != fpid)
					? etFeedback_Fail : etFeedback_Ack;
		}
		case etConfig_Reset:
		{
			if(config.store.cm_flags) Storage::ResetParameters(model, config.store.cm_flags);
			if(config.store.pid_flags) Storage::ResetPIDs(model.GetPIDs(), config.store.pid_flags);
			return etFeedback_Ack;
		}
		case etConfig_Delete:
		{
			Storage::ClearFlags(config.store.cm_flags, config.store.pid_flags);
			return etFeedback_Ack;
		}

			//Log
		case etConfig_Log:
		{
			model.SetLog(config.bytes[0], config.bytes[1], config.bytes[2]);
			bLog = config.bytes[3];
			return etFeedback_Ack;
		}
		}

		return etFeedback_UnknownConfigCommand;
	}


	//----------------------------- Radio Function -----------------------------//

	//检查接收主机发送的命令(Configure/Control)
	bool TryReceiveData()
	{
		if(radio.available())
		{
			radio.read(&receive, BUFFER_SIZE);
			if(bLog)
			{
				LOGF("\nReceive Data: 0b");
				LOGLN(*(u8 *)&receive.head, BIN);
			}
			//增加接收计数
			receive_count++;
			return true;
		}
		else return false;
	}

	//发送回传数据
	//**需要轮换发送内容
	bool SendFeedback(RequestFeedbackType request, Model *model)
	{
		if(request == etFeedback_None) return false;

		u32 t_start_send = micros();
		//#=>发送模式
		radio.stopListening();

		FeedbackPackage feedback;
		//feedback.flag = 0b10101010;
		feedback.request = (u8)request; //*Ack/Fail/Others
		feedback.u8_flag = 0;

		if(model != nullptr)
		{
			if(request == etFeedback_State)
			{
				feedback.bytes[0] = (u8)model->state;
				if(bLog)
				{
					LOGF("Send State: ");
					LOGLN(feedback.bytes[0]);
				}
			}
			else if(request == etFeedback_Angles)
			{
				Vector3FToVector3FP16(model->status.angle, feedback.v3f16);
			}
			else if(request == etFeedback_AngularVelocity)
			{
				Vector3FToVector3FP16(model->status.angular_velocity, feedback.v3f16);
			}
			else if(request == etFeedback_Acceleration)
			{
				Vector3FToVector3FP16(model->status.acceleration, feedback.v3f16);
			}
			else if(request == etFeedback_PIDParameters)
			{
				u8 index = receive.config_package.get.index;
				PIDController &pid = model->GetPID(index);
				feedback.u8_flag = index;
				feedback.s16s[0] = (s16)(pid.kp * 1000);
				feedback.s16s[1] = (s16)(pid.ki * 1000);
				feedback.s16s[2] = (s16)(pid.kd * 1000);
				if(bLog)
				{
					LOGF("Send PID[");
					LOG(index);
					LOGF("]\n");
				}
			}
			else if(request == etFeedback_Battery)
			{
				feedback.bytes[0] = model->status.battery_level;
			}
			else if(request == etFeedback_TimeCycle)
			{
				feedback.bytes[0] = model->status.delta_time_1024;
			}
			else if(request == etFeedback_Config)
			{
				feedback.bytes[0] = model->config.action_angle[0];
				feedback.bytes[1] = model->config.action_angle[1];
				feedback.bytes[2] = model->config.action_angle[2];
				feedback.bytes[3] = model->config.use_PID;
				feedback.bytes[4] = model->config.use_cascade.value;
			}
			else if(request == etFeedback_RuddersProperty)
			{
				feedback.bytes[0] = model->property.rudder.angle_range;
				feedback.bytes[1] = (model->property.rudder.middle[0] - 1000) / 4;
				feedback.bytes[2] = (model->property.rudder.middle[1] - 1000) / 4;
				feedback.bytes[3] = (model->property.rudder.middle[2] - 1000) / 4;
				feedback.bytes[4] = (model->property.rudder.middle[3] - 1000) / 4;
			}
			else if(request == etFeedback_LogSwitch)
			{
				feedback.bytes[0] = model->log.status.value;
				feedback.bytes[1] = model->log.pid.value;
				feedback.bytes[2] = model->log.rudder;
				feedback.bytes[3] = bLog;
			}
			else if(request == etFeedback_StoreInfo)
			{
				Storage::DataExists(feedback.bytes[0], feedback.bytes[1]);
			}
			else if(request == etFeedback_ReceiveCount)
			{
				feedback.u16s[0] = receive_count;
				receive_count = 0;
			}
			else
			{
				feedback.v3f16 = { 0, 0, 0 };
			}
		}


		bool result = radio.write(&feedback, BUFFER_SIZE);
		u32 t_end = micros();

		if(bLog)
		{
			LOGF("Send feedback: ");
			if(result) { LOGF("Successed  "); }
			else { LOGF("Failed  "); }

			LOG((t_end - t_start_send) / 1000.0);
			LOGF("ms");

			u8 retry_count = radio.getARC();
			LOGF("  Retry Count = ");
			LOGLN(retry_count);
		}

		//#=>接收模式
		radio.startListening();

		return result;
	}

	//------------------------------- Drive Function ------------------------------//


	public:
	/*
	void Connect()
	{
		//接收到启动令牌才能继续运行
		while(!connected)
		{
			//接收处理主机的配置命令
			//Sync();

			if(TryReceiveData())
			{
				if(receive.head.package_type == etPackage_Config)
				{
					if(receive.config_package.command == etConfig_Hello)
					{
						connected = true;
						SendFeedback(etFeedback_Ack, nullptr);
						LOGF("Connected!");
					}
					else SendFeedback(etFeedback_Fail, nullptr);
				}
			}
		}
	}
	*/

	void WaitConnect(Model &model)
	{
		//接收到启动令牌才能继续运行
		while(!connected)
		{
			//接收处理主机的配置命令
			Sync(model);
		}
	}




	//查询并处理从主机获取到的数据
	void Sync(Model &model)
	{
		if(TryReceiveData())
		{
			PackageType ptype = receive.head.package_type;
			if(ptype == etPackage_Control)
			{
				model.SetInput(receive.control_package.command);

				/*
				if(bLog)
				{
					LOGF("Set Control:");
					LOGLN(receive.control_package.command.throttle);
				}
				*/

				//直接发送
				//auto fb = (RequestFeedbackType)receive.control_package.request;
				//SendFeedback(fb);

				//延迟发送回传
				mFeedbackType = (RequestFeedbackType)receive.control_package.request;
			}
			else if(ptype == etPackage_Config)
			{
				RequestFeedbackType fb = ExecuteConfigure(receive.config_package, model);

				mFeedbackType = fb;

				//马上发送结果, 如果失败, 后续重试一次
				if(fb == etFeedback_Ack || fb == etFeedback_Fail)
				{
					if(SendFeedback(fb, &model)) mFeedbackType = etFeedback_None;
				}
			}
			else mFeedbackType = etFeedback_UnknownPackageType;
		}
		else mFeedbackType = etFeedback_None;
	}


	void LateSync(Model &model)
	{
		if(mFeedbackType == etFeedback_None) return;

		SendFeedback(mFeedbackType, &model);
	}


};

