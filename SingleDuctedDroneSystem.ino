#include "HL.Types.h"
#include "HL.Vectors.h"
#include "HL.PID.h"
#include "HL.Log.h"

#include "Wire.h"
#include "SPI.h"

//#define USE_HL_MPU6050

#ifdef USE_HL_MPU6050
#include "HL.MPU6050.h"
#else
#include "MPU6050_light.h"
#endif

#include "RF24.h"
#include "Servo.h"
//#include "EEPROM.h"
#include "avr/eeprom.h"


#define PIN_MOTOR   9   //电机PWM信号引脚
#define PIN_SERVO_1 3   //前舵机PWM信号引脚
#define PIN_SERVO_2 10  //右
#define PIN_SERVO_3 6   //后
#define PIN_SERVO_4 5   //左

#define PIN_RADIO_CE 7  //无线电模块 使能引脚
#define PIN_RADIO_CS 8  //无线电模块 片选引脚

#define PIN_BEEP    A0
#define PIN_BATTERY A6

//-------------------------------------- EEPROM ------------------------------------//

#define EADDR_INFO     ((void *)0)  //储存情况的EEPROM储存地址
#define EADDR_CONFIG   ((void *)8)  //配置的EEPROM储存地址
#define EADDR_THROTTLE ((void *)32) //油门属性的EEPROM储存地址
#define EADDR_RUDDER   ((void *)40)
#define EADDR_PID      ((void *)64)

#define ESIZE_INFO     sizeof(StoreInfoStruct)
#define ESIZE_CONFIG   sizeof(ConfigStruct)
#define ESIZE_THROTTLE sizeof(PropertyStruct::ThrottleProperty)
#define ESIZE_RUDDER   sizeof(PropertyStruct::RudderProperty)
#define ESIZE_PID_PART (sizeof(float) * 3) //PID的关键参数大小

#define EKEY (0b10101010)

//...array


struct StoreInfoStruct
{
	u8 key;
	u8 flags;
	u8 pid_flags;
	//...
}StoreInfo;

/*
enum StoreFlag
{
	etStore_Config   = 0b0001,
	etStore_Throttle = 0b0010,
	etStore_Rudder   = 0b0100,
	etStore_PID      = 0b1000,
};
*/

//-------------------------------- Command Enum --------------------------------//

enum PackageType
{
	etPackage_None = 0b00,
	etPackage_Control = 0b01,
	etPackage_Config = 0b10,
};

enum ConfigCommandType
{
	etConfig_Hello = 0x01,
	etConfig_ByeBye = 0x02,
	etConfig_Get = 0x03,  //获取参数

	etConfig_StateToken  = 0x05,  //无人机状态转移符
	etConfig_ActionAngle = 0x08,  //控制角度幅度

	etConfig_Throttle    = 0x10,  //油门最大/最小/开始值
	etConfig_RudderBias  = 0x20,
	etConfig_RudderRatio = 0x21,

	etConfig_EnablePID      = 0x41,
	etConfig_DisablePID     = 0x42,
	etConfig_EnableCascade  = 0x45,
	etConfig_DisableCascade = 0x46,

	etConfig_SetPID    = 0x50,  //设置PID的参数
	etConfig_ZeroPID   = 0x51,  //清空PID的积分

	etConfig_Calibrate = 0x60,  //以当前姿态为基准校正MPU偏移

	etConfig_Store     = 0x80,  //储存某参数到EEPROM
	etConfig_Load      = 0x81,  //从EEPROM读取数据
	etConfig_Delete    = 0x8F,  //清除储存到EEPROM的参数

	etConfig_Log       = 0xB1,  //控制打印的信息

	//重置所有参数
	etConfig_Reset     = 0xAB,

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

	etFeedback_PIDParameters = 0x21,
	etFeedback_RudderParameters = 0x22,  //bias[0][1][2][3] [ratio] [shift]

	etFeedback_StoreInfo = 0x31,

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
	etStateToken_PowerOn  = 0x01, //? => 锁定模式
	etStateToken_PowerOff = 0x88, //锁定模式/安全模式 => 

	etStateToken_Lock   = 0xF0,   //安全模式 => 锁定模式
	etStateToken_Unlock = 0xFA,   //锁定模式 => 安全模式

	etStateToken_Launch   = 0x11, //安全模式 => 飞行模式
	etStateToken_SafeMode = 0xFE, //飞行模式 => 安全模式
};


enum SystemState
{
	eState_PowerOff = 0,
	//eState_PowerOn  = 1,
	eState_LockMode = 2,
	eState_SafeMode = 3,
	eState_FlightMode = 4,

	//eState_Unknown = 255,
} state;


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

	u16 throttle;  //油门T [0, 1000] **本来应该控制竖直速度
	s8 angle_x;    //X倾斜角度 angle = value/128 * action_angle(deg);
	s8 angle_y;    //Y倾斜角度
	s16 angle_z;   //Z偏航速度 [-128, 127] => velocity(float) = value;
};


struct FeedbackPackage
{
	u8 flag;
	u8 request;

	union
	{
		byte bytes[6];
		Vector3FP16 v3f16; //姿态角度/角速度/加速度

		struct
		{

		}rudders;
	};

	//u8 battery_level;    //电池电位 [0V, 5V] => [0,255] (ADC的原始转换结果)
};


//----------------------------- Structure Parameter ----------------------------//


//无人机配置参数 (可以动态变化的量)
struct ConfigStruct
{
	u8 action_angle;      //姿态倾斜角幅度(deg)

	bool use_PID;         //是否应用PID控制
	bool use_cascade;     //是否应用串级(内外环)

	union
	{
		struct
		{
			bool angle : 1;
			bool angular_velocity : 1;
			bool acceleration : 1;
			bool delta_time : 1;
		};
		u8 value;
	}log_status;          //是否打印状态信息

	union
	{
		struct
		{
			bool x : 1;
			bool y : 1;
			bool z : 1;
		};
		u8 value;
	}log_pid;             //是否打印PID输出

	bool log_rudder;      //是否打印舵位置
	bool log_radio;       //是否打印无线电信息

}Config = { 16, true, true, 0b0000, 0b000, false, false };


//无人机初始属性 (可以设为static的量)
struct PropertyStruct
{
	struct ThrottleProperty
	{
		u16 start;        //0油门对应的PWM信号时长(us)
		u16 range;        //油门信号范围(us)
	}throttle;

	struct RudderProperty
	{
		u16 middle[4];    //PWM信号中间值(us) 针对舵机型号和安装情况设置
		u8 angle_range;   //舵片单侧转角(deg) PWM信号变化范围 = Control.angle[-1,1f] * value * (1000/180) (us)
	}rudder;

}Property = { { 1000, 1000 }, { { 1500, 1500, 1500, 1500 }, 16 } };


//无人机状态
struct
{
	Vector3F acceleration;      //加速度
	Vector3F angle;             //姿态角度
	Vector3F angular_velocity;  //角速度

	u8 battery_level;           //电池电位 [0V, 5V] => [0,255] (ADC的原始转换结果)

	u8 delta_time_1024;         //最近一次更新的间隔(1/1024s)
	float delta_time;           //更新间隔(s)
	u32 last_update_time;       //上一更新的时间(us)

	u16 lost_control_timer;     //失去控制的时间(ms)
}Status;


//系统控制的内容
struct
{
	//[0, 1000] => 1000 + value =>>= [0%, 100%]
	u16 throttle;
	float rudders[4];

	Servo motor;
	Servo servo[4];
}Control = { 0, { 0, 0, 0, 0 } };



//控制命令
ControlCommandPackage Command = { { PackageType::etPackage_Control, 0, 0 }, 0, 0, 0, 0 };


#define BUFFER_SIZE 8

//buffer共用体, 用于储存从radio读取的数据
union
{
	byte bytes[BUFFER_SIZE];
	PackageHead head;
	ConfigCommandPackage config_package;
	ControlCommandPackage control_package;
}receive;


//------------------------------- Control: PID --------------------------------//

//所有PID控制器的集合体
struct
{
	PIDController VH;  //竖直速度
	PIDController AH;  //竖直加速度

	PIDController RX;  //X轴姿态角度
	PIDController VX;  //X轴角速度

	PIDController RY;
	PIDController VY;

	PIDController RZ;
	PIDController VZ;
}PID;


//--------------------------------- Switch -----------------------------------//

//是否已启动，指示能否运行系统
//false: 不能使用能源，所有程序都不应该运行，尽量不要输出信号
//bool power_on = false;

//是否被操纵
//false: 飞行器由系统暂时掌控
//bool is_driving = false;

//是否处在锁定模式
//true: 不能控制, 动力逐渐减小
//bool locked = true;

//指示系统是否处于安全模式
//true: 系统应该马上关闭电机、收起可能损坏的设备, 但仍可以控制伺服器
//bool safe_mode = true;

//--------------------------------- Device -----------------------------------//

MPU6050 mpu(Wire);

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CS);

const u8 address[][6] = { "S1234", "C5678" };

bool connected = false;


// *Beep*
//由于3个定时器可能都被6路PWM使用了, 所以使用轮巡实现有源蜂鸣器的节律
struct
{
	u8 length;      //长度(>8则重复) (255: 无限循环)
	u8 wait_timer;  //蜂鸣每节时长
	u8 pattern;     //Bit[x]=1 代表蜂鸣器响1/4s (低位开始)
	u8 position;    //当前pattern的bit位置 [0, 8]
}Beep = { 0, 0, 0, 0 };



/*
u16 throttle_max = 2000;
u16 throttle_min = 1000;
u16 throttle_start = 1050;

s8 rudder_angle_bias[4];
u8 rudder_angle_ratio = 16;  //[1, 32] 默认16 =>单侧转角25.6°    32 => 51.2°
*/


//------------------------------- Init Function -------------------------------//

void InitProperty()
{
	Control.motor.attach(PIN_MOTOR, 1000, 2000);
	Control.servo[0].attach(PIN_SERVO_1, 1000, 2000);  //**最大最小值待测试
	Control.servo[1].attach(PIN_SERVO_2, 1000, 2000);  //**待测试
	Control.servo[2].attach(PIN_SERVO_3, 1000, 2000);  //**待测试
	Control.servo[3].attach(PIN_SERVO_4, 1000, 2000);  //**待测试

	//重置控制
	SetControl(0, 0, 0, 0);

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
	/*
	PID.RZ.kp = FP16_1 * 2;
	PID.RZ.ki = FP16_H5;
	PID.RZ.kd = FP16_H5;

	PID.VZ.kp = FP16_1 * 2;
	PID.VZ.ki = FP16_1;
	PID.VZ.kd = 0;
	*/
	PID.RZ.kp = 0.2f;
	PID.RZ.ki = 0;
	PID.RZ.kd = 0;

	PID.VZ.kp = 2.0f;
	PID.VZ.ki = 1.0f;
	PID.VZ.kd = 0;

	//***DEBUG***//
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


bool InitRadio()
{
	LOGF("Initial RF24/Transimiter: ");

	// initialize the transceiver on the SPI bus
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

	// Set the PA Level low to try preventing power supply related problems
	// because these examples are likely run with nodes in close proximity to
	// each other.
	//radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

	// save on transmission time by setting the radio to only transmit the
	// number of bytes we need to transmit a float
	radio.setPayloadSize(BUFFER_SIZE);

	// set the TX address of the RX node into the TX pipe
	radio.openWritingPipe(address[1]); // always uses pipe 0

	// set the RX address of the TX node into a RX pipe
	radio.openReadingPipe(1, address[0]); // using pipe 1

	//#=>发送模式
	radio.startListening();
	LOGF("#=> Receive Mode.\n");

	return true;
}


bool InitMPU()
{
	LOGF("Initial MPU6050... ");

#ifdef USE_HL_MPU6050
	byte status = mpu.Begin();
#else
	byte status = mpu.begin();
#endif
	LOGF("Status: ");
	LOGLN(status);
	if(status != 0) return false;

	LOGF("Calculating offsets, do not move MPU6050. ");
	delay(255);

#ifdef USE_HL_MPU6050
	mpu.Calibrate();
#else
	mpu.calcOffsets(true, true);
#endif
	LOGF("Done!\n");
	return true;
}



void SetMotor(u16 throttle)
{
	Control.throttle = throttle > 1000 ? 1000 : throttle;
	Control.motor.writeMicroseconds(Control.throttle + Property.throttle.start);
}

void SetServo(u8 index, float value)
{
	if(value > 1) value = 1;
	else if(value < -1) value = -1;

	//滤波平滑 20%
	const float cof = 0.8f;
	float sValue = value * (1 - cof) + Control.rudders[index] * cof;
	Control.rudders[index] = sValue;

	// = middle + bias + value * ratio / 16
	s16 center = (s16)Property.rudder.middle[index];
	s16 offset = (s16)(sValue * Property.rudder.angle_range * (1000.0f / 180.0f));
	Control.servo[index].writeMicroseconds(center + offset);
}



void SetControl(u16 throttle, float x, float y, float z)
{
	SetMotor(throttle);
	z /= 2;
	SetServo(0, -y + z);
	SetServo(1, -x + z);
	SetServo(2,  y + z);
	SetServo(3,  x + z);
}



void SetBeep(u8 pattern, u8 length = 8)
{
	//还未开始loop循环, 堵塞鸣叫
	if(Status.last_update_time == 0 && false)
	{
		//不提供无限鸣叫
		for(u8 i = 0; i != length; i++)
		{
			u8 pos = i & 0b111;
			u8 beep = (pattern >> pos) & 1;
			digitalWrite(PIN_BEEP, beep);
			delay(250);
		}
		digitalWrite(PIN_BEEP, LOW);
	}
	else
	{
		Beep.length = length;
		Beep.wait_timer = 0;
		Beep.pattern = pattern;
		Beep.position = 0;
		digitalWrite(PIN_BEEP, LOW);
	}
}

void UpdateBeep(u8 delta_time_1024)
{
	//无蜂鸣器任务
	if(Beep.pattern == 0) return;
	//上一次任务已完成
	if(Beep.length == 0)
	{
		digitalWrite(PIN_BEEP, LOW);
		Beep.pattern = 0;
		return;
	}

	u16 t = Beep.wait_timer + delta_time_1024;
	if(t > 128)
	{
		Beep.wait_timer = 0;

		u8 pos = Beep.position & 0b111;
		u8 beep = (Beep.pattern >> pos) & 1;
		digitalWrite(PIN_BEEP, beep);

		++Beep.position;

		//结束
		if(Beep.length != 255 && Beep.position == Beep.length)
		{
			Beep.length = 0;
		}
	}
	else Beep.wait_timer = (u8)t;
}


//-------------------------------- Configure --------------------------------//


//设定无人机状态机
void SetState(StateTransferToken token)
{
	if(token == etStateToken_PowerOn)
	{
		if(state == eState_PowerOff)
		{
			SetBeep(0b11, 2);
			state = eState_LockMode;
			SetControl(0, 0, 0, 0);
		}
	}
	else if(token == etStateToken_PowerOff)
	{
		if(state != eState_FlightMode)
		{
			SetBeep(0b1111, 4);
			state = eState_PowerOff;
		}
	}
	else if(token == etStateToken_Launch)
	{
		//只有处于安全模式下才能起飞
		if(state == eState_SafeMode)
		{
			SetBeep(0b11010101);
			Status.lost_control_timer = 0;
			state = eState_FlightMode;
			return;
		}
	}
	else if(token == etStateToken_SafeMode)
	{
		//**锁定模式不能直接转入安全模式(只能使用Unlock)
		//只在飞行模式时才能直接进入安全模式
		if(state == eState_FlightMode)
		{
			SetBeep(0b11001001);
			state = eState_SafeMode;
			return;
		}
	}
	else if(token == etStateToken_Lock)
	{
		//只在安全模式时才进入锁定模式
		if(state == eState_SafeMode)
		{
			SetBeep(0b1001);
			state = eState_LockMode;
			SetControl(0, 0, 0, 0);
		}
	}
	else if(token == etStateToken_Unlock)
	{
		if(state == eState_LockMode)
		{
			//校准
			//Calibrate();
			SetBeep(0b0101);
			state = eState_SafeMode;
		}
	}
}


void SetThrottleProperty(ConfigCommandPackage::ThrottlePropertyStruct &throttle)
{
	Property.throttle.start = throttle.start;
	Property.throttle.range = throttle.range;
}


void SetRudderBias(ConfigCommandPackage::RudderCenterStruct center)
{
	Property.rudder.middle[center.index] = center.value;

	SetBeep(0b1, 2);
}


//设定某个PID的某个参数
void SetPIDParameter(ConfigCommandPackage::PIDConfigureStruct &config)
{
	PIDController &tpid = ((PIDController *)&PID)[config.index];
	float value = fp32_to_float(config.value);
	if(config.id == 1)      tpid.kp = value;
	else if(config.id == 2) tpid.ki = value;
	else if(config.id == 3) tpid.kd = value;
	//else if(config.pid.id == 4) tpid.integration_clamp = config.value;

	LOGF("Setting PID[");
	LOG(config.index);
	LOGF("][");
	LOG(config.id);
	LOGF("] => ");
	LOGLN(value);

	SetBeep(0b010101, 6);
}

void ClearPIDIntegration(u8 index)
{
	//清除全部PID积分
	if(index == 255)
	{
		for(int i = 0; i < 8; i++)
		{
			PIDController &tpid = ((PIDController *)&PID)[i];
			tpid.error_integration = 0;
		}
	}
	else
	{
		PIDController &tpid = ((PIDController *)&PID)[index];
		tpid.error_integration = 0;
	}

	SetBeep(0b1, 2);
}



bool StoreBytes(const void *data, void *pos, u8 size)
{
	eeprom_busy_wait();
	eeprom_update_block(data, pos, size);
	return true;
}

bool LoadBytes(void *buffer, const void *pos, u8 size)
{
	eeprom_busy_wait();
	eeprom_read_block(buffer, pos, size);
	return true;
}


bool PersistParameters(ConfigCommandPackage::StoreStruct store, bool is_store)
{
	u8 cm_flags = store.cm_flags;
	u8 pid_flags = store.pid_flags;

	//u8 * const addr_info_key = &((StoreInfoStruct *)EADDR_INFO)->key;
	//u8 * const addr_info_flags = &((StoreInfoStruct *)EADDR_INFO)->flags;
	if(cm_flags)
	{
		void *const address[] = { EADDR_CONFIG, EADDR_THROTTLE, EADDR_RUDDER };
		void *const target[] = { &Config, &Property.throttle, &Property.rudder };
		const u8 size[] = { ESIZE_CONFIG, ESIZE_THROTTLE, ESIZE_RUDDER };

		for(u8 i = 0; i < 3; i++)
		{
			u8 fg = (1 << i);
			if(cm_flags & fg)
			{
				if(is_store)
				{
					StoreInfo.flags |= fg;
					StoreBytes(&StoreInfo, EADDR_INFO, ESIZE_INFO);
					StoreBytes(target[i], address[i], size[i]);
					LOGF("\nStore parameters[");
					LOG(i);
					LOGF("]\n");
				}
				else
				{
					LoadBytes(target[i], address[i], size[i]);
					LOGF("\nLoad parameters[");
					LOG(i);
					LOGF("]\n");
				}
			}
		}
	}

	if(pid_flags)
	{
		for(u8 i = 0; i < 8; i++)
		{
			u8 fg = (1 << i);
			if(pid_flags & fg)
			{
				void *parameters = (void *)&((PIDController *)&PID)[i].kp;
				void *address = (void *)((u16)EADDR_PID + i * ESIZE_PID_PART);
				if(is_store)
				{
					StoreInfo.pid_flags |= fg;
					StoreBytes(&StoreInfo, EADDR_INFO, ESIZE_INFO);
					StoreBytes(parameters, address, ESIZE_PID_PART);
					LOGF("\nStore PID[");
					LOG(i);
					LOGF("]\n");
				}
				else
				{
					LoadBytes(parameters, address, ESIZE_PID_PART);
					LOGF("\nLoad PID[");
					LOG(i);
					LOGF("]\n");
				}
			}
		}
	}

	SetBeep(0b11000101);
	return true;
}

bool DeleteStored(ConfigCommandPackage::StoreStruct store)
{
	StoreInfo.flags &= ~store.cm_flags;
	StoreInfo.pid_flags &= ~store.pid_flags;
	StoreBytes(&StoreInfo, EADDR_INFO, ESIZE_INFO);

	LOGF("\nDelete Stored: ");
	LOG(store.cm_flags, BIN);
	LOGF("  PID: ");
	LOGLN(store.pid_flags, BIN);

	SetBeep(0b11110101);
	return true;
}


void Calibrate()
{

#ifdef USE_HL_MPU6050
	mpu.Calibrate();
	mpu.Reset();
#else
	mpu.calcOffsets(true, true);
	mpu._Reset();
#endif

	ClearPIDIntegration(255);

	Status.last_update_time = 0;

	LOGF("Calibarate Finish...\n");
	SetBeep(0b10101101);
}


void Reset()
{
	//.....**.....
}

void SetLog(byte bytes[4])
{
	Config.log_status.value = bytes[0];
	Config.log_pid.value = bytes[1];
	Config.log_rudder = bytes[2];
	Config.log_radio = bytes[3];
}


//读取并应用Configure数据
RequestFeedbackType ExecuteConfigure(ConfigCommandPackage &config)
{
	if(Config.log_radio)
	{
		LOGF("Executing configure: ");
		LOGLN(config.command);
	}
	switch(config.command)
	{
	case etConfig_Hello:       connected = true;                      return etFeedback_Ack;
	case etConfig_ByeBye:      connected = false;                     return etFeedback_Ack;
	case etConfig_Get:         return (RequestFeedbackType)config.get.target; //返回发送目标

	case etConfig_StateToken:  SetState((StateTransferToken)config.value_u8); return etFeedback_Ack;
		//Config
	case etConfig_ActionAngle:  Config.action_angle = config.value_u8; return etFeedback_Ack;  //未检查值
		//Property
	case etConfig_Throttle:    SetThrottleProperty(config.throttle);  return etFeedback_Ack;
	case etConfig_RudderBias:  SetRudderBias(config.rudder_center);   return etFeedback_Ack;
	case etConfig_RudderRatio: Property.rudder.angle_range = config.value_u8; return etFeedback_Ack;
		//PID
	case etConfig_EnablePID:   Config.use_PID = true;                 return etFeedback_Ack;
	case etConfig_DisablePID:  Config.use_PID = false;                return etFeedback_Ack;
	case etConfig_EnableCascade:  Config.use_cascade = true;          return etFeedback_Ack;
	case etConfig_DisableCascade: Config.use_cascade = false;         return etFeedback_Ack;
	case etConfig_SetPID:      SetPIDParameter(config.pid);           return etFeedback_Ack;
	case etConfig_ZeroPID:     ClearPIDIntegration(config.value_u8);  return etFeedback_Ack;
		//Calibrate
	case etConfig_Calibrate:   Calibrate();                           return etFeedback_Ack;
		//Store
	case etConfig_Store:       return PersistParameters(config.store, true)  ? etFeedback_Ack : etFeedback_Fail;
	case etConfig_Load:        return PersistParameters(config.store, false) ? etFeedback_Ack : etFeedback_Fail;
	case etConfig_Delete:      return DeleteStored(config.store) ? etFeedback_Ack : etFeedback_Fail;
		//Log
	case etConfig_Log:         SetLog(config.bytes);                  return etFeedback_Ack;
	case etConfig_Reset:       Reset();                               return etFeedback_Ack;
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
		if(Config.log_radio)
		{
			LOGF("Receive Data: 0b");
			LOGLN(*(u8 *)&receive.head, BIN);
		}
		return true;
	}
	else return false;
}

//发送回传数据
//**需要轮换发送内容
bool SendFeedback(RequestFeedbackType request)
{
	if(request == etFeedback_None) return false;

	u32 t_start_send = micros();
	//#=>发送模式
	radio.stopListening();

	FeedbackPackage feedback;
	feedback.flag = 0b10101010;
	feedback.request = (u8)request; //*Ack/Fail/Others

	if(request == etFeedback_State)
	{
		feedback.bytes[0] = (u8)state;
	}
	if(request == etFeedback_Angles)
	{
		Vector3FToVector3FP16(Status.angle, feedback.v3f16);
	}
	else if(request == etFeedback_AngularVelocity)
	{
		Vector3FToVector3FP16(Status.angular_velocity, feedback.v3f16);
	}
	else if(request == etFeedback_Acceleration)
	{
		Vector3FToVector3FP16(Status.acceleration, feedback.v3f16);
	}
	else if(request == etFeedback_PIDParameters)
	{
		u8 index = receive.config_package.get.index;
		PIDController &pid = ((PIDController *)&PID)[index];
		Vector3FToVector3FP16(*(Vector3F *)&pid.kp, feedback.v3f16);
	}
	else
	{
		feedback.v3f16 = { 0, 0, 0 };
	}

	//feedback.battery_level = Status.battery_level;

	bool result = radio.write(&feedback, BUFFER_SIZE);
	u32 t_end = micros();

	if(Config.log_radio)
	{
		LOGF("\nSend feedback: ");
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

//查询并处理从主机获取到的数据
RequestFeedbackType HandleReceiveData()
{
	if(!TryReceiveData()) return etFeedback_None;

	PackageType ptype = receive.head.package_type;
	if(ptype == etPackage_Control)
	{
		//if(!locked) 锁定时也可以接受控制命令(但不会执行)
		Command = receive.control_package;

		//直接发送
		//auto fb = (RequestFeedbackType)receive.control_package.request;
		//SendFeedback(fb);

		//重置失去控制计时
		Status.lost_control_timer = 0;

		//延迟发送回传
		return (RequestFeedbackType)receive.control_package.request;
	}
	else if(ptype == etPackage_Config)
	{
		RequestFeedbackType fb = ExecuteConfigure(receive.config_package);

		//马上发送结果
		if(fb == etFeedback_Ack || fb == etFeedback_Fail)
		{
			//delay(2);  //***DEBUG
			return SendFeedback(fb) ? etFeedback_None : fb; //如果失败，后续重试一次
		}
		else return fb;
	}
	else return etFeedback_UnknownPackageType;
}


//----------------------------- Routine Function ------------------------------//


//从各设备上获取当前无人机状态
void UpdateStatus()
{
	//更新时间
	u32 now_time = micros();
	u16 dtime_us = (u16)(now_time - Status.last_update_time);
	//初始化时间
	if(Status.last_update_time == 0)
	{
		Status.delta_time_1024 = 8;
		Status.last_update_time = now_time;
	}
	else
	{
		// dtime.1 = 1/1024s = 1000000us/1024 = 977us  **u16: 64s溢出**
		u16 dtime = dtime_us / 977;
		// dTime钳制到[1, 255]
		Status.delta_time_1024 = dtime > 255 ? 255 : dtime == 0 ? 1 : dtime;
		Status.delta_time = dtime_us / 1000000.0f;
		Status.last_update_time = now_time;
		Status.lost_control_timer += dtime;
	}

#ifdef USE_HL_MPU6050
	u32 before_update_mpu_time = micros();
	mpu.Update(dtime_us);
	u32 after_update_mpu_time = micros();

	Status.acceleration.x = -mpu.GetAccX();
	Status.acceleration.y = -mpu.GetAccY();
	Status.acceleration.z = -mpu.GetAccZ();
	Status.angle.x = -mpu.GetAngleX();
	Status.angle.y = -mpu.GetAngleY();
	Status.angle.z = -mpu.GetAngleZ();
	Status.angular_velocity.x = -mpu.GetGyroX();
	Status.angular_velocity.y = -mpu.GetGyroY();
	Status.angular_velocity.z = -mpu.GetGyroZ();
#else
	u32 before_update_mpu_time = micros();
	mpu.update(Status.delta_time);
	u32 after_update_mpu_time = micros();

	Status.acceleration.x = -mpu.getAccX();
	Status.acceleration.y = -mpu.getAccY();
	Status.acceleration.z = -mpu.getAccZ();
	Status.angle.x = -mpu.getAngleX();
	Status.angle.y = -mpu.getAngleY();
	Status.angle.z = -mpu.getAngleZ();
	Status.angular_velocity.x = -mpu.getGyroX();
	Status.angular_velocity.y = -mpu.getGyroY();
	Status.angular_velocity.z = -mpu.getGyroZ();
#endif

	//**Battery**//
	Status.battery_level = analogRead(PIN_BATTERY);

	//DEBUG//
	if(Config.log_status.delta_time)
	{
		LOGF(" dT= ");
		LOG(Status.delta_time_1024);

		LOGF(" / ");
		LOG(Status.delta_time * 1000.0f);
		LOG("ms");

		LOGF(" MPU.dT= ");
		LOG(after_update_mpu_time - before_update_mpu_time);
		LOGF("us ");
	}
	if(Config.log_status.angle)
	{
		LOGF(" |r.x= ");
		LOG(Status.angle.x);
		LOGF(" r.y= ");
		LOG(Status.angle.y);
		LOGF(" r.z= ");
		LOG(Status.angle.z);
	}
	if(Config.log_status.angular_velocity)
	{
		LOGF(" |rv.x= ");
		LOG(Status.angular_velocity.x);
		LOGF(" rv.y= ");
		LOG(Status.angular_velocity.y);
		LOGF(" rv.z= ");
		LOG(Status.angular_velocity.z);
	}
	if(Config.log_status.acceleration)
	{
		LOGF(" |a.x= ");
		LOG(Status.acceleration.x);
		LOGF(" a.y= ");
		LOG(Status.acceleration.y);
		LOGF(" a.z= ");
		LOG(Status.acceleration.z);
	}
}

//应用PID
void UpdatePID()
{
	const float dtime = Status.delta_time;
	const u8 avFactor = 32;  //32/64/100 影响角速度PID的量级

	if(Config.use_cascade)
	{
		//RX -> VX
		float target_rx = ((s16)(Command.angle_x) * Config.action_angle) / 128.0f;
		float rx = Status.angle.x;
		float vx = Status.angular_velocity.x / avFactor;

		float o_rx = PID.RX.Step(target_rx, rx, dtime);
		float o_vx = PID.VX.Step(o_rx, vx, dtime);


		//RY -> VY
		float target_ry = ((s16)(Command.angle_y) * Config.action_angle) / 128.0f;
		float ry = Status.angle.y;
		float vy = Status.angular_velocity.y / avFactor;

		float o_ry = PID.RY.Step(target_ry, ry, dtime);
		float o_vy = PID.VY.Step(o_ry, vy, dtime);


		//RZ -> VZ
		float target_rz = ((s16)(Command.angle_z) * Config.action_angle) / 128.0f;
		float rz = Status.angle.z;
		float vz = Status.angular_velocity.z / avFactor;

		float o_rz = PID.RZ.Step(target_rz, rz, dtime);
		float o_vz = PID.VZ.Step(o_rz, vz, dtime);

		//LOG
		if(Config.log_pid.x)
		{
			LOGF(" |PID.RX= ");
			LOG(o_rx);
			LOGF(" PID.VX= ");
			LOG(o_vx);

			LOGF(" RX.I=");
			LOG(PID.RX.error_integration);
			LOGF(" VX.I=");
			LOG(PID.VX.error_integration);
		}
		if(Config.log_pid.y)
		{
			LOGF(" |PID.RY= ");
			LOG(o_ry);
			LOGF(" PID.VY= ");
			LOG(o_vy);

			LOGF(" RY.I=");
			LOG(PID.RY.error_integration);
			LOGF(" VY.I=");
			LOG(PID.VY.error_integration);
		}
		if(Config.log_pid.z)
		{
			LOGF(" |PID.RZ= ");
			LOG(o_rz);
			LOGF(" PID.VZ= ");
			LOG(o_vz);

			LOGF(" RZ.I=");
			LOG(PID.RZ.error_integration);
			LOGF(" VZ.I=");
			LOG(PID.VZ.error_integration);
		}
	}
	else
	{
		//RX -> VX
		float target_vx = ((s16)(Command.angle_x) * Config.action_angle) / (128.0f * avFactor);
		float vx = Status.angular_velocity.x / avFactor;
		float o_vx = PID.VX.Step(target_vx, vx, dtime);

		//RY -> VY
		float target_vy = ((s16)(Command.angle_y) * Config.action_angle) / (128.0f * avFactor);
		float vy = Status.angular_velocity.y / avFactor;
		float o_vy = PID.VY.Step(target_vy, vy, dtime);

		//RZ -> VZ
		float target_vz = ((s16)(Command.angle_z) * Config.action_angle) / (128.0f * avFactor);
		float vz = Status.angular_velocity.z / avFactor;
		float o_vz = PID.VZ.Step(target_vz, vz, dtime);

		//LOG
		if(Config.log_pid.x)
		{
			LOGF(" |PID.VX= ");
			LOG(o_vx);
			LOGF(" I=");
			LOG(PID.VX.error_integration);
		}
		if(Config.log_pid.y)
		{
			LOGF(" |PID.VY= ");
			LOG(o_vy);
			LOGF(" I=");
			LOG(PID.VY.error_integration);
		}
		if(Config.log_pid.z)
		{
			LOGF(" |PID.VZ= ");
			LOG(o_vz);
			LOGF(" I=");
			LOG(PID.VZ.error_integration);
		}
	}


}


void UpdateControl()
{
	if(state == eState_LockMode)
	{
		//锁定模式不启动电机
		SetMotor(0);
		//退出锁定模式时PID会重置, 所以锁定时不更新PID
		return;
	}

	u16 th;
	if(state == eState_FlightMode)
	{
		//失去控制时间超出
		if(Status.lost_control_timer > 1024)
		{
			Status.lost_control_timer = 0;
			SetState(etStateToken_SafeMode);
		}
		else
		{
			th = Command.throttle;  //**用户直接控制电机油门
		}
	}

	if(state == eState_SafeMode)
	{
		//安全模式逐渐关闭电机
		if(Control.throttle > 500) th = 500;
		else
		{
			s16 sth = Control.throttle - Status.delta_time_1024;
			th = sth < 50 ? 0 : (u16)sth;
		}
	}

	if(Config.use_PID)
	{
		UpdatePID();
		float o_x = -PID.VX.output;
		float o_y = -PID.VY.output;
		float o_z = -PID.VZ.output;   //**----
		SetControl(th, o_x, o_y, o_z);
	}
	else
	{
		SetControl(th, 0, 0, 0);
	}

	//LOG
	if(Config.log_rudder)
	{
		LOGF(" |R.1= ");
		LOG(Control.rudders[0]);
		LOGF(" R.2= ");
		LOG(Control.rudders[1]);
		LOGF(" R.3= ");
		LOG(Control.rudders[2]);
		LOGF(" R.4= ");
		LOG(Control.rudders[3]);
	}
}


//------------------------ Arduino Schedule Function -----------------------//

void setup()
{
	pinMode(PIN_BEEP, OUTPUT);
	SetBeep(0b01010101);

	//设置必要的属性初始值
	InitProperty();

	Serial.begin(115200);
	Wire.begin();

	bool bRadio = InitRadio();

	bool bMPU = InitMPU();

	if(!bRadio || !bMPU) while(true);

	LOGF("Waiting for command...");

	//接收到启动令牌才能继续运行
	while(!connected)
	{
		//接收处理主机的配置命令
		HandleReceiveData();
	}

	//更新时间, 避免PID出错
	Status.last_update_time = 0;
}



void loop()
{
	RequestFeedbackType fb = HandleReceiveData();

	if(state == eState_PowerOff || !connected) return;

	UpdateStatus();

	UpdateControl();

	//延迟发送的回传
	if(fb != etFeedback_None) SendFeedback(fb);

	UpdateBeep(Status.delta_time_1024);

	if(Config.log_status.value || Config.log_pid.value || Config.log_radio || Config.log_rudder ) LOG('\n');
}
