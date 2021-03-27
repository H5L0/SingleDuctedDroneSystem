#pragma once
#include "HL.Types.h"
#include "HL.Vectors.h"
#include "HL.PID.h"
#include "HL.Log.h"

#include "Pins.h"
#include "Beeper.h"

#include "Wire.h"
#include "Servo.h"

//#define USE_HL_MPU6050
#ifdef USE_HL_MPU6050
#include "HL.MPU6050.h"
#else
#include "MPU6050_light.h"
#endif



struct ControlCommand
{
	u16 throttle;  //油门T [0, 1000] **本来应该控制竖直速度
	s8 angle_x;    //X倾斜角度 angle = value/128 * action_angle(deg);
	s8 angle_y;    //Y倾斜角度
	s16 angle_z;   //Z偏航速度 [-128, 127] => velocity(float) = value;
};



class Model
{
	public:
	enum SystemState
	{
		eState_PowerOff = 0,
		//eState_PowerOn  = 1,
		eState_LockMode = 2,
		eState_SafeMode = 3,
		eState_FlightMode = 4,

		//eState_Unknown = 255,
	}state;

	// 无人机配置参数(可以动态变化的量)
	struct ConfigStruct
	{
		u8 action_angle[3];   //姿态倾斜角幅度(deg)
		//u8 action_angle_z;    //z轴动作角

		bool use_PID;         //是否应用PID控制
		union
		{
			struct
			{
				bool x : 1;
				bool y : 1;
				bool z : 1;
			};
			u8 value;
		}use_cascade;         //是否应用串级(内外环)
		
	}config;


	struct LogStruct
	{
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
		}status;          //是否打印状态信息

		union
		{
			struct
			{
				bool x : 1;
				bool y : 1;
				bool z : 1;
			};
			u8 value;
		}pid;             //是否打印PID输出

		bool rudder;      //是否打印舵位置
		//bool log_radio;       //是否打印无线电信息
	}log;  // 0b0000, 0b000, false, false


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

	}property;


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
	}status;


	//系统控制的内容
	struct
	{
		//[0, 1000] => 1000 + value =>>= [0%, 100%]
		u16 throttle;
		float rudders[4];

		Servo motor;
		Servo servo[4];
	}control = { 0, { 0, 0, 0, 0 } };


	ControlCommand command;


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

	}pids;


	MPU6050 mpu;

	bool InitMPU();

	public:
	Model() :
		mpu(Wire)
	{

	}

	bool Init();
	bool Reset();

	bool PowerOn();
	bool PowerOff();
	bool Unlock();
	bool Lock();
	bool Launch();
	bool TurnToSafeMode();

	//-------------------------------- Configure --------------------------------//

	void SetInput(ControlCommand &command);
	void SetLog(u8 logStatus, u8 logPids, u8 logRudders);

	void SetActionAngle(u8 angle_x, u8 angle_y, u8 angle_z);
	void SetThrottleProperty(u16 start, u16 range);
	void SetRudderCenter(u8 index, u16 value);
	void SetRudderAngle(u8 value);


	inline PIDController &GetPID(u8 index) { return ((PIDController *)&pids)[index]; }
	inline PIDController *GetPIDs() { return (PIDController *)&pids; }

	//设定某个PID的某个参数
	void SetPIDParameter(u8 index, u8 id, fp32 value_fp32);
	void SetPIDParameter(u8 index, s16 kp, s16 ki, s16 kd);
	void ConfigPID(u8 index, byte u8);

	void Calibrate();

	void SetServo(u8 index, float value);

	void SetMotor(u16 throttle);

	void SetControl(u16 throttle, float x, float y, float z);

	//----------------------------- Routine Function ------------------------------//

	//从各设备上获取当前无人机状态
	void UpdateStatus();

	//应用PID
	void UpdatePID();

	//更新PID和控制输出
	void UpdateControl();


	void Update();


};



