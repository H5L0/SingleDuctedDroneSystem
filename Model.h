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
	u16 throttle;  //����T [0, 1000] **����Ӧ�ÿ�����ֱ�ٶ�
	s8 angle_x;    //X��б�Ƕ� angle = value/128 * action_angle(deg);
	s8 angle_y;    //Y��б�Ƕ�
	s16 angle_z;   //Zƫ���ٶ� [-128, 127] => velocity(float) = value;
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

	// ���˻����ò���(���Զ�̬�仯����)
	struct ConfigStruct
	{
		u8 action_angle[3];   //��̬��б�Ƿ���(deg)
		//u8 action_angle_z;    //z�ᶯ����

		bool use_PID;         //�Ƿ�Ӧ��PID����
		union
		{
			struct
			{
				bool x : 1;
				bool y : 1;
				bool z : 1;
			};
			u8 value;
		}use_cascade;         //�Ƿ�Ӧ�ô���(���⻷)
		
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
		}status;          //�Ƿ��ӡ״̬��Ϣ

		union
		{
			struct
			{
				bool x : 1;
				bool y : 1;
				bool z : 1;
			};
			u8 value;
		}pid;             //�Ƿ��ӡPID���

		bool rudder;      //�Ƿ��ӡ��λ��
		//bool log_radio;       //�Ƿ��ӡ���ߵ���Ϣ
	}log;  // 0b0000, 0b000, false, false


	//���˻���ʼ���� (������Ϊstatic����)
	struct PropertyStruct
	{
		struct ThrottleProperty
		{
			u16 start;        //0���Ŷ�Ӧ��PWM�ź�ʱ��(us)
			u16 range;        //�����źŷ�Χ(us)
		}throttle;

		struct RudderProperty
		{
			u16 middle[4];    //PWM�ź��м�ֵ(us) ��Զ���ͺźͰ�װ�������
			u8 angle_range;   //��Ƭ����ת��(deg) PWM�źű仯��Χ = Control.angle[-1,1f] * value * (1000/180) (us)
		}rudder;

	}property;


	//���˻�״̬
	struct
	{
		Vector3F acceleration;      //���ٶ�
		Vector3F angle;             //��̬�Ƕ�
		Vector3F angular_velocity;  //���ٶ�

		u8 battery_level;           //��ص�λ [0V, 5V] => [0,255] (ADC��ԭʼת�����)


		u8 delta_time_1024;         //���һ�θ��µļ��(1/1024s)

		float delta_time;           //���¼��(s)

		u32 last_update_time;       //��һ���µ�ʱ��(us)

		u16 lost_control_timer;     //ʧȥ���Ƶ�ʱ��(ms)
	}status;


	//ϵͳ���Ƶ�����
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

	//����PID�������ļ�����
	struct
	{
		PIDController VH;  //��ֱ�ٶ�
		PIDController AH;  //��ֱ���ٶ�

		PIDController RX;  //X����̬�Ƕ�
		PIDController VX;  //X����ٶ�

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

	//�趨ĳ��PID��ĳ������
	void SetPIDParameter(u8 index, u8 id, fp32 value_fp32);
	void SetPIDParameter(u8 index, s16 kp, s16 ki, s16 kd);
	void ConfigPID(u8 index, byte u8);

	void Calibrate();

	void SetServo(u8 index, float value);

	void SetMotor(u16 throttle);

	void SetControl(u16 throttle, float x, float y, float z);

	//----------------------------- Routine Function ------------------------------//

	//�Ӹ��豸�ϻ�ȡ��ǰ���˻�״̬
	void UpdateStatus();

	//Ӧ��PID
	void UpdatePID();

	//����PID�Ϳ������
	void UpdateControl();


	void Update();


};



