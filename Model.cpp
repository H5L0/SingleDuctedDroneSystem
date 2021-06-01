#include "Model.h"
#include "Storage.h"


bool Model::InitMPU()
{
	LOGF("Initial MPU6050... ");

#ifdef USE_HL_MPU6050
	byte status = mpu.Begin();
	mpu.filter_gyro_coefficient = 0.99f;
#else
	byte status = mpu.begin();
	//(*)将陀螺仪系数设定为0.99
	//mpu.setFilterGyroCoef(0.99f);
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


bool Model::Init()
{
	if(!InitMPU()) return false;

	Reset();

	//绑定控制设备
	control.motor.attach(PIN_MOTOR, 1000, 2000);
	control.servo[0].attach(PIN_SERVO_1, 1000, 2000);  //**最大最小值待测试
	control.servo[1].attach(PIN_SERVO_2, 1000, 2000);  //**待测试
	control.servo[2].attach(PIN_SERVO_3, 1000, 2000);  //**待测试
	control.servo[3].attach(PIN_SERVO_4, 1000, 2000);  //**待测试

	//重置控制
	SetControl(0, 0, 0, 0);

	//启动时电源默认关闭
	state = eState_PowerOff;

	//更新时间, 避免PID出错
	status.last_update_time = 0;

	return true;
}


bool Model::Reset()
{
	//加载默认值
	Storage::ResetParameters(*this, 0b111);
	Storage::ResetPIDs(GetPIDs(), 0xFF);
	//加载储存值
	Storage::LoadParameters(*this, 0b111);
	Storage::LoadPIDs(GetPIDs(), 0xFF);

	/*
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
	*/
	return true;
}


bool Model::PowerOn()
{
	if(state != eState_PowerOff) return false;
	state = eState_LockMode;
	SetControl(0, 0, 0, 0);
	status.last_update_time = 0;
	Beeper::Beep(0b11, 2);
	return true;
}

bool Model::PowerOff()
{
	if(state == eState_FlightMode) return false;
	state = eState_PowerOff;
	Beeper::Beep(0b1111, 4);
	return true;
}

bool Model::Unlock()
{
	if(state != eState_LockMode) return false;
	state = eState_SafeMode;
	//校准
	//Calibrate();
	Beeper::Beep(0b1001, 4);
	return true;
}

bool Model::Lock()
{
	//只在安全模式时才进入锁定模式
	if(state != eState_SafeMode) return false;
	state = eState_LockMode;
	SetControl(0, 0, 0, 0);
	Beeper::Beep(0b1101);
	return true;
}

bool Model::Launch()
{
	//只有处于安全模式下才能起飞
	if(state != eState_SafeMode) return false;
	state = eState_FlightMode;
	//重置失去控制计时
	status.lost_control_timer = 0;
	Beeper::Beep(0b11010101);
	return true;
}

bool Model::TurnToSafeMode()
{
	//**锁定模式不能直接转入安全模式(只能使用Unlock)
	//只在飞行模式时才能直接进入安全模式
	if(state != eState_FlightMode) return false;
	state = eState_SafeMode;
	Beeper::Beep(0b11001001);
	return true;
}

//-------------------------------- Configure --------------------------------//

void Model::SetInput(ControlCommand &command)
{
	this->command = command;

	//重置失去控制计时
	status.lost_control_timer = 0;
}


void Model::SetLog(u8 logStatus, u8 logPids, u8 logRudders)
{
	log.status.value = logStatus;
	log.pid.value = logPids;
	log.rudder = logRudders;
	//log.log_radio = bytes[3];
}


void Model::SetActionAngle(u8 angle_x, u8 angle_y, u8 angle_z)
{
	config.action_angle[0] = angle_x;
	config.action_angle[1] = angle_y;
	config.action_angle[2] = angle_z;

	Beeper::Beep(0b101, 3);
}

void Model::SetThrottleProperty(u16 start, u16 range)
{
	//property.throttle.start = start;
	//property.throttle.range = range;
	//安全起见, 不使用传入值, 避免使用从EEPROM读取的错误数据
	property.throttle.start = 1000;
	property.throttle.range = 1000;

	Beeper::Beep(0b10101001);
}

void Model::SetRudderCenter(u8 index, u16 value)
{
	property.rudder.middle[index] = value;

	Beeper::Beep(0b1, 2);
}

void Model::SetRudderAngle(u8 value)
{
	property.rudder.angle_range = value;

	Beeper::Beep(0b101, 3);
}


//设定某个PID的某个参数
void Model::SetPIDParameter(u8 index, u8 id, fp32 value_fp32)
{
	// 清除全部PID积分
	if(index == 0xFF && id == 9 && value_fp32 == 0)
	{
		for(int i = 0; i < 8; i++) GetPID(i).error_integration = 0;

		LOGF("Clear interation of all pids.");
		Beeper::Beep(0b101111, 6);
	}
	else
	{
		PIDController &tpid = GetPID(index);
		float value = fp32_to_float(value_fp32);
		if(id == 1)      tpid.kp = value;
		else if(id == 2) tpid.ki = value;
		else if(id == 3) tpid.kd = value;
		else if(id == 9) tpid.error_integration = value;

		LOGF("Setting PID[");
		LOG(index);
		LOGF("][");
		LOG(id);
		LOGF("] => ");
		LOGLN(value);
		Beeper::Beep(0b101011, 6);
	}
}

//设定某个PID的某个参数
void Model::SetPIDParameter(u8 index, s16 kp, s16 ki, s16 kd)
{
	PIDController &tpid = GetPID(index);
	tpid.kp = kp / 1000.0f;
	tpid.ki = ki / 1000.0f;
	tpid.kd = kd / 1000.0f;
	//tpid.error_integration = value;

	LOGF("Set PID[");
	LOG(index);
	LOGF("] p:");
	LOG(tpid.kp);
	LOGF(" i:");
	LOG(tpid.ki);
	LOGF(" d:");
	LOGLN(tpid.kd);
	Beeper::Beep(0b101011, 6);
}

void Model::ConfigPID(u8 index, byte u8)
{
	if(index == 0xFF)
	{
		// 清除全部PID积分
		for(int i = 0; i < 8; i++) GetPID(i).error_integration = 0;

		LOGF("Clear interation of all pids.\n");
		Beeper::Beep(0b101111, 6);
	}
	else
	{
		GetPID(index).error_integration = 0;
		LOGF("Clear interation of pid[");
		LOG(index);
		LOGF("]\n");
		Beeper::Beep(0b110111, 6);
	}
}

//----------------------//

void Model::Calibrate()
{

#ifdef USE_HL_MPU6050
	mpu.Calibrate();
	mpu.Reset();
#else
	mpu.calcOffsets(true, true);
	mpu._Reset();
#endif

	SetPIDParameter(0xFF, 9, 0);

	status.last_update_time = 0;

	LOGF("Calibarate Finish...\n");
	Beeper::Beep(0b10101101);
}


//----------------------//

void Model::SetMotor(u16 throttle)
{
	control.throttle = throttle > 1000 ? 1000 : throttle;
	control.motor.writeMicroseconds(control.throttle + property.throttle.start);
}

void Model::SetServo(u8 index, float value)
{
	if(value > 1) value = 1;
	else if(value < -1) value = -1;

	//对大偏转舵片做效率校正 
	//(*) x*((x*a)+1)会超出[-1,1]范围
	//    x*((x*a)+(1-a))不会
	const float addi = 0.1f;
	//if(value > 0) value = value * (1 + value * addi);
	//else value = -value * (1 - value * addi);

	//一阶线性滤波平滑
	const float cof = 0.875f;
	float sValue = value * (1 - cof) + control.rudders[index] * cof;
	control.rudders[index] = sValue;

	// us = middle + bias + value * angle * (usPerDeg)
	s16 center = (s16)property.rudder.middle[index];
	s16 offset = (s16)(sValue * property.rudder.angle_range * (1000.0f / 180.0f));
	control.servo[index].writeMicroseconds(center + offset);
}


void Model::SetControl(u16 throttle, float x, float y, float z)
{
	SetMotor(throttle);
	z /= 2;
	SetServo(0, -y + z);
	SetServo(1, -x + z);
	SetServo(2, y + z);
	SetServo(3, x + z);
}


//----------------------------- Routine Function ------------------------------//

//从各设备上获取当前无人机状态
void Model::UpdateStatus()
{
	//更新时间
	u32 t_now_us = micros();
	//间隔时间
	u32 dt_us = t_now_us - status.last_update_time;
	//初始化时间
	if(status.last_update_time == 0)
	{
		status.delta_time_1024 = 8;
		status.delta_time = 8.0f / 1024.0f;
		status.last_update_time = t_now_us;
	}
	else
	{
		// dt_1024.1 = 1/1024s = 1000000us/1024 = 977us
		u16 dt_1024 = dt_us / 977;  //(*) 64s溢出
		// dTime钳制到[1, 255]
		status.delta_time_1024 = dt_1024 > 255 ? 255 : dt_1024 == 0 ? 1 : dt_1024;
		status.delta_time = dt_us / 1000000.0f;
		status.last_update_time = t_now_us;
		status.lost_control_timer += dt_1024;
	}

	u32 before_update_mpu_time = log.status.delta_time ? micros() : 0;
#ifdef USE_HL_MPU6050
	mpu.Update(status.delta_time);
	status.acceleration.x = mpu.GetAccX();
	status.acceleration.y = mpu.GetAccY();
	status.acceleration.z = mpu.GetAccZ();
	status.angle.x = mpu.GetAngleX();
	status.angle.y = mpu.GetAngleY();
	status.angle.z = mpu.GetAngleZ();
	status.angular_velocity.x = mpu.GetGyroX();
	status.angular_velocity.y = mpu.GetGyroY();
	status.angular_velocity.z = mpu.GetGyroZ();
#else
	mpu.update(status.delta_time);
	status.acceleration.x = -mpu.getAccX();
	status.acceleration.y = -mpu.getAccY();
	status.acceleration.z = -mpu.getAccZ();
	status.angle.x = -mpu.getAngleX();
	status.angle.y = -mpu.getAngleY();
	status.angle.z = -mpu.getAngleZ();
	status.angular_velocity.x = -mpu.getGyroX();
	status.angular_velocity.y = -mpu.getGyroY();
	status.angular_velocity.z = -mpu.getGyroZ();
#endif
	u32 after_update_mpu_time = log.status.delta_time ? micros() : 0;


	//Clamp [-540, 540] => [-180, 180]
	float ang_z = status.angle.z;
	
	if(ang_z > 180)
	{
		status.angle.z = ang_z - 360;
	}
	else if(ang_z < -180)
	{
		status.angle.z = ang_z + 360;
	}


	//**Battery**//
	status.battery_level = analogRead(PIN_BATTERY);

	//DEBUG//
	if(log.status.delta_time)
	{
		LOGF(" dT= ");
		LOG(status.delta_time_1024);

		LOGF(" / ");
		LOG(status.delta_time * 1000.0f);
		LOG("ms");

		LOGF("  MPU.dT= ");
		LOG(after_update_mpu_time - before_update_mpu_time);
		LOGF("us\n");
	}
	if(log.status.angle)
	{
		LOGF(" |r.x= ");
		LOG(status.angle.x);
		LOGF(" r.y= ");
		LOG(status.angle.y);
		LOGF(" r.z= ");
		LOGLN(status.angle.z);
	}
	if(log.status.angular_velocity)
	{
		LOGF(" |rv.x= ");
		LOG(status.angular_velocity.x);
		LOGF(" rv.y= ");
		LOG(status.angular_velocity.y);
		LOGF(" rv.z= ");
		LOGLN(status.angular_velocity.z);
	}
	if(log.status.acceleration)
	{
		LOGF(" |a.x= ");
		LOG(status.acceleration.x);
		LOGF(" a.y= ");
		LOG(status.acceleration.y);
		LOGF(" a.z= ");
		LOGLN(status.acceleration.z);
	}
}

//应用PID
void Model::UpdatePID()
{
	const float dtime = status.delta_time;
	const u8 avFactor = 32;  //32/64/100 影响角速度PID的量级

	float cmd_values[3] =
	{
		command.angle_x / 128.0f,
		command.angle_y / 128.0f,
		command.angle_z / 128.0f,
	};

	for(u8 i = 0; i < 3; ++i)
	{
		u8 flag = (1 << i);
		u8 index_pid = (i + 1) * 2;
		if(config.use_cascade.value & flag)
		{
			PIDController &pid1 = GetPID(index_pid + 0);
			PIDController &pid2 = GetPID(index_pid + 1);

			float target = cmd_values[i] * config.action_angle[i];
			float ang = status.angle.values[i];
			float vel = status.angular_velocity.values[i] / avFactor;

			float out1 = pid1.Step(target, ang, dtime);
			float out2 = pid2.Step(out1, vel, dtime);

			//LOG
			if(log.pid.value & flag)
			{
				LOGF("PID[");
				LOG(i);
				LOGF("]|O.R= ");
				LOG(out1);
				LOGF(" O.V= ");
				LOG(out2);

				LOGF(" R.I=");
				LOG(pid1.error_integration);
				LOGF(" V.I=");
				LOGLN(pid2.error_integration);
			}
		}
		else
		{
			PIDController &pid2 = GetPID(index_pid + 1);

			float target = cmd_values[i] * config.action_angle[i] / avFactor;
			float vel = status.angular_velocity.values[i] / avFactor;
			float out2 = pid2.Step(target, vel, dtime);

			//LOG
			if(log.pid.value & flag)
			{
				LOGF("PID[");
				LOG(i);
				LOGF("]|O.V= ");
				LOG(out2);
				LOGF(" I=");
				LOG(pid2.error_integration);
			}
		}
	}

	if(log.pid.value) LOGF("\n");

}


//更新PID和控制输出
void Model::UpdateControl()
{
	u16 th;
	if(state == eState_FlightMode)
	{
		//失去控制计时器超时, 转入安全模式
		if(status.lost_control_timer > 1024)
		{
			status.lost_control_timer = 0;
			TurnToSafeMode();
		}
		else
		{
			th = command.throttle;  //**用户直接控制电机油门
		}
	}

	if(state == eState_SafeMode)
	{
		//安全模式逐渐关闭电机
		if(control.throttle > 500) th = 500;
		else
		{
			s16 sth = control.throttle - status.delta_time_1024;
			th = sth < 50 ? 0 : (u16)sth;
		}
	}

	if(config.use_PID)
	{
		UpdatePID();
		//(*) 输出控制的方向还没有测试, 不清楚是正的还是反的
		//    如果反了, 把下面的负号去掉
		float o_x = -pids.VX.output;
		float o_y = -pids.VY.output;
		float o_z = -pids.VZ.output;
		SetControl(th, o_x, o_y, o_z);
	}
	else
	{
		SetControl(th, 0, 0, 0);
	}

	//LOG
	if(log.rudder)
	{
		LOGF(" |R.1= ");
		LOG(control.rudders[0]);
		LOGF(" R.2= ");
		LOG(control.rudders[1]);
		LOGF(" R.3= ");
		LOG(control.rudders[2]);
		LOGF(" R.4= ");
		LOGLN(control.rudders[3]);
	}
}



void Model::Update()
{
	if(state == eState_PowerOff) return;

	UpdateStatus();

	if(state == eState_LockMode) return;

	//锁定模式不启动电机
	//退出锁定模式时PID会重置, 所以锁定时不更新PID
	//SetMotor(0);

	UpdateControl();
}


