//#
//# HL.MPU6050库, 基于MPU6050_light库修改而成
//# 用于读取MPU6050传感器的角速度和加速度值, 通过一阶线性互补滤波获得姿态角
//# 
#pragma once
#include "Arduino.h"
#include "Wire.h"

#include "HL.Types.h"
#include "HL.Vectors.h"


#define MPU6050_ADDR             0x68
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1a
#define MPU6050_REG_GYRO_CONFIG  0x1b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_PWR_MGMT_1   0x6b

#define MPU6050_REG_ACCEL_OUT    0x3B
#define MPU6050_REG_GYRO_OUT     0x43

#define RAD_2_DEG                57.29578 // [°/rad]
//#define TEMP_LSB_2_DEGREE        340.0    // [bit/celsius]
//#define TEMP_LSB_OFFSET          12412.0

#define MPU6050_CALIB_COUNT      512
#define MPU6050_GYRO_COEFF       0.99


#define MPU6050_AXIS_X  1
#define MPU6050_AXIS_Y  2
#define MPU6050_AXIS_Z  3


//===================================================//
//无人机的坐标系定义
#define DRONE_X      DRONE_LEFT  //X轴为无人机右方
#define DRONE_Y      DRONE_FRONT //Y轴为无人机前方
#define DRONE_Z      DRONE_UP    //Z轴为无人机上方

//无人机方向对应的MPU轴向
//(*) 请根据MPU的安装方向更改
#define DRONE_LEFT   +MPU6050_AXIS_X
#define DRONE_FRONT  +MPU6050_AXIS_Y
#define DRONE_UP     +MPU6050_AXIS_Z
//===================================================//


class MPU6050
{
	public:
	MPU6050(TwoWire &i2c);

	u8 Begin();

	void Calibrate();

	void Update(float dtime);

	void Reset() { angle.z = 0; }

	fp16 GetTemperature() { return temperature; };

	float GetAccX() { return acc.x; };
	float GetAccY() { return acc.y; };
	float GetAccZ() { return acc.z; };
	
	float GetGyroX() { return gyro.x; };
	float GetGyroY() { return gyro.y; };
	float GetGyroZ() { return gyro.z; };

	float GetAccAngleX() { return acc_angle.x; };
	float GetAccAngleY() { return acc_angle.y; };

	float GetAngleX() { return angle.x; };
	float GetAngleY() { return angle.y; };
	float GetAngleZ() { return angle.z; };

	void GetAcceleration(Vector3F &acc) { acc = this->acc; }
	void GetAngularVelocity(Vector3F &av) { av = this->gyro; }
	void GetAngles(Vector3F &angles) { angles = this->angle; }
	void GetAccAngles(Vector3F &angles) { angles = this->angle; }


	private:
	//[ax, ay, az, temp, gx, gy, gz]
	void FetchRawData(s16 raws[7]);

	byte _Write(byte reg, byte data);
	byte _Read(byte reg);
	void _ReadBytes(byte reg, byte *bytes, u8 count);

	TwoWire &wire;

	// deg/s = (raw * 500) << shift
	//u8 gyro_degsec_shift_500;
	// g = (raw * 4) << shift
	//u8 acc_g_shift_4;

	//Vector3S16 gyro_offset;
	//Vector3S16 acc_offset;
	s16 offsets[6];

	fp16 temperature;
	Vector3F acc;
	Vector3F gyro;
	Vector2F acc_angle;
	Vector3F angle;

public:
	float filter_gyro_coefficient; //陀螺仪置信度
};