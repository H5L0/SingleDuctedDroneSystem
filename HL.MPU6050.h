#pragma once
//#define USE_HL_MPU6050
#ifdef USE_HL_MPU6050

#include "Arduino.h"
#include "Wire.h"

#include "HL.Types.h"
#include "HL.Vectors.h"

#define MPU6050_ADDR                  0x68
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1a
#define MPU6050_REG_GYRO_CONFIG  0x1b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_PWR_MGMT_1   0x6b

#define MPU6050_REG_ACCEL_OUT    0x3B
#define MPU6050_REG_GYRO_OUT     0x43

#define RAD_2_DEG             57.29578 // [¡ã/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98



class MPU6050
{
	public:
	MPU6050(TwoWire &i2c);

	u8 Begin();

	void Calibrate();

	void Update(u16 delta_time_us);

	void Reset() { angle.z = 0; }

	fp16 GetTemperature() { return temperature; };

	fp32 GetAccX() { return acc.x; };
	fp32 GetAccY() { return acc.y; };
	fp32 GetAccZ() { return acc.z; };
	
	fp32 GetGyroX() { return gyro.x; };
	fp32 GetGyroY() { return gyro.y; };
	fp32 GetGyroZ() { return gyro.z; };

	fp32 GetAccAngleX() { return angle_acc_x; };
	fp32 GetAccAngleY() { return angle_acc_y; };

	fp32 GetAngleX() { return angle.x; };
	fp32 GetAngleY() { return angle.y; };
	fp32 GetAngleZ() { return angle.z; };

	void GetAcceleration(Vector3FP32 &acc) { acc = this->acc; }
	void GetAngularVelocity(Vector3FP32 &av) { av = this->gyro; }
	void GetAngles(Vector3FP32 &angles) { angles = this->angle; }


	private:
	void FetchRawData(s16 raws[7]);

	byte _Write(byte reg, byte data);
	byte _Read(byte reg);
	byte _ReadBytes(byte reg, byte *bytes, u8 count);

	TwoWire &wire;

	// deg/s = (raw * 500) << shift
	u8 gyro_degsec_shift_500;
	// g = (raw * 4) << shift
	u8 acc_g_shift_4;

	Vector3S16 gyro_offset;
	Vector3S16 acc_offset;

	fp16 temperature;
	Vector3FP32 acc;
	Vector3FP32 gyro;
	Vector3FP32 angle;

	fp32 angle_acc_x;
	fp32 angle_acc_y;

	u8 filter_gyro_coefficient_shift; //ÍÓÂÝÒÇÖÃÐÅ¶È ((1<<shift) - 1) / (1<<shift) (* = 6)
};

#endif