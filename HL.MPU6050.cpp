#include "HL.MPU6050.h"

byte MPU6050::_Write(byte reg, byte data)
{
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(reg);
	wire.write(data);
	byte status = wire.endTransmission();
	return status; // 0 if success
}

byte MPU6050::_Read(byte reg)
{
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(reg);
	wire.endTransmission(true);
	wire.requestFrom(MPU6050_ADDR, 1);
	byte data = wire.read();
	return data;
}

/*
void MPU6050::_ReadBytes(byte reg, byte *bytes, u8 count)
{
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(reg);
	wire.endTransmission(false);
	wire.requestFrom(MPU6050_ADDR, count);
	for (u8 i = 0; i < count; ++i) bytes[i] = wire.read();
}
*/

MPU6050::MPU6050(TwoWire &i2c) :
	wire(i2c),
	//gyro_degsec_shift_500(1), acc_g_shift_4(0),
	filter_gyro_coefficient(MPU6050_GYRO_COEFF),
	//gyro_offset({0, 0, 0}),
	angle({0, 0, 0}),
	acc_angle({0, 0})
{
}


u8 MPU6050::Begin()
{
	_Write(MPU6050_REG_SMPLRT_DIV, 0x00);
	_Write(MPU6050_REG_CONFIG, 0x00);
	//TODO: Check status
	byte status = _Write(MPU6050_REG_PWR_MGMT_1, 0x01);

	_Write(MPU6050_REG_GYRO_CONFIG, 0x08);  //+- 500 °/s
	_Write(MPU6050_REG_ACCEL_CONFIG, 0x00); //+- 2 g

	return status;
}


void MPU6050::FetchRawData(s16 raws[7])
{
	//仅适用小端模式
	//_ReadBytes(MPU6050_REG_ACCEL_OUT, (byte*)raws , 14)
	
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(MPU6050_REG_ACCEL_OUT);
	wire.endTransmission(false);
	wire.requestFrom((int)MPU6050_ADDR, 14);

	//MPU6050数据是大端模式
	for(u8 i = 0; i < 7; i++)
	{
		raws[i] = ((s16)wire.read() << 8) | (s16)wire.read();
	}
}


void MPU6050::Calibrate()
{
	s16 raws[7]; // [ax,ay,az,temp,gx,gy,gz]
	s32 sums[6] = {0, 0, 0, 0, 0, 0};

	for(u32 i = 0; i < MPU6050_CALIB_COUNT; i++)
	{
		FetchRawData(raws);
		sums[0] += raws[0];
		sums[1] += raws[1];
		sums[2] += raws[2];  //sums[2] += raws[2] - 16384;
		sums[3] += raws[4];
		sums[4] += raws[5];
		sums[5] += raws[6];
		delay(1);
	}

	//acc_offset.x = sums[0] / MPU6050_CALIB_COUNT;
	//acc_offset.y = sums[1] / MPU6050_CALIB_COUNT;
	//acc_offset.z = sums[2] / MPU6050_CALIB_COUNT;
	//gyro_offset.x = sums[3] / MPU6050_CALIB_COUNT;
	//gyro_offset.y = sums[4] / MPU6050_CALIB_COUNT;
	//gyro_offset.z = sums[5] / MPU6050_CALIB_COUNT;

	for (u8 i = 0; i < 6; i++)
	{
		offsets[i] = sums[i] / MPU6050_CALIB_COUNT;
	}
}


#define AXIS_INDEX(name) ((name) >= 0 ? ((name) - 1) : (-(name) - 1))
#define AXIS_SIGN(name) ((name) >= 0 ? +1 : -1)

void MPU6050::Update(float dtime)
{
	s16 raws[7]; 
	FetchRawData(raws);

	acc.x = AXIS_SIGN(DRONE_X) * (raws[AXIS_INDEX(DRONE_X)] - offsets[AXIS_INDEX(DRONE_X)]) / 16384.0f;
	acc.y = AXIS_SIGN(DRONE_Y) * (raws[AXIS_INDEX(DRONE_Y)] - offsets[AXIS_INDEX(DRONE_Y)]) / 16384.0f;
	acc.z = AXIS_SIGN(DRONE_Z) * (raws[AXIS_INDEX(DRONE_Z)] - offsets[AXIS_INDEX(DRONE_Z)]) / 16384.0f + 1.0f;
	temperature = (fp16)((((s32)raws[3] + 12412) << FP16_SHIFT) / 340);
	gyro.x = AXIS_SIGN(DRONE_X) * (raws[4 + AXIS_INDEX(DRONE_X)] - offsets[3 + AXIS_INDEX(DRONE_X)]) / 65.5f;
	gyro.y = AXIS_SIGN(DRONE_Y) * (raws[4 + AXIS_INDEX(DRONE_Y)] - offsets[3 + AXIS_INDEX(DRONE_Y)]) / 65.5f;
	gyro.z = AXIS_SIGN(DRONE_Z) * (raws[4 + AXIS_INDEX(DRONE_Z)] - offsets[3 + AXIS_INDEX(DRONE_Z)]) / 65.5f;

	double magY = sqrt(acc.z * acc.z + acc.x * acc.x);
	double magX = sqrt(acc.z * acc.z + acc.y * acc.y);
	if(acc.z < 0) magY = -magY;
	float nax =  atan2(acc.y, magY) * RAD_2_DEG;
	float nay = -atan2(acc.x, magX) * RAD_2_DEG;

	//float nax =  atan2(acc.y, acc.z) * RAD_2_DEG;
	//float nay = -asin(acc.x) * RAD_2_DEG;

	//先对加速度计解算的姿态角做一阶线性插值滤波
 	acc_angle.x = (acc_angle.x * 3 + nax) / 4;
	acc_angle.y = (acc_angle.y * 3 + nay) / 4;

	float ngx = angle.x + gyro.x * dtime;
	float ngy = angle.y + gyro.y * dtime;
	float ngz = angle.z + gyro.z * dtime;

	//angle.x = filter_gyro_coefficient * ngx + (1.0 - filter_gyro_coefficient) * acc_angle.x;
	//angle.y = filter_gyro_coefficient * ngy + (1.0 - filter_gyro_coefficient) * acc_angle.y;
	angle.x = acc_angle.x + filter_gyro_coefficient * (ngx - acc_angle.x);
	angle.y = acc_angle.y + filter_gyro_coefficient * (ngy - acc_angle.y);
	angle.z = ngz;
}