#include "HL.MPU6050.h"
#ifdef USE_HL_MPU6050


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
byte MPU6050::_ReadBytes(byte reg, byte *bytes, u8 count)
{
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(reg);
	wire.endTransmission(false);
	wire.requestFrom(MPU6050_ADDR, count);
	byte data = wire.read();
	return data;
}
*/


void MPU6050::FetchRawData(s16 raws[7])
{
	wire.beginTransmission(MPU6050_ADDR);
	wire.write(MPU6050_REG_ACCEL_OUT);
	wire.endTransmission(false);
	wire.requestFrom(MPU6050_ADDR, 14);

	//s16 raws[7]; // [ax,ay,az,temp,gx,gy,gz]
	//_ReadBytes(MPU6050_REG_ACCEL_OUT, (byte*)raws , 14)

	for(int i = 0; i < 7; i++)
	{
		raws[i] = wire.read() << 8;
		raws[i] |= wire.read();
	}
}


MPU6050::MPU6050(TwoWire &i2c) :
	wire(i2c),
	gyro_degsec_shift_500(1), acc_g_shift_4(0), filter_gyro_coefficient_shift(6)
	//gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
{
}


u8 MPU6050::Begin()
{
	_Write(MPU6050_REG_SMPLRT_DIV, 0x00);
	_Write(MPU6050_REG_CONFIG, 0x00);
	//TODO: Check status
	byte status = _Write(MPU6050_REG_PWR_MGMT_1, 0x01);

	_Write(MPU6050_REG_GYRO_CONFIG, 0x08);
	_Write(MPU6050_REG_ACCEL_CONFIG, 0x00);

	return status;
}


void MPU6050::Calibrate()
{
	s16 raws[7]; // [ax,ay,az,temp,gx,gy,gz]
	s32 sums[7];

	for(int i = 0; i < 1024; i++)
	{
		FetchRawData(raws);
		sums[0] += raws[0];
		sums[1] += raws[1];
		sums[2] += raws[2] - 16384;
		sums[4] += raws[4];
		sums[5] += raws[5];
		sums[6] += raws[6];
		delay(1);
	}

	acc_offset.x = sums[0] >> 10;
	acc_offset.y = sums[1] >> 10;
	acc_offset.z = sums[2] >> 10;

	gyro_offset.x = sums[4] >> 10;
	gyro_offset.y = sums[5] >> 10;
	gyro_offset.z = sums[6] >> 10;
}


void MPU6050::Update(u8 delta_time_1024s)
{
	s16 raws[7]; // [ax,ay,az,temp,gx,gy,gz]

	FetchRawData(raws);

	//fp32: fixed point shift = 16
	acc.x = ((fp32)raws[0] - acc_offset.x) << (acc_g_shift_4 + 2);
	acc.y = ((fp32)raws[1] - acc_offset.y) << (acc_g_shift_4 + 2);
	acc.z = ((fp32)raws[2] - acc_offset.z) << (acc_g_shift_4 + 2);
	temperature = (fp16)((((s32)raws[3] + 12412) << FP16_SHIFT) / 340);
	gyro.x = (((fp32)raws[4] - gyro_offset.x) * 500) << gyro_degsec_shift_500;
	gyro.y = (((fp32)raws[5] - gyro_offset.y) * 500) << gyro_degsec_shift_500;
	gyro.z = (((fp32)raws[6] - gyro_offset.z) * 500) << gyro_degsec_shift_500;


	double magY = sqrt((double)(acc.z * acc.z + acc.x * acc.x));
	double magX = sqrt((double)(acc.z * acc.z + acc.y * acc.y));

	if(acc.z < 0) magY = -magY;
	angle_acc_x =  (fp32)(atan2(acc.y, magY) * (RAD_2_DEG * FP32_1));
	angle_acc_y = -(fp32)(atan2(acc.x, magX) * (RAD_2_DEG * FP32_1));

	delta_time_us >>= 6;  //16 * (16 - 6) => (-32 ~ 32)
	fp32 new_gyro_angle_x = angle.x + (gyro.x * delta_time_us) / 15625;
	fp32 new_gyro_angle_y = angle.y + (gyro.y * delta_time_us) / 15625;
	fp32 new_gyro_angle_z = angle.z + (gyro.z * delta_time_us) / 15625;

	//u8 filter_count = (1 << filter_gyro_coefficient_shift) - 1;
	//angle.x = (nx * filter_count + angle_acc_x) >> shift;
	//angle.x = (nx * ((1 << shift) - 1) + angle_acc_x) >> shift;
	//angle.x = ((nx << shift) - nx + angle_acc_x) >> shift;

	angle.x = (
		((new_gyro_angle_x << filter_gyro_coefficient_shift) - new_gyro_angle_x) + angle_acc_x
		) >> filter_gyro_coefficient_shift;
	angle.y = (
		((new_gyro_angle_y << filter_gyro_coefficient_shift) - new_gyro_angle_y) + angle_acc_y
		) >> filter_gyro_coefficient_shift;

	angle.z = new_gyro_angle_z;
}

#endif