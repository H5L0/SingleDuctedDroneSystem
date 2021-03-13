#pragma once
#include "HL.Types.h"


class PIDControllerFP16
{
	public:
	fp16 kp;
	fp16 ki;
	fp16 kd;

	//static float T = 1.0f / 32;

	//float input;
	fp16 output;

	//上一误差
	fp16 last_error;

	//误差积分
	fp16 error_integration;


	PIDControllerFP16() { }
	//PIDControllerFP16(fp32 kp32, fp32 ki32, fp32 kd32) : kp(kp32), ki(ki32), kd(kd32) { }
	PIDControllerFP16(fp16 kp16, fp16 ki16, fp16 kd16) : kp(kp16), ki(ki16), kd(kd16) { }


	//input: 目标值
	//state: 系统当前输出值 s{7}.{8}  p = 0.0039  r = [-128.996, 127.996]
	//time:  更新周期  u0.00{8}  p = 0.0009766s = 0.9766ms  r = (0, 0.2490] = (0, 249ms] (不可为0）
	fp16 Step(fp16 input, fp16 state, u8 time)
	{
		//当前误差
		fp16 this_error = input - state;
		//误差微分 f = 8
		s32 this_diff = (((s32)(this_error - last_error)) << 10) / time;  //**可能溢出2位

		last_error = this_error;

		//误差积分 f = 8
		error_integration += (fp16)(((s32)this_error * time) >> 10);

		if(error_integration >  FP16_1) error_integration = FP16_1;
		else if(error_integration < -FP16_1) error_integration = -FP16_1;

		//             fp16 * ( fp16    +  ( fp16 * s32(f8) + fp16  *  s32(f8) ))
		output = (fp16)((kp * (this_error + ((kd * this_diff + ki * error_integration) >> FP16_SHIFT))) >> FP16_SHIFT);
		return output;
	}

};



class PIDController
{
	public:
	float kp;
	float ki;
	float kd;

	float output;

	//上一误差
	float last_error;

	//误差积分
	float error_integration;


	PIDController() {}
	PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}


	//input: 目标值
	//state: 系统当前输出值
	//time:  更新周期
	float Step(float input, float state, float time)
	{
		//当前误差
		float this_error = input - state;
		//误差微分
		float this_diff = (this_error - last_error) / time;

		last_error = this_error;

		//误差积分
		error_integration += this_error * time;

		if(error_integration > 1.0f) error_integration = 1.0f;
		else if(error_integration < -1.0f) error_integration = -1.0f;

		output = kp * (this_error + kd * this_diff + ki * error_integration);
		return output;
	}

};
