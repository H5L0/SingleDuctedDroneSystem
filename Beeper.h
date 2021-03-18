#pragma once
#include "HL.Types.h"
#include "Pins.h"


#define BEEPER_CYCLE 128


//蜂鸣器 *Beep* (静态)
//由于3个定时器可能都被6路PWM使用了, 所以使用轮巡实现有源蜂鸣器的节律
class Beeper
{
	static u8 pattern;     //Bit[x]=1 代表蜂鸣器响1/4s (低位开始)
	static u8 length;      //长度(>8则重复) (255: 无限循环)
	static u8 position;    //当前pattern的bit位置 [0, 8]
	static u8 wait_timer;  //蜂鸣每节时长

	public:
	static void Init();

	static void Beep(u8 pattern, u8 length = 8, bool block = false);

	static void Update(u8 delta_time_1024);
};

