#include "Beeper.h"
#include "Arduino.h"


 u8 Beeper::pattern    = 0;
 u8 Beeper::length     = 0;
 u8 Beeper::position   = 0;
 u8 Beeper::wait_timer = 0;


void Beeper::Init()
{
	pinMode(PIN_BEEPER, OUTPUT);
}


void Beeper::Beep(u8 pattern, u8 length, bool block)
{
	//��δ��ʼloopѭ��, ��������
	if(block)
	{
		//���ṩ��������
		for(u8 i = 0; i != length; i++)
		{
			u8 pos = i & 0b111;
			u8 beep = (pattern >> pos) & 1;
			digitalWrite(PIN_BEEPER, beep);
			delay(250);
		}
		digitalWrite(PIN_BEEPER, LOW);
	}
	else
	{
		Beeper::length = length;
		Beeper::wait_timer = 0;
		Beeper::pattern = pattern;
		Beeper::position = 0;

		digitalWrite(PIN_BEEPER, pattern & 1);
	}
}


void Beeper::Update(u8 delta_time_1024)
{
	//�޷���������
	if(pattern == 0) return;

	u16 t = wait_timer + delta_time_1024;
	if(t > BEEPER_CYCLE)
	{
		wait_timer = 0;
		position++;

		//Pattern����
		if(length != 255 && position == length)
		{
			length = 0;
			pattern = 0;
			digitalWrite(PIN_BEEPER, LOW);
		}
		//�л���Pattern��һ��
		else
		{
			u8 pos = position & 0b111;
			u8 beep = (pattern >> pos) & 1;
			digitalWrite(PIN_BEEPER, beep);
		}

	}
	else wait_timer = (u8)t;
}