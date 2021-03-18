#pragma once
#include "HL.Types.h"
#include "Pins.h"


#define BEEPER_CYCLE 128


//������ *Beep* (��̬)
//����3����ʱ�����ܶ���6·PWMʹ����, ����ʹ����Ѳʵ����Դ�������Ľ���
class Beeper
{
	static u8 pattern;     //Bit[x]=1 �����������1/4s (��λ��ʼ)
	static u8 length;      //����(>8���ظ�) (255: ����ѭ��)
	static u8 position;    //��ǰpattern��bitλ�� [0, 8]
	static u8 wait_timer;  //����ÿ��ʱ��

	public:
	static void Init();

	static void Beep(u8 pattern, u8 length = 8, bool block = false);

	static void Update(u8 delta_time_1024);
};

