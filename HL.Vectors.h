#pragma once
#include "Types.h"

struct Vector3F
{
	float x;
	float y;
	float z;
};

struct Vector3FP16
{
	fp16 x;
	fp16 y;
	fp16 z;
};


void Vector3ToVector3FP16(const Vector3F &v3f, Vector3FP16 &v3fp16)
{
	v3fp16.x = (fp16)(v3f.x * (1 << FP16_SHIFT));
	v3fp16.y = (fp16)(v3f.y * (1 << FP16_SHIFT));
	v3fp16.z = (fp16)(v3f.z * (1 << FP16_SHIFT));
}


struct Vector3FP32
{
	fp32 x;
	fp32 y;
	fp32 z;
};


struct Vector3S16
{
	s16 x;
	s16 y;
	s16 z;
};


