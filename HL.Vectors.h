#pragma once
#include "HL.Types.h"

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


inline void Vector3FToVector3FP16(const Vector3F &v3f, Vector3FP16 &v3fp16)
{
	v3fp16.x = (fp16)(v3f.x * FP16_1);
	v3fp16.y = (fp16)(v3f.y * FP16_1);
	v3fp16.z = (fp16)(v3f.z * FP16_1);
}


struct Vector3FP32
{
	fp32 x;
	fp32 y;
	fp32 z;
};

inline void Vector3FP32ToVector3FP16(const Vector3FP32 &v3fp32, Vector3FP16 &v3fp16)
{
	v3fp16.x = (fp16)(v3fp32.x >> (FP32_SHIFT - FP16_SHIFT));
	v3fp16.y = (fp16)(v3fp32.y >> (FP32_SHIFT - FP16_SHIFT));
	v3fp16.z = (fp16)(v3fp32.z >> (FP32_SHIFT - FP16_SHIFT));
}


struct Vector3S16
{
	s16 x;
	s16 y;
	s16 z;
};


