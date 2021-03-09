#pragma once


typedef int8_t   s8;
typedef uint8_t  u8;
typedef int16_t  s16;
//typedef uint16_t u16;
typedef int32_t  s32;
typedef uint32_t u32;


typedef int32_t fp32;  //bit of fraction = 12
typedef int16_t fp16;  //bit of fraction = 8  range = [-128.996, 127.996]  ps = 0.004

//typedef int16_t fp16_12; //fixed point, bit of fraction = 12, range = [0, 15.99975]

#define FP16_SHIFT 8

#define FP16_1 (1 << FP16_SHIFT)
#define FP16_H5 (1 << (FP16_SHIFT - 1))

