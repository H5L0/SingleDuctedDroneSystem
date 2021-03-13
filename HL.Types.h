#pragma once


typedef int8_t   s8;
typedef uint8_t  u8;
typedef int16_t  s16;
//typedef uint16_t u16;
typedef int32_t  s32;
typedef uint32_t u32;


//16位定点数 8位小数
typedef int16_t fp16;  //bit of fraction = 8  range = [-128.996, 127.996]  ps = 0.004
#define FP16_SHIFT 8
#define FP16_1  (((fp16)1) << FP16_SHIFT)
#define FP16_H5 (((fp16)1) << (FP16_SHIFT - 1))

//typedef int16_t fp16_12; //fixed point, bit of fraction = 12, range = [0, 15.99975]

//32位定点数 16位小数
typedef int32_t fp32;  //bit of fraction = 12   ** => 16
#define FP32_SHIFT 16
#define FP32_1  (((fp32)1) << FP32_SHIFT)


inline float fp32_to_float(fp32 value) { return value / (float)FP32_1; }



