#pragma once

#define DEBUG

//#define LOG(str) do{(Serial.print(F(str)}while(0);

#ifdef DEBUG
#define LOG Serial.print
#define LOGLN Serial.println
#define LOGF(str) Serial.print(F(str))
#else
#define LOG
#define LOGLN
#define LOGF(str)
#endif

