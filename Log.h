#pragma once

#define DEBUG

//#define LOG(str) do{(Serial.print(F(str)}while(0);

#ifdef DEBUG
#define LOG Serial.print
#define LOGLN Serial.println
#else
#define LOG
#define LOGLN
#endif

