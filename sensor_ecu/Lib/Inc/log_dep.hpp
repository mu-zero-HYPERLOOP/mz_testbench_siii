/*
 * log.h
 *
 *  Created on: Apr 12, 2023
 *      Author: karl
 */

#ifndef LOG_H_
#define LOG_H_
#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "math.h"

#ifdef DEBUG

/**
 * logs a Null terminated c-style string through Serial Wire Interface SWI
 */
inline void log(const char *str) {
	while (*str != '\0') {
		ITM_SendChar(*str);
		str++;
	}
}

inline void logln() {
	ITM_SendChar('\n');
}

/**
 * logs a Null terminated c-style string through Serial Wire Interface SWI
 * and appends a line break at the end.
 */
inline void logln(const char *str) {
	log(str);
	logln();
}

/**
 * logs a format Null terminated c-style string through Serial Wire Interface SWI
 * and appends a line break at the end.
 * The resulting string cannot exceed 512 characters.
 */
inline void logf(const char *fmt, ...) {
	static char fmtStr[512];
	va_list args;
	va_start(args, fmt);
	unsigned long len = vsnprintf(fmtStr, 512, fmt, args);
	va_end(args);
	for (unsigned long i = 0; i < len; i++) {
		ITM_SendChar(fmtStr[i]);
	}
}

inline void logValue(const char *title, uint64_t value) {
	log(title);
	log(" = ");
	static char str[64] = { };
	char *sptr = str + 64 - 2; //last character (before \0).
	do {
		unsigned int remainder = value % 10;
		*sptr-- = ('0' + remainder);
		value /= 10; //implicit floor.
	} while (value != 0);
	sptr++;
	log(sptr);
}

inline void loglnValue(const char *title, uint64_t value) {
	logValue(title, value);
	logln();
}

inline void logValue(const char *title, float value) {
	log(title);
	log(" = ");
	if (value < 0) {
		ITM_SendChar('-');
		value = -value;
	}

	uint64_t u64Value = value; //implicit floor.
	static char str[128];
	char *cptr = str + 64;
	char *sptr = cptr - 1;
	do {
		unsigned int remainder = u64Value % 10; //implicit floor.
		*sptr-- = ('0' + remainder);
		u64Value /= 10; //implicit floor.
	} while (u64Value != 0);
	sptr++;

	value = fmod(value, 1.0f); //really slow =^/
	if(value == 0){
		*cptr = '\0';
	}else{
		*cptr = '.';
		char* eptr = cptr + 1;
		for(unsigned int i=0;i<6;i++){
			value *= 10;
			unsigned int digit = value; //implicit floor.
			value -= digit;
			*eptr++ = '0' + digit;
		}
		*eptr = '\0';
	}
	log(sptr);
}

inline void loglnValue(const char *title, float value) {
	logValue(title, value);
	logln();
}

#else

inline void log(const str* str){ }
inline void logln(const char* str){ }
inline void logf(const char* fmt, ...);
inline void loglnValue(const char *title, uint64_t value);
inline void loglnValue(const char *title, float value);

#endif

#endif /* LOG_H_ */
