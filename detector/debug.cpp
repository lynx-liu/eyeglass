///////////////////////////////////////////////////
//				date: 2025.03.09
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include <windows.h>
#include <stdio.h>
#include "debug.h"

#define _CRT_SECURE_NO_WARNINGS

//打印日志到Output窗口
void debug(char* fmt, ...) {
	char out[MAX_PATH] = {};
	va_list body;
	va_start(body, fmt);
	vsprintf_s(out, MAX_PATH, fmt, body);
	va_end(body);
	OutputDebugStringA(out);
}