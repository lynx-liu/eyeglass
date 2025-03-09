///////////////////////////////////////////////////
//				date: 2025.03.09
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "debug.h"

//打印日志到Output窗口
void debug(char* fmt, ...) {
	char out[1024];
	va_list body;
	va_start(body, fmt);
	vsprintf(out, fmt, body);
	va_end(body);
	OutputDebugStringA(out);
}