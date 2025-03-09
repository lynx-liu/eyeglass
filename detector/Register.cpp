///////////////////////////////////////////////////
//				date: 2025.03.09
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include <windows.h>
#include <intrin.h>
#include <codecvt>
#include "Register.h"

Register::Register()
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    code = converter.to_bytes(L"QQ:515311445");
    registed = isRegisted(code.c_str());
}

Register::~Register()
{
    registed = false;
}

bool Register::isRegisted() {
    return registed;
}

bool Register::showQQ() {
    return code.find("QQ") == 0;
}

std::string Register::getMark() {
    return code;
}

bool Register::isRegisted(const char* code) {
    DWORD dwCode = 0;
    DWORD dwSerial = 0;
    DWORD dwIDESerial = 0;
    //GetVolumeInformation("\\", NULL, NULL, &dwIDESerial, NULL, NULL, NULL, NULL);
    int cpuInfo[4] = { 0 };
    __cpuid(cpuInfo, 1); // EAX=1 查询 Processor ID
    dwIDESerial = (cpuInfo[0] & 0xFFFFFFFFl + cpuInfo[3] & 0xFFFFFFFFl) & 0xFFFFFFFFl;//cpuInfo[1]可能每次不一样

    sscanf_s(code, "%x", &dwCode);
    while (dwCode)
    {
        dwSerial = dwSerial * 10 + dwCode % 10;
        dwCode /= 10;
    }

    return dwSerial == dwIDESerial;
}