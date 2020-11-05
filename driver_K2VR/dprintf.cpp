//////////////////////////////////////////////////////////////////////////////
// dprintf.h
// See header file for description.
//
#include <stdarg.h>
#include <stdio.h>
#include <openvr_driver.h>
#include <chrono>
#include <ctime>

#if defined(_WIN32)
#include <windows.h>
// disable vc fopen warning
#pragma warning(disable : 4996)
#define vsprintf vsprintf_s
#endif

void dprintf(const char* fmt, ...)
{
	va_list args;
	char buffer[2048];

	va_start(args, fmt);
	vsprintf(buffer, fmt, args);
	va_end(args);

#if !defined(NDEBUG)
    static FILE *factory_log;
    if (factory_log == 0)
    {
        factory_log = fopen("C:\\.K2VR_log", "wt"); //open only
        if (!factory_log)
        {
            factory_log = fopen("C:\\.K2VR_log", "wt"); //open only
        }
    }

    if (factory_log)
    {
        fprintf(factory_log, "%s", buffer);
        fflush(factory_log);
    }

#ifdef WIN32
	OutputDebugStringA(buffer);
#endif

#endif

	if (vr::VRDriverContext() && vr::VRDriverLog())
	{
		vr::VRDriverLog()->Log(buffer);
	}
}
