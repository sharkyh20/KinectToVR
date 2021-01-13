//========= Copyright Valve Corporation ============//
#pragma once

#include <string>
#include <openvr_driver.h>

namespace soft_knuckles
{
	class Logger
	{
	public:
		static bool InitDriverLog(vr::IVRDriverLog* pDriverLog);
		static void CleanupDriverLog();
		static void DriverLogVarArgs(const char* pMsgFormat, va_list args, std::string logLevel);
		static void Info(const char* pchFormat, ...);
		static void Debug(const char* pchFormat, ...);
		static void Warn(const char* pchFormat, ...);
		static void Error(const char* pchFormat, ...);
	};
}
