//========= Copyright Valve Corporation ============//
#include "logger.h"

#include <stdio.h>
#include <stdarg.h>

/** Provides printf-style line logging via the vr::IVRDriverLog interface provided by SteamVR
* during initialization.  Client logging ends up in vrclient_appname.txt and server logging
* ends up in vrserver.txt.
*/
namespace soft_knuckles
{
	// TODO make this a singleton logger per class for convenience

	static vr::IVRDriverLog* s_pLogFile = nullptr;

#if !defined( WIN32)
#define vsnprintf_s vsnprintf
#endif

	// Initialises the IVRDriverLog logger.
	bool Logger::InitDriverLog(vr::IVRDriverLog* pDriverLog)
	{
		if (s_pLogFile)
			return false;
		s_pLogFile = pDriverLog;
		return s_pLogFile != nullptr;
	}

	// Cleans up the IVRDriverLog logger.
	void Logger::CleanupDriverLog()
	{
		s_pLogFile = nullptr;
	}


	// Takes a given message and using vsprint
	void Logger::DriverLogVarArgs(const char* pMsgFormat, va_list args, std::string logLevel)
	{
		char buf[1024];
#if defined( WIN32 )
		vsprintf_s(buf, pMsgFormat, args);
#else
		vsnprintf(buf, sizeof(buf), pMsgFormat, args);
#endif

		if (s_pLogFile)
			s_pLogFile->Log((logLevel + buf).c_str());
	}

	/** Logs a printf-style info line logging.
	*/
	void Logger::Info(const char* pMsgFormat, ...)
	{
		va_list args;
		va_start(args, pMsgFormat);
		DriverLogVarArgs(pMsgFormat, args, "INFO - ");
		va_end(args);
	}

	/** Logs a printf-style warn line logging.
	*/
	void Logger::Warn(const char* pMsgFormat, ...)
	{
		va_list args;
		va_start(args, pMsgFormat);
		DriverLogVarArgs(pMsgFormat, args, "WARN - ");
		va_end(args);
	}

	/** Logs a printf-style error line logging.
	*/
	void Logger::Error(const char* pMsgFormat, ...)
	{
		va_list args;
		va_start(args, pMsgFormat);
		DriverLogVarArgs(pMsgFormat, args, "ERROR - ");
		va_end(args);
	}

	/** Logs a printf-style debug line logging.
	*/
	void Logger::Debug(const char* pMsgFormat, ...)
	{
#ifdef _DEBUG
		va_list args;
		va_start(args, pMsgFormat);
		Logger::DriverLogVarArgs(pMsgFormat, args, "DEBUG - ");
		va_end(args);
#endif
	}
}
