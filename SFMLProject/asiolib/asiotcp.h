#pragma once

#ifdef ASIOTCP_EXPORTS
#define ASIOTCP_API __declspec(dllexport)
#else
#define ASIOTCP_API __declspec(dllimport)
#endif

extern "C" ASIOTCP_API void pingport(int port);