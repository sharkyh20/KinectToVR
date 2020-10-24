//////////////////////////////////////////////////////////////////////////////
// dprintf.h
// common logging functionality.  
// In debug builds, logs to a file as well as to vr::VRDriverLog and outputdebugstring
// In release builds, logs only to vr::VRDriverLog
//
#pragma once
extern void dprintf(const char* fmt, ...);
