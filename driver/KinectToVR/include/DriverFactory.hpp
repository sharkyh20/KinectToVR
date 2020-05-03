#pragma once

#include <string>

#include <openvr_driver.h>

#include "ServerDriver.hpp"

/// <summary>
/// A define for exporting our factory function with C linkage
/// </summary>
#define EXPORT extern "C" __declspec(dllexport)

/// <summary>
/// Driver factory function
/// </summary>
/// <param name="interface_name">Requested interface name</param>
/// <param name="return_code">Return code, 0 if no error</param>
/// <returns>Non-owning pointer to the requested interface</returns>
EXPORT void* HmdDriverFactory(const char *interface_name, int *return_code);