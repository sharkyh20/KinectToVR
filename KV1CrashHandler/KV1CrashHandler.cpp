// KV1CrashHandler.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>

#include "KV1ModHandler.h"

BOOL IsProcessRunning(DWORD pid)
{
    HANDLE process = OpenProcess(SYNCHRONIZE, FALSE, pid);
    DWORD ret = WaitForSingleObject(process, 0);
    CloseHandle(process);
    return ret == WAIT_TIMEOUT;
}

int main(int argc, char* argv[])
{
    // Hide console window
    ShowWindow(GetConsoleWindow(), SW_HIDE);
	
    if (argc > 1) {
        auto pid = atoi(argv[1]);
        while (IsProcessRunning(pid))
        {
            // Sleep not to load out CPU
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

	// Open the kinect to get its handle
    KinectV1Handler kinect;
    kinect.update();

	// Shut down the kinect
    kinect.terminateColor();
    kinect.terminateDepth();
    kinect.terminateSkeleton();
    
	return 0;
}
