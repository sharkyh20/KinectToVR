// PSMSProcess.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <KinectToVR.h>
#include <openvr.h>
#include <Windows.h>

// Logging Definitions
INITIALIZE_EASYLOGGINGPP
const char* logConfigFileName = "logging.conf";
const char* logConfigDefault =
"* GLOBAL:\n"
"	FORMAT = \"[%level] %datetime{%Y-%M-%d %H:%m:%s}: %msg\"\n"
"	FILENAME = \"K2VR.log\"\n"
"	ENABLED = true\n"
"	TO_FILE = true\n"
"	TO_STANDARD_OUTPUT = true\n"
"	MAX_LOG_FILE_SIZE = 2097152 ## 2MB\n"
"* TRACE:\n"
"	ENABLED = false\n"
"* DEBUG:\n"
"	ENABLED = false\n";

void init_logging() {
    el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
    el::Configurations conf(logConfigFileName);
    conf.parseFromText(logConfigDefault);
    conf.parseFromFile(logConfigFileName);
    conf.setRemainingToDefault();
    el::Loggers::reconfigureAllLoggers(conf);
}

//This process is intended to be run in situations without a kinect
// i.e. at time of writing, the SteamVRBridge driver for PSMoves is broke
// and I just want to use the moves as trackers, so the process will be run without the kinect

class FakeKinect : public KinectHandlerBase {
public:
    FakeKinect() {
        initialised = true;
        isPSMS = true; //Notify Parent that we're using PSMS
    }
    ~FakeKinect() {

    }
};

int main(int argc, char* argv[])
{
    START_EASYLOGGINGPP(argc, argv);
    init_logging();
    HWND hWnd = GetConsoleWindow();
    ShowWindow(hWnd, SW_SHOW);
#ifndef _DEBUG 
    ShowWindow(hWnd, SW_HIDE);
#endif 
    FakeKinect kinect;

    processLoop(kinect);

    return 0;
}
