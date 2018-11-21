// KinectV1Process.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "KinectV1Handler.h"
#include <KinectToVR.h>
#include <openvr.h>
#include <Windows.h>

#include <easylogging++.h>

// Logging Definitions
INITIALIZE_EASYLOGGINGPP
#define ELPP_FEATURE_CRASH_LOG

int main(int argc, char* argv[])
{
    START_EASYLOGGINGPP(argc, argv);
#ifndef _DEBUG 
    HWND hWnd = GetConsoleWindow();
    ShowWindow(hWnd, SW_HIDE);
#endif 
    KinectV1Handler kinect;
    // Reversed for now to investigate perf. bug
    KinectSettings::leftFootJointWithRotation = KVR::KinectJointType::AnkleLeft;
    KinectSettings::rightFootJointWithRotation = KVR::KinectJointType::AnkleRight;
    KinectSettings::leftFootJointWithoutRotation = KVR::KinectJointType::FootLeft;
    KinectSettings::rightFootJointWithoutRotation = KVR::KinectJointType::FootRight;
    processLoop(kinect);
    
    return 0;
}
/*
#ifdef _WIN32
// This disables the console window from appearing on windows only if the Project Settings->Linker->System->SubSystem is set to Windows (rather than Console).
int WinMain(HINSTANCE hinstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCMDShow)
{
	KinectV1Handler kinect;

	processLoop(kinect);

	return 0;
}
#endif
*/