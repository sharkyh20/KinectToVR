// KinectV2Process.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "KinectV2Handler.h"
#include <KinectToVR.h>
#include <sstream>
#include <string>
#include <iostream>
int main() {
    KinectV2Handler kinect;
    processLoop(kinect);
    return 0;
}

#ifdef _WIN32
// This disables the console window from appearing on windows only if the Project Settings->Linker->System->SubSystem is set to Windows (rather than Console).
int WinMain(HINSTANCE hinstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCMDShow)
{
    KinectV2Handler kinect;

    processLoop(kinect);

    return 0;
}
#endif