// KinectV1Process.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "KinectV1Handler.h"
#include <KinectToVR.h>
#include <openvr.h>

int main()
{

    KinectV1Handler kinect;
    processLoop(kinect);
    return 0;
}

