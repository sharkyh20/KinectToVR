// KinectV2Process.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "KinectV2Handler.h"
#include <KinectToVR.h>

int main()
{
    KinectV2Handler kinect;
    processLoop(kinect);
    return 0;
}

