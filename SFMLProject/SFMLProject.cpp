// SFMLProject.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <iostream>

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/OpenGL.hpp>

// Kinect Includes
#include <Windows.h> //MUST BE BEFORE NUI
#include <ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

//Window Variables
int width = 640;
int height = 480;

//Kinect Variables
HANDLE rgbStream;               // The identifier of the Kinect's RGB Camera
INuiSensor* sensor;             // The Kinect Sensor

bool initKinect() {
    //Get a working Kinect Sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
        return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0)
        return false;
    
    //Initialise Sensor
    sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
    sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR,               //Depth Camera or RGB Camera?
        NUI_IMAGE_RESOLUTION_640x480,       //Image Resolution
        0,                                  //Image stream flags, e.g. near mode
        2,                                  //Number of frames to buffer
        NULL,                               //Event handle
        &rgbStream);
    return sensor;
}

void acquireKinectFrame();
void lockKinectPixelData();
void copyKinectPixelData();
void unlockKinectPixelData();
void releaseKinectFrame();



void getKinectData(GLubyte* dest) {
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0)
        return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    
    int bytesInFrameRow = LockedRect.Pitch;
    if (bytesInFrameRow != 0) {
        const BYTE* curr = (const BYTE*)LockedRect.pBits;
        const BYTE* dataEnd = curr + (width*height) * 4;

        while (curr < dataEnd) {
            *dest++ = *curr++;
        }
    }

    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}


int main()
{
    sf::RenderWindow window(sf::VideoMode(width, height), "SFML WORKS");
    
    while (window.isOpen()) 
    {

        sf::RectangleShape rect(sf::Vector2f(0.0, 0.0));
        rect.setFillColor(sf::Color::Green);
        rect.setSize(sf::Vector2f(100, 100));
        
        sf::Event event;
        while (window.pollEvent(event)) 
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        //Button Checks
      

        //Render Start Frame
        window.clear();

        //Render Here
        window.draw(rect);
      
        

        //End Frame
        window.display();
        
    }
    return 0;
}

