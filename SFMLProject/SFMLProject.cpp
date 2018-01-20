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
const int width = 640;
const int height = 480;

//OpenGL Variables
GLuint textureId;              // ID of the texture to contain Kinect RGB Data
GLubyte data[width*height * 4];  // BGRA array containing the texture data

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

bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame);
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect);
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest);
void unlockKinectPixelData(INuiFrameTexture* texture);
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame);



void getKinectData(GLubyte* dest) {
    NUI_IMAGE_FRAME imageFrame{};
    NUI_LOCKED_RECT LockedRect{};
    if (acquireKinectFrame(imageFrame))
        return;
    INuiFrameTexture* texture = lockKinectPixelData(imageFrame, LockedRect);
    copyKinectPixelData(LockedRect, dest);
    unlockKinectPixelData(texture);

    releaseKinectFrame(imageFrame);
}
bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame)
{
    return (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0);
}
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect)
{
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    return imageFrame.pFrameTexture;
}
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest)
{
    int bytesInFrameRow = LockedRect.Pitch;
    if (bytesInFrameRow != 0) {
        const BYTE* curr = (const BYTE*)LockedRect.pBits;
        const BYTE* dataEnd = curr + (width*height) * 4;

        while (curr < dataEnd) {
            *dest++ = *curr++;
        }
    }
}
void unlockKinectPixelData(INuiFrameTexture* texture)
{
    texture->UnlockRect(0);
}
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame)
{
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void drawKinectData() {
    
    glBindTexture(GL_TEXTURE_2D, textureId);
    getKinectData(data);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(width, 0, 0);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(width, height, 0.0f);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(0, height, 0.0f);
    
    glEnd();
    
}


int main()
{
    sf::RenderWindow window(sf::VideoMode(width, height), "SFML WORKS");
    if (!initKinect()) return 1;

    // Initialize textures
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
        0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // OpenGL setup
    glClearColor(0, 0, 0, 0);
    glClearDepth(1.0f);
    glEnable(GL_TEXTURE_2D);

    // Camera setup
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();



    while (window.isOpen()) 
    {

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
        //Prepare for drawing
        drawKinectData();
        //End Frame
        window.display();
        
    }
    return 0;
}

