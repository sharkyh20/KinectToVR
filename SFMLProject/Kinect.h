#pragma once
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
bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame);
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect);
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest);
void unlockKinectPixelData(INuiFrameTexture* texture);
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame);
void getSkeletalData(NUI_SKELETON_FRAME &skeletonFrame);
void drawKinectImageData();

void processSkeleton(sf::RenderWindow &window);
void getSkeletalData(NUI_SKELETON_FRAME &skeletonFrame);
void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window);
void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
    NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window);
void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window);
sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height);