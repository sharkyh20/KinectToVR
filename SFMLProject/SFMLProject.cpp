// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "Kinect.h"
#include "sfLine.h"
//Window Variables
const int width = 640;
const int height = 480;

//OpenGL Variables
GLuint textureId;              // ID of the texture to contain Kinect RGB Data
GLubyte data[width*height * 4];  // BGRA array containing the texture data

//Kinect Variables
HANDLE rgbStream;               // The identifier of the Kinect's RGB Camera
INuiSensor* sensor;             // The Kinect Sensor
//Skeleton Tracking
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];  //Body Tracking
sf::Vector2f m_points[NUI_SKELETON_POSITION_COUNT]; // Converted to screen space
float g_TrackedBoneThickness = 3.0f;
float g_InferredBoneThickness = 2.0f;


bool initKinect() {
    //Get a working Kinect Sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
        return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0)
        return false;
    
    //Initialise Sensor
    sensor->NuiInitialize(
        NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX 
        | NUI_INITIALIZE_FLAG_USES_COLOR 
        | NUI_INITIALIZE_FLAG_USES_SKELETON);
    
    sensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR,               //Depth Camera or RGB Camera?
        NUI_IMAGE_RESOLUTION_640x480,       //Image Resolution
        0,                                  //Image stream flags, e.g. near mode
        2,                                  //Number of frames to buffer
        NULL,                               //Event handle
        &rgbStream);
        
    sensor->NuiSkeletonTrackingEnable(
        NULL,
        0       // Enable seat support (Upper body only)
    );
    return sensor;
}




void getKinectData(GLubyte* dest) {
    NUI_IMAGE_FRAME imageFrame{};
    NUI_LOCKED_RECT LockedRect{};
    if (acquireKinectFrame(imageFrame)) {
        std::cout << "Get Frame Failed!!!!!!" << std::endl;
        return;
    }
    INuiFrameTexture* texture = lockKinectPixelData(imageFrame, LockedRect);
    copyKinectPixelData(LockedRect, dest);
    unlockKinectPixelData(texture);

    releaseKinectFrame(imageFrame);
}

bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame)
{
    return (sensor->NuiImageStreamGetNextFrame(rgbStream, 1, &imageFrame) < 0);
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



void processSkeleton(sf::RenderWindow &window) {
    NUI_SKELETON_FRAME skeletonFrame = { 0 };
    getSkeletalData(skeletonFrame);
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            // We're tracking the skeleton, draw it
            DrawSkeleton(skeletonFrame.SkeletonData[i], window);
        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState) {
            //ONLY CENTER POINT TO DRAW
        }
    }
}
void getSkeletalData(NUI_SKELETON_FRAME &skeletonFrame) {
    if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
        sensor->NuiTransformSmooth(&skeletonFrame, NULL);   //Smooths jittery tracking
        //Loop over all sensed skeletons
        for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
            const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
            //Check the state of the skeleton
            if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {
                // Copy the joint positions into the array
                for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
                    skeletonPosition[i] = skeleton.SkeletonPositions[i];
                    if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
                        skeletonPosition[i].w = 0;
                    }
                }
                return; //Only take the data for one skeleton
            }
        }
    }
}

void drawKinectImageData() {
    getKinectData(data);

    glBindTexture(GL_TEXTURE_2D, textureId);
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

void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window) {
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
        m_points[i] = SkeletonToScreen(skeletonPosition[i], width, height);
    }
    // Render Torso
    DrawBone(skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER, window);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, window);

    // Left Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT, window);

    // Right Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT, window);

    // Left Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, window);

    // Right Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, window);

    /*
    // Draw the joints in a different color
    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(m_Points[i], g_JointThickness, g_JointThickness);

    if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED)
    {
    m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointInferred);
    }
    else if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED)
    {
    m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
    }
    }
    */
}
sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height) {
    LONG x, y;
    USHORT depth;

    // Calculate the skeleton's position on the screen
    // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
    NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

    float screenPointX = static_cast<float>(x * _width) / 320;
    float screenPointY = static_cast<float>(y * _height) / 240;

    return sf::Vector2f(screenPointX, screenPointY);
}
void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
    NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];

    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }

    // Don't draw if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }
    // Assume all bones are inferred unless BOTH joints are tracked
    if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
    {
        DrawLine(m_points[joint0], m_points[joint1], sf::Color::Green, g_TrackedBoneThickness, window);
    }
    else
    {
        DrawLine(m_points[joint0], m_points[joint1], sf::Color::Cyan, g_InferredBoneThickness, window);
    }
}
void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
    window.pushGLStates();
    
    //sf::VertexArray VertexLines(sf::Lines);

    sfLine line(start, end);
    line.setColor(colour);
    window.draw(line);
    /*
    VertexLines.append(start);
    VertexLines[0].color = colour;
    VertexLines.append(end);
    VertexLines[1].color = colour;

    window.draw(VertexLines);
    */
    window.popGLStates();
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
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        window.clear();

        //Render Here
        
        drawKinectImageData();
        
        
        processSkeleton(window);
        
        //End Frame
        window.display();
        
    }
    return 0;
}

