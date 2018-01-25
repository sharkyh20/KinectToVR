// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "KinectToVR.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode(SFMLsettings::m_window_width, SFMLsettings::m_window_height), "SFML WORKS");
    sf::Clock clock;
    
    KinectHandler kinect;
    initOpenGL(kinect.kinectTextureId, kinect.kinectImageData.get());
    NUI_SKELETON_FRAME skeletonFrame = { 0 };
    updateSkeletalData(skeletonFrame, kinect.kinectSensor);
    zeroAllTracking(skeletonFrame);

    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();

    KinectTrackedDevice leftFootTracker(inputEmulator, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, false);
    KinectTrackedDevice rightFootTracker(inputEmulator, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, false);
    KinectTrackedDevice hipTracker(inputEmulator, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, false);
    
    //KinectTrackedDevice kinectTrackerRef(inputEmulator, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_HEAD, true);
    //setKinectTrackerProperties(kinectTrackerRef.deviceId);

    std::vector<KinectTrackedDevice> v_trackers{};
    v_trackers.push_back(leftFootTracker);
    v_trackers.push_back(rightFootTracker);
    v_trackers.push_back(hipTracker);
    //v_trackers.push_back(kinectTrackerRef);

    while (window.isOpen()) 
    {
        sf::Event event;
        while (window.pollEvent(event)) 
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }

        //Clear
        window.clear();
        
        
        //Process
        if (KinectSettings::userChangingZero) {
            
            zeroAllTracking(skeletonFrame);
        }
        updateSkeletalData(skeletonFrame, kinect.kinectSensor);
        updateTrackersWithSkeletonPosition(inputEmulator, v_trackers, skeletonFrame);
        //Draw
        
        if (KinectSettings::isKinectDrawn)
            drawKinectImageData(kinect);
        drawTrackedSkeletons( skeletonFrame, window);
       
        //End Frame
        window.display();
    }
    destroyTrackers(inputEmulator, v_trackers);
    return 0;
}

