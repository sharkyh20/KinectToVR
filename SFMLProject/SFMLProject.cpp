// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "KinectToVR.h"



/** VR controller button and axis IDs */
/*
enum EVRButtonId
{
    k_EButton_System = 0,
    k_EButton_ApplicationMenu = 1,
    k_EButton_Grip = 2,
    k_EButton_DPad_Left = 3,
    k_EButton_DPad_Up = 4,
    k_EButton_DPad_Right = 5,
    k_EButton_DPad_Down = 6,
    k_EButton_A = 7,

    k_EButton_ProximitySensor = 31,

    k_EButton_Axis0 = 32,
    k_EButton_Axis1 = 33,
    k_EButton_Axis2 = 34,
    k_EButton_Axis3 = 35,
    k_EButton_Axis4 = 36,

    // aliases for well known controllers
    k_EButton_SteamVR_Touchpad = k_EButton_Axis0,
    k_EButton_SteamVR_Trigger = k_EButton_Axis1,

    k_EButton_Dashboard_Back = k_EButton_Grip,

    k_EButton_Max = 64
};
*/
sf::Vector2f GetControllerAxisValue(vr::EVRButtonId buttonId, vr::VRControllerState_t state_)
{
    sf::Vector2f axis_value;
    uint64_t axisId = (uint64_t)buttonId - (uint64_t)vr::k_EButton_Axis0;
    switch (axisId)
    {
    case 0: axis_value = sf::Vector2f(state_.rAxis[0].x, state_.rAxis[0].y); break;
    case 1: axis_value = sf::Vector2f(state_.rAxis[1].x, state_.rAxis[1].y); break;
    case 2: axis_value = sf::Vector2f(state_.rAxis[2].x, state_.rAxis[2].y); break;
    case 3: axis_value = sf::Vector2f(state_.rAxis[3].x, state_.rAxis[3].y); break;
    case 4: axis_value = sf::Vector2f(state_.rAxis[4].x, state_.rAxis[4].y); break;
    }
    return axis_value;

}
//controller input
bool GetPress(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) { return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0; }
bool GetPressDown(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) {
    return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0;
} // && (prev_state_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) == 0; }
bool GetPressUp(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) {
    return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) == 0;
} //&& (prev_state_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) != 0; }

bool GetTouch(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) {  return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0; }
bool GetTouchDown(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) { return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0; }// && (prev_state_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) == 0; }
bool GetTouchUp(vr::EVRButtonId buttonId, vr::VRControllerState_t state_) { return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) == 0; }// && (prev_state_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) != 0; }

int main()
{
    sf::RenderWindow window(sf::VideoMode(SFMLsettings::m_window_width, SFMLsettings::m_window_height), "SFML WORKS");
    sf::Clock clock;
    
    sf::Font font;
    sf::Text text;
    // Global Debug Font

    font.loadFromFile("arial.ttf");
    text.setFont(font);
    text.setString("");
    text.setCharacterSize(40);
    text.setFillColor(sf::Color::Red);

    window.draw(text);
    KinectHandler kinect;
    initOpenGL(kinect.kinectTextureId, kinect.kinectImageData.get());
    NUI_SKELETON_FRAME skeletonFrame = { 0 };
    updateSkeletalData(skeletonFrame, kinect.kinectSensor);
    

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
        std::stringstream ss;
        //VR input -- Holy shit fuck the openvr 'docs'
        vr::EVRInitError eError = vr::VRInitError_None;
        vr::IVRSystem *m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Utility);
        bool m_rbShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];

        //https://github.com/zecbmo/ViveSkyrim/blob/master/Source/ViveSupport.cpp - Has quite a few useful bits of stuff that the docs don't tell
        vr::TrackedDeviceIndex_t rightControllerId = m_VRSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
        vr::TrackedDeviceIndex_t leftControllerId = m_VRSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
        vr::VRControllerState_t state;
        if (m_VRSystem->GetControllerState(rightControllerId, &state, sizeof(state)))
        {
            //Works
            if (GetPress(vr::EVRButtonId::k_EButton_A, state)) {    //Works
                KinectSettings::userChangingZero = true;
            }
        }
        //Clear
        window.clear();
        
        
        //Process
        updateSkeletalData(skeletonFrame, kinect.kinectSensor);
        zeroAllTracking(skeletonFrame);
        if (KinectSettings::userChangingZero) {
            if (GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad, state)) { //works
                
                //zeroAllTracking(skeletonFrame);
                sf::Vector2f axis = GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad, state); //works
                KinectSettings::trackedPositionOffset[0] += 0.04f * axis.x;
                KinectSettings::trackedPositionOffset[2] +=  0.04f * axis.y;
                ss << "Axis value = " << axis.x << ", " << axis.y << '\n';
            }/*
            else if (GetPress(vr::EVRButtonId::k_EButton_A, state)){
                KinectSettings::userChangingZero = false;
            }
            */
            
            
            
            
            //If clock elapsed > 2 seconds, set new zero

            //If left arm above head, restart clock

            //If clock elapsed > 2 seconds, set new zero
            
        }
        
        updateTrackersWithSkeletonPosition(inputEmulator, v_trackers, skeletonFrame);
        ss << "userchangingzero = " << KinectSettings::userChangingZero << '\n';
        text.setString(ss.str());
           
        
        //Draw
        
        if (KinectSettings::isKinectDrawn)
            drawKinectImageData(kinect);
        drawTrackedSkeletons( skeletonFrame, window);
        //Draw debug font
        window.pushGLStates();
        window.resetGLStates();
        window.draw(text);
        window.popGLStates();
        //End Frame
        window.display();
    }
    destroyTrackers(inputEmulator, v_trackers);
    return 0;
}

