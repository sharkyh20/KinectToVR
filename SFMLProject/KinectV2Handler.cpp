#pragma once
#include "stdafx.h"
#include "KinectV2Handler.h"


void KinectV2Handler::initialise() {
    try {
        kVersion = KinectVersion::Version2;
        kinectImageData = std::make_unique<GLubyte[]>(KinectSettings::kinectV2Width * KinectSettings::kinectV2Height * 4);  //RGBA
        initialised = initKinect();
        if (!initialised) throw FailedKinectInitialisation;
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
void KinectV2Handler::drawKinectData() {
    if (KinectSettings::isKinectDrawn) {
        drawKinectImageData();
    }
    if (KinectSettings::isSkeletonDrawn) {
        drawTrackedSkeletons();
    }
}
void KinectV2Handler::drawKinectImageData() {

}
void KinectV2Handler::drawTrackedSkeletons() {

}

void KinectV2Handler::updateSkeletalData() {
    IBodyFrame* bodyFrame = nullptr;
    IBodyFrameReference* frameRef = nullptr;
    multiFrame->get_BodyFrameReference(&frameRef);
    frameRef->AcquireFrame(&bodyFrame);
    if (frameRef) frameRef->Release();

    if (!bodyFrame) return;

    IBody* bodies[BODY_COUNT];
    bodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);
    for (int i = 0; i < BODY_COUNT; i++) {
        bodies[i]->get_IsTracked(&isTracking);
        if (isTracking) {
            bodies[i]->GetJoints(JointType_Count, joints);
            break;
        }
    }
    if (bodyFrame) bodyFrame->Release();
}
bool KinectV2Handler::initKinect() {
    if (FAILED(GetDefaultKinectSensor(&kinectSensor))) {
        return false;
    }
    if (kinectSensor) {
        kinectSensor->get_CoordinateMapper(&coordMapper);

        kinectSensor->Open();
        kinectSensor->OpenMultiSourceFrameReader(
            FrameSourceTypes::FrameSourceTypes_Depth
            | FrameSourceTypes::FrameSourceTypes_Color
            | FrameSourceTypes::FrameSourceTypes_Body,
            &frameReader);
        return frameReader;
    }
    else {
        return false;
    }
}
void KinectV2Handler::getKinectData() {
    if (SUCCEEDED(frameReader->AcquireLatestFrame(&multiFrame))) {
        GLubyte* ptr;
        glBindBuffer(GL_ARRAY_BUFFER, colourBufferObjectId);
        ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

        if (ptr) {
            getRGBImageData(multiFrame, ptr);
        }
        glUnmapBuffer(GL_ARRAY_BUFFER);

        updateSkeletalData();
    }
    if (multiFrame) multiFrame->Release();
}
void KinectV2Handler::getRGBImageData(IMultiSourceFrame* multiFrame, GLubyte* dest) {
    IColorFrame* colorFrame = nullptr;
    IColorFrameReference* frameRef = nullptr;

    multiFrame->get_ColorFrameReference(&frameRef);
    frameRef->AcquireFrame(&colorFrame);
    if (frameRef) frameRef->Release();

    if (!colorFrame) return;

    //Get data from current frame
    colorFrame->CopyConvertedFrameDataToArray(KinectSettings::kinectV2Width*KinectSettings::kinectV2Height * 4, kinectImageData.get(), ColorImageFormat_Rgba);

    if (colorFrame) colorFrame->Release();
}