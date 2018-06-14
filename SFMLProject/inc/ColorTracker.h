#pragma once
#include <opencv2\opencv.hpp>
#include <string>
#include <vector>

#include "TrackingMethod.h"

using namespace cv;

void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed
}
struct HSVFilter {
    int H_MIN = 0;
    int H_MAX = 256;
    int S_MIN = 0;
    int S_MAX = 256;
    int V_MIN = 0;
    int V_MAX = 256;
};
struct TrackedColorComponent {
    HSVFilter filter;
    int imagePosX = 0;
    int imagePosY = 0;
};
class ColorTracker : public TrackingMethod {
public:
    ColorTracker(int frameWidth, int frameHeight) : FRAME_HEIGHT(frameHeight), FRAME_WIDTH(frameWidth) {
        cv::setUseOptimized(true);
        MaximumObjectArea = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
    }
    ~ColorTracker() {

    }
    void initialise() {
        //addBlueFilter();
        addTrackedComponent();
        createTrackbars();

        namedWindow(colorFeedWindow, WINDOW_NORMAL);
        namedWindow(hsvWindow, WINDOW_NORMAL);
        namedWindow(thresholdWindow, WINDOW_NORMAL);
        namedWindow(depthWindow, WINDOW_NORMAL);
        resizeWindow(colorFeedWindow, 640, 360);
        resizeWindow(hsvWindow, 640, 360);
        resizeWindow(thresholdWindow, 640, 360);
        resizeWindow(depthWindow, 640, 360);

        active = true;

    }
    void terminate() {
        destroyAllWindows();
        active = false;
    }
    std::vector<TrackedColorComponent> getTrackedPoints() {
        return trackedComponents;
    }
    void update(KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers) {
        Mat imageFeed = kinect.colorMat;
        Mat depthFeed = kinect.depthMat;
        if (!active) { return; }


        if (imageFeed.empty() || depthFeed.empty())
            return;

        int scaleReductionFactor = 3;
        int convertedWidth = FRAME_WIDTH;
        int convertedHeight = FRAME_HEIGHT;
        if (scaleImageForProcessing) {
            int convertedWidth = FRAME_WIDTH / scaleReductionFactor;
            int convertedHeight = FRAME_HEIGHT / scaleReductionFactor;
            resize(imageFeed, imageFeed, Size( convertedHeight, convertedWidth), 0.5, 0.5 , INTER_AREA); // For some reason, height and width need to be swapped around, as otherwise PositionX is really PositionY
        }
        MaximumObjectArea = convertedWidth * convertedHeight / 1.5;
        //convert frame from BGR to HSV colorspace
        cvtColor(imageFeed, imageHSV, COLOR_BGR2HSV);
        
        for (int i = 0; i < trackedComponents.size(); i++) {
            //filter HSV image between values and store filtered image to
            //threshold matrix
            TrackedColorComponent& trackedComponent = trackedComponents[currentTrackedIndex];
            auto &f = trackedComponent.filter;
            inRange(imageHSV, 
                Scalar(f.H_MIN, f.S_MIN, f.V_MIN),
                Scalar(f.H_MAX, f.S_MAX, f.V_MAX), imageThresholded);
            //perform morphological operations on thresholded image to eliminate noise
            //and emphasize the filtered object(s)
            if (useMorphOps)
                morphOps(imageThresholded);
            //pass in thresholded frame to our object tracking function
            //this function will return the x and y coordinates of the
            //filtered object
            if (trackObjects) {
                bool objectTracked = trackFilteredObject(trackedComponent.imagePosX, trackedComponent.imagePosY, imageThresholded, imageFeed);
                if (objectTracked && scaleImageForProcessing) {
                    trackedComponent.imagePosX *= scaleReductionFactor;
                    trackedComponent.imagePosY *= scaleReductionFactor;
                }
            }

            drawPicker(convertedHeight /2, convertedWidth / 2, imageFeed); // For some weird reason using FRAME width/height constants causes drawing glitches

            //show frames 
            imshow(thresholdWindow, imageThresholded);
            imshow(colorFeedWindow, imageFeed);
            imshow(hsvWindow, imageHSV);
            imshow(depthWindow, depthFeed);
        }
        //delay 30ms so that screen can refresh.
        //image will not appear without this waitKey() command
        waitKey(1);

    }
    void updateTrackers(KinectHandlerBase& kinect, std::vector<KVR::KinectTrackedDevice> & v_trackers) {
        //Temporary, need to seperate kinect calculations and the updating of the trackers
        sf::Vector2i colorCoords = { getTrackedPoints()[0].imagePosX, getTrackedPoints()[0].imagePosY };
        kinect.updateTrackersWithColorPosition(v_trackers, colorCoords);
    }
    void addTrackedComponent() {
        TrackedColorComponent c;
        trackedComponents.push_back(c);
    }
    /*
    void addBlueFilter() {
        HSVFilter f;
        f.H_MIN = 44;
        f.H_MAX = 126;
        f.S_MIN = 160;
        f.S_MAX = 251;
        f.V_MIN = 88;
        f.V_MAX = 256;
        filters.push_back(f);
    }
    */
private:
    bool useMorphOps = true;
    bool trackObjects = true;
    bool scaleImageForProcessing = false;

    uchar lastPickedHSV {};

    Mat lastImageFeed = Mat();
    Mat lastDepthFeed = Mat();

    Mat imageHSV;
    Mat imageThresholded;

    std::vector<TrackedColorComponent> trackedComponents;
    int currentTrackedIndex = 0;
    const int MaxTrackedObjects = 50;
    int FRAME_WIDTH = 1920;
    int FRAME_HEIGHT = 1080;
    
    int MinimumObjectArea = 5 * 5;
    int MaximumObjectArea = FRAME_HEIGHT * FRAME_WIDTH / 1.5; // Needs to change for Xbone

    //names that will appear at the top of each window
    const std::string colorFeedWindow = "Original Image";
    const std::string depthWindow = "Depth Image";
    const std::string hsvWindow = "HSV Image";
    const std::string thresholdWindow = "Thresholded Image";
    const std::string windowName3 = "After Morphological Operations";
    const std::string trackbarWindowName = "Trackbars";
        
    std::string intToString(int number) {
        std::stringstream ss;
        ss << number;
        return ss.str();
    }
    void createTrackbars() {
        cv::namedWindow(trackbarWindowName, WINDOW_NORMAL);

        HSVFilter& f = trackedComponents[currentTrackedIndex].filter;
        createTrackbar("INDEX", trackbarWindowName, &currentTrackedIndex, 10);
        createTrackbar("H_MIN", trackbarWindowName, &f.H_MIN, f.H_MAX, on_trackbar);
        cv::createTrackbar("H_MAX", trackbarWindowName, &f.H_MAX, f.H_MAX, on_trackbar);
        cv::createTrackbar("S_MIN", trackbarWindowName, &f.S_MIN, f.S_MAX, on_trackbar);
        cv::createTrackbar("S_MAX", trackbarWindowName, &f.S_MAX, f.S_MAX, on_trackbar);
        cv::createTrackbar("V_MIN", trackbarWindowName, &f.V_MIN, f.V_MAX, on_trackbar);
        cv::createTrackbar("V_MAX", trackbarWindowName, &f.V_MAX, f.V_MAX, on_trackbar);
    }
    void drawObject(int x, int y, Mat &frame) {

        //use some of the openCV drawing functions to draw crosshairs
        //on your tracked image!

        //UPDATE:JUNE 18TH, 2013
        //added 'if' and 'else' statements to prevent
        //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

        circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
        if (y - 25>0)
            line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
        if (y + 25<FRAME_HEIGHT)
            line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
        if (x - 25>0)
            line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
        if (x + 25<FRAME_WIDTH)
            line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

        putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

    }
    void drawPicker(int x, int y, Mat & frame) {
        auto RED = Scalar(0, 0, 255);
        line(frame, Point(x - 25, y), Point(x + 25, y), RED, 3);
        line(frame, Point(x , y-25), Point(x, y+25), RED, 3);
    }
    void morphOps(Mat &thresh) {

        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle

        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);


        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);



    }
    bool trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {

        Mat temp;
        threshold.copyTo(temp);
        //these two vectors needed for output of findContours
        std::vector< std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;
        //find contours of filtered image using openCV findContours function
        findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if (numObjects<MaxTrackedObjects) {
                for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                    Moments moment = moments((cv::Mat)contours[index]);
                    double area = moment.m00;

                    //if the area is less than 20 px by 20px then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if (area>MinimumObjectArea && area<MaximumObjectArea && area>refArea) {
                        x = moment.m10 / area;
                        y = moment.m01 / area;
                        objectFound = true;
                        refArea = area;
                    }
                    else objectFound = false;

                }
                //let user know you found an object
                if (objectFound == true) {
                    putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                    //draw object location on screen
                    drawObject(x, y, cameraFeed);
                }

            }
            else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
        return objectFound;
    }
};

