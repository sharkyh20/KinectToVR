#pragma once
#include <string>
#include <vector>
#include <windows.h>

class IKinectHandler {
    // Interface base for Kinectv1 and v2
public:
    virtual ~IKinectHandler() {}

    virtual void initOpenGL() = 0;
    virtual void initialise() = 0;

    virtual HRESULT getStatusResult() = 0;
    virtual std::string statusResultString(HRESULT stat) = 0;

    virtual void update() = 0;

    bool isInitialised() { return initialised; }
    bool isZeroed() { return zeroed; }

    bool zeroed = false;
protected:
    bool initialised= false;

    class FailedKinectInitialisation : public std::exception
    {
        virtual const char* what() const throw()
        {
            return "Failure to initialise the kinect sensor. Is it plugged in and supplied with power?";
        }
    } FailedKinectInitialisation;
private:
};
