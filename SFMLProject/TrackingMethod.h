#pragma once

#include "stdafx.h"
#include <string>
#include <vector>
enum class {
};
struct RawTrackedPoint {

};
struct SolvedTrackedPoint {

};
class TrackingMethod {
public:
    TrackingMethod() {}
    ~TrackingMethod() {}

    void initialise() {}
    void terminate() {}

    void update() {}

    void updateTrackers() {}

};