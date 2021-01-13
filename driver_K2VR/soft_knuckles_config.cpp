//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_config.cpp
// 
// See header for description
//
#include "soft_knuckles_config.h"

#define COMPONENT_DEFINITIONS(HAND) \
    {\
        { "/input/joystick/x", CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided},\
        { "/input/joystick/y",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided },\
        { "/input/joystick/click",  CT_BOOLEAN },\
        { "/input/joystick/touch",  CT_BOOLEAN },\
     \
        { "/input/trackpad/x",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided },\
        { "/input/trackpad/y",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided },\
        { "/input/trackpad/click",  CT_BOOLEAN },\
        { "/input/trackpad/touch",  CT_BOOLEAN },\
     \
        { "/input/thumbstick/x",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided },\
        { "/input/thumbstick/y",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided },\
        { "/input/thumbstick/click",  CT_BOOLEAN },\
        { "/input/thumbstick/touch",  CT_BOOLEAN },\
     \
        { "/input/trigger/value",   CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
        { "/input/trigger/click",   CT_BOOLEAN },\
        { "/input/trigger/touch",   CT_BOOLEAN },\
     \
        { "/input/grip/force",      CT_BOOLEAN, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
        { "/input/grip/click",      CT_BOOLEAN },\
        { "/input/grip/touch",      CT_BOOLEAN },\
     \
        { "/input/finger/index",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
        { "/input/finger/middle",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
        { "/input/finger/ring",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
        { "/input/finger/pinky",      CT_SCALAR, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided },\
     \
        { "/input/system/click",    CT_BOOLEAN },\
        { "/input/system/touch",    CT_BOOLEAN },\
        { "/input/a/click", CT_BOOLEAN },\
        { "/input/a/touch", CT_BOOLEAN },\
        { "/input/b/click", CT_BOOLEAN },\
        { "/input/b/touch", CT_BOOLEAN },\
        { "/input/application_menu/click",  CT_BOOLEAN },\
        { "/input/application_menu/touch",  CT_BOOLEAN },\
        { "/input/skeleton/" HAND, CT_SKELETON,EVRScalarType(0),EVRScalarUnits(0),"/skeleton/hand/" HAND, "/pose/raw" },\
     \
        { "/output/haptic", CT_HAPTIC },\
    }

namespace soft_knuckles
{
	const KnuckleComponentDefinition component_definitions_left[] = COMPONENT_DEFINITIONS("left");
	const KnuckleComponentDefinition component_definitions_right[] = COMPONENT_DEFINITIONS("right");
	const int NUM_INPUT_COMPONENT_DEFINITIONS = sizeof(component_definitions_left) / sizeof(component_definitions_left[0
	]);
}
