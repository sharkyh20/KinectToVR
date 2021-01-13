//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_config.h
//
// Each controller device is responsible for registering each of it's input
// source paths with the vrsystem.
//
// This module is responsible for holding the configuration for the left
// and right controllers in one, table based place.
//
// This table is used by the soft_knuckles_device.cpp for registration
// and by the soft_knuckles_debug_handler.cpp to convert between strings
// (input source paths) and interface handles
//
// For more information on inputsource type to components, 
// look at the section "input source path" at: 
// https://github.com/ValveSoftware/openvr/wiki/Input-Profiles
// and
// https://github.com/ValveSoftware/openvr/wiki/Creating-a-Skeletal-Input-Driver
// 
#pragma once
#include <openvr_driver.h>

namespace soft_knuckles
{
	using namespace vr;
	static const char* kSettingsSection = "soft_knuckles";

	enum ComponentType
	{
		CT_BOOLEAN,
		CT_SCALAR,
		CT_SKELETON,
		CT_HAPTIC,
	};

	struct KnuckleComponentDefinition
	{
		const char* full_path;
		ComponentType component_type;
		EVRScalarType scalar_type;
		EVRScalarUnits scalar_units;
		const char* skeleton_path;
		const char* base_pose_path;
	};

	extern const int NUM_INPUT_COMPONENT_DEFINITIONS;
	extern const KnuckleComponentDefinition component_definitions_left[];
	extern const KnuckleComponentDefinition component_definitions_right[];
};
