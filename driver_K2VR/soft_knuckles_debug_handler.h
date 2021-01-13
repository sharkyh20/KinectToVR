//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_debug_handler.h
//
// The "soft" in soft_knuckles is to be able to use software
// to simulate a knuckles controller.   To simulate a knuckles controller, the
// debug_handler is is registered on each soft_knuckles device and receives
// requests to change input component states.
//
// This module uses the DebugRequest mechanism provided by 
// ITrackedDeviceServerDriver and uses the input configuration provided by
// soft_knuckles_config.h
//
// See soft_knuckles_debug_client.cpp for an example client.
//
#pragma once
#include <openvr_driver.h>
#include <unordered_map>

class SoftKnucklesDevice;

namespace soft_knuckles
{
	enum ControllerButtonInputType : unsigned char
	{
		CBIT_None = 0U,
		CBIT_Boolean,
		CBIT_Float
	};

	class CControllerButton
	{
		VRInputComponentHandle_t m_handle;
		float m_value;
		bool m_state;
		ControllerButtonInputType m_inputType;
		bool m_updated;
	public:
		CControllerButton();
		~CControllerButton();

		VRInputComponentHandle_t GetHandle() const { return m_handle; }
		VRInputComponentHandle_t& GetHandleRef() { return m_handle; }

		void SetInputType(ControllerButtonInputType f_type) { m_inputType = f_type; }
		ControllerButtonInputType GetInputType() const { return m_inputType; }

		void SetValue(float f_value);
		float GetValue() const { return m_value; }

		void SetState(bool f_state);
		bool GetState() const { return m_state; }

		bool IsUpdated() const { return m_updated; }
		void ResetUpdate() { m_updated = false; }
	};

	class SoftKnucklesDebugHandler
	{
		SoftKnucklesDevice* m_device;
		std::unordered_map<std::string, uint32_t> m_inputstring2index;

	public:
		SoftKnucklesDebugHandler();
		void Init(SoftKnucklesDevice*);

		void SetPosition(double x, double y, double z);
		void SetRotation(double yaw, double roll, double pitch);
		void UpdateSkeleton(const char* un_WhichHand, const char* un_WhichFinger, float bend);
		void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);
		void DebugRequestE();

	private:
		void InitializeLookupTable();
	};
};
