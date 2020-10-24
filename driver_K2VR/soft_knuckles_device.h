//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_device.h
//
// Implements the ITrackedDeviceServerDriver to emulate a single knuckles
// controller.  The handedness of the controller is passed into the Init
// function.
//
// It uses it's own thread to continually send pose updates to the vrsystem.
// It uses soft_knuckles_config to define the input configuration.
//
#ifndef SOFT_KNUCKLES_DEVICE_H
#define SOFT_KNUCKLES_DEVICE_H

#include <openvr_driver.h>
#include <thread>
#include <atomic>
#include "soft_knuckles_config.h"
#include <boost/thread.hpp>
#include "BaseStation.h"

using namespace vr;
using namespace std;

namespace soft_knuckles
{
	class SoftKnucklesDebugHandler;

	extern BaseStation* m_station1;

	static boost::thread m_pipeM_thread;
	static boost::thread m_pipeH_thread;

	void dlPipeM();
	void dlPipeH();

	float map(float value, float start1, float stop1, float start2, float stop2);

	extern float migi[62][4];
	extern float hidari[62][4];
	extern DriverPose_t mposet, hposet, pposet;

	void transformleftroot(float bend);
	void transformleftwrist(float bend);
	void transformleftthumb(float bend);
	void transformleftindex(float bend);
	void transformleftmiddle(float bend);
	void transformleftring(float bend);
	void transformleftpinky(float bend);

	void transformrightroot(float bend);
	void transformrightwrist(float bend);
	void transformrightthumb(float bend);
	void transformrightindex(float bend);
	void transformrightmiddle(float bend);
	void transformrightring(float bend);
	void transformrightpinky(float bend);
	void dlPipeTR();

	void transformallleft(float bend);
	void transformallright(float bend);
	void transformall(float bend);

	void UpdateSkeleton(const char* un_WhichHand, const char* un_WhichFinger, float bend);

	class SoftKnucklesDevice : public ITrackedDeviceServerDriver
	{
		friend class SoftKnucklesDebugHandler;

		uint32_t m_id;
		bool m_activated;
		IVRDriverContext* m_driver_context;
		PropertyContainerHandle_t m_tracked_device_container;
		ETrackedControllerRole m_role;
		const KnuckleComponentDefinition* m_component_definitions;
		uint32_t m_num_component_definitions;
		SoftKnucklesDebugHandler* m_debug_handler;

		static const int NUM_BONES = 31;

		DriverPose_t m_pose;
		float ibendt;
		float gtpt;

		/*m_pipeM_thread = boost::thread(dlPipeM);
		m_pipeM_thread.detach();
		m_pipeH_thread = boost::thread(dlPipeH);
		m_pipeH_thread.detach();*/

		string m_serial_number;
		string m_model_number;
		string m_render_model_name;
		vector<VRInputComponentHandle_t> m_component_handles;
		std::atomic<bool> m_running;
		boost::thread m_pose_thread;
		boost::thread m_pipeMOF_thread;
		boost::thread m_pipeHOF_thread;
		boost::thread m_pipeMF_thread;
		boost::thread m_pipeHF_thread;
		boost::thread m_pipeMROT_thread;
		boost::thread m_pipeHROT_thread;

		boost::thread m_pipeHBUT_thread;
		boost::thread m_pipeMBUT_thread;

		boost::thread m_pipeHBUUT_thread;
		boost::thread m_pipeMBUUT_thread;

		boost::thread bt;


		std::thread shm;

	public:
		SoftKnucklesDevice();
		void Init(ETrackedControllerRole role,
		          const KnuckleComponentDefinition* component_definitions,
		          uint32_t num_component_definitions,
		          SoftKnucklesDebugHandler* debug_handler);

		// implement required ITrackedDeviceServerDriver interfaces
		EVRInitError Activate(uint32_t unObjectId) override;
		void Deactivate() override;
		void EnterStandby() override;
		void Reactivate(); // TBD: how does a device become reactivated when it leave standby?
		void* GetComponent(const char* pchComponentNameAndVersion) override;
		void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
		DriverPose_t GetPose() override;
		VRBoneTransform_t* GetRfPose();
		VRBoneTransform_t* GetLfPose(float t, float i, float m, float r, float l);
		float GetThumb();
		float gpt();
		static void update_pose_thread(SoftKnucklesDevice* pthis);
		static void dlPipeHBUT(SoftKnucklesDevice* pthis);
		static void dlPipeMBUT(SoftKnucklesDevice* pthis);
		string get_serial() const;

	private:
		VRInputComponentHandle_t CreateBooleanComponent(const char* full_path);
		VRInputComponentHandle_t CreateScalarComponent(const char* full_path, EVRScalarType scalar_type,
		                                               EVRScalarUnits scalar_units);
		VRInputComponentHandle_t CreateHapticComponent(const char* name);
		VRInputComponentHandle_t CreateSkeletonComponent(const char* name, const char* skeleton_path,
		                                                 const char* base_pose_path,
		                                                 const VRBoneTransform_t* pGripLimitTransforms,
		                                                 uint32_t unGripLimitTransformCount);
		void SetProperty(ETrackedDeviceProperty prop_key, const char* prop_value);
		void SetInt32Property(ETrackedDeviceProperty prop_key, int32_t value);
		void SetBoolProperty(ETrackedDeviceProperty prop_key, int32_t value);
	};
}

#endif
