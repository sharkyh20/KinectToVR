//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_provider.cpp
//
// This is the "main" of the entire driver.  It defines HmdDriverFactory
// below which is the entry point from the vrserver into this shared library.
//
// It also defines SoftKnucklesProvider that waits on a listen socket thread
// thread trigger instantiating the the soft_knuckles_devices.   When it
// does receive a new connection, it creates left and right handed
// soft_knuckles_devices.
//
// While it is possible to instantiate the controllers at startup, 
// sometimes you may want to start other controllers like the vive controller
// earlier so that you can use a tested and real controller and then add the 
// simulated ones in later.
//
//
//#if defined( _WIN32 )
//#define WIN32_LEAN_AND_MEAN
//#include <windows.h>
//#endif

#include <Windows.h>
#include <thread>
#include <string.h>
#include <string>
#include <openvr_driver.h>
#include "soft_knuckles_device.h"
#include "soft_knuckles_debug_handler.h"
#include "socket_notifier.h"
#include "BodyTracker.h"
#include "dprintf.h"
#include <boost/interprocess/managed_shared_memory.hpp>
#pragma comment(lib, "Ws2_32.lib")

using namespace vr;
bool activatedSpawned = false;

namespace soft_knuckles
{
	static const int NUM_DEVICES = 2;
	static const int BASES = 3;
	static const char* listen_address = "127.0.0.1";
	static const unsigned short listen_port = 5741;
	bool activated = false;

	class SoftKnucklesProvider;

	class SoftKnucklesSocketNotifier : public SocketNotifier
	{
		SoftKnucklesProvider* m_provider;
	public:
		SoftKnucklesSocketNotifier(SoftKnucklesProvider* p);
		void Notify() override;
	};

	class SoftKnucklesProvider : public IServerTrackedDeviceProvider
	{
		SoftKnucklesDevice m_knuckles[NUM_DEVICES];
		SoftKnucklesDebugHandler m_debug_handler[NUM_DEVICES];
		SoftKnucklesSocketNotifier m_notifier;
		BodyTracker *trh = new BodyTracker("RFOOT"), *trm = new BodyTracker("LFOOT"), *trp = new BodyTracker("HIP");

	public:
		SoftKnucklesProvider()
			: m_notifier(this)
		{
			dprintf("Constructing...\n");
		}

		EVRInitError Init(IVRDriverContext* pDriverContext) override
		{
			// NOTE 1: use the driver context.  Sets up a big set of globals
			VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
			dprintf("Initializing...\n");

			/*if constexpr (NUM_DEVICES > 0)
			{
				m_knuckles[0].Init(TrackedControllerRole_LeftHand, component_definitions_left,
				                   NUM_INPUT_COMPONENT_DEFINITIONS,
				                   &m_debug_handler[0]);
			}
			if constexpr (NUM_DEVICES > 1)
			{
				m_knuckles[1].Init(TrackedControllerRole_RightHand, component_definitions_right,
				                   NUM_INPUT_COMPONENT_DEFINITIONS,
				                   &m_debug_handler[1]);
			}*/

			m_notifier.StartListening(listen_address, listen_port);

			std::thread* inittrackers = new std::thread([&]
				{
					while (true) //don't need to wait - pipe's ReadFile already does it
					{
						HANDLE pipeInit = CreateNamedPipe(
							TEXT("\\\\.\\pipe\\TrackersInitPipe"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
							PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE, 1, 1024, 1024, 120 * 1000, nullptr);

						char InitD[1024];
						DWORD Init = DWORD();

						ConnectNamedPipe(pipeInit, nullptr);
						ReadFile(pipeInit, InitD, 1024, &Init, nullptr);
						CloseHandle(pipeInit);

						std::string InitS = InitD;

						if (InitS.find("Initialize Trackers!") != std::string::npos)
						{
							VRServerDriverHost()->TrackedDeviceAdded(trp->get_serial().c_str(),
								TrackedDeviceClass_GenericTracker, trp);
							VRServerDriverHost()->TrackedDeviceAdded(trm->get_serial().c_str(),
								TrackedDeviceClass_GenericTracker, trm);
							VRServerDriverHost()->TrackedDeviceAdded(trh->get_serial().c_str(),
								TrackedDeviceClass_GenericTracker, trh);

							boost::thread m_pipeTR_thread = boost::thread(dlPipeTR);
							m_pipeTR_thread.detach();
							activatedSpawned = true;
							return;
						}
					}
				});

			std::thread* serverstatus = new std::thread([&]
				{
					using clock = std::chrono::steady_clock;
					auto next_frame = clock::now();

					while(true) // We could reboot the application!
					{
						// Add time to wait
						next_frame += std::chrono::milliseconds(1000 / 30);
						
						HANDLE pingPipe = CreateFile(
							TEXT("\\\\.\\pipe\\K2ServerStatusPipe"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
						DWORD Written;

						std::string ServerString = std::to_string(activated ? 1 : 10);

						char ServerData[1024];
						strcpy_s(ServerData, ServerString.c_str());

						WriteFile(pingPipe, ServerData, sizeof(ServerData), &Written, nullptr);
						CloseHandle(pingPipe);

						// Wait until next frame
						std::this_thread::sleep_until(next_frame);
					}
				});

			return VRInitError_None;
		}

		// not virtual: 
		void AddDevices()
		{
			/*for (int i = 0; i < NUM_DEVICES; i++)
			{
				VRServerDriverHost()->TrackedDeviceAdded(
					m_knuckles[i].
					get_serial().c_str(),
					TrackedDeviceClass_Controller,
					&m_knuckles[i]);
			}*/

			/*m_station1->pose = glm::vec3(0, 1, 2);
			m_station1->rots = glm::vec3(glm::radians(-15.0), glm::radians(0.0), glm::radians(0.0));
			VRServerDriverHost()->TrackedDeviceAdded(m_station1->GetSteamVRIdentifier(),
			                                         TrackedDeviceClass_TrackingReference, m_station1);*/

			/*m_pipeM_thread = boost::thread(dlPipeM);
			m_pipeM_thread.detach();
			m_pipeH_thread = boost::thread(dlPipeH);
			m_pipeH_thread.detach();*/
		}

		void Cleanup() override
		{
			dprintf("Cleaning...\n");
			m_notifier.StopListening();
			for (int i = 0; i < NUM_DEVICES; i++)
			{
				m_knuckles[i].Deactivate();
			}
		}

		const char* const * GetInterfaceVersions() override
		{
			return k_InterfaceVersions;
		}

		void RunFrame() override
		{
			static int i;
			if (i++ % 10000 == 0)
			{
				dprintf("Frame: %d\n", i);
			}
		}

		bool ShouldBlockStandbyMode() override
		{
			dprintf("Block standby:\n");
			return true;
		}

		void EnterStandby() override
		{
			dprintf("Entered standby:\n");
			for (int i = 0; i < NUM_DEVICES; i++)
			{
				m_knuckles[i].Deactivate();
			}
		}

		void LeaveStandby() override
		{
			dprintf("Left standby:\n");
			m_knuckles[0].Reactivate();
			m_knuckles[1].Reactivate();
		}
	};

	SoftKnucklesSocketNotifier::SoftKnucklesSocketNotifier(SoftKnucklesProvider* p)
		: m_provider(p)
	{
	}

	void SoftKnucklesSocketNotifier::Notify()
	{
		m_provider->AddDevices();
	}
} // end of namespace 

bool g_bExiting = false;

class CWatchdogDriver_Sample : public IVRWatchdogProvider
{
public:
	CWatchdogDriver_Sample()
	{
		m_pWatchdogThread = nullptr;
	}

	EVRInitError Init(IVRDriverContext* pDriverContext) override;
	void Cleanup() override;

private:
	thread* m_pWatchdogThread;
};

void WatchdogThreadFunction()
{
	while (!g_bExiting)
	{
#if defined( _WINDOWS )
		// on windows send the event when the Y key is pressed.
		//if ((0x01 & GetAsyncKeyState('Y')) != 0)
		{
			// Y key was pressed. 
			VRWatchdogHost()->WatchdogWakeUp(TrackedDeviceClass_HMD);
		}
		this_thread::sleep_for(chrono::microseconds(500));
#else
        // for the other platforms, just send one every five seconds
        this_thread::sleep_for(chrono::seconds(5));
        vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
#endif
	}
}

EVRInitError CWatchdogDriver_Sample::Init(IVRDriverContext* pDriverContext)
{
	VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
	dprintf("Watchdog started...\n");

	// Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
	// be pressed. A real driver should wait for a system button event or something else from the 
	// the hardware that signals that the VR system should start up.
	g_bExiting = false;
	m_pWatchdogThread = new thread(WatchdogThreadFunction);
	if (!m_pWatchdogThread)
	{
		dprintf("N/P cannot create!\n");
		return VRInitError_Driver_Failed;
	}

	soft_knuckles::activated = true; //notify success
	return VRInitError_None;
}

void CWatchdogDriver_Sample::Cleanup()
{
	g_bExiting = true;
	if (m_pWatchdogThread)
	{
		m_pWatchdogThread->join();
		delete m_pWatchdogThread;
		m_pWatchdogThread = nullptr;
	}
}

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	dprintf("HmdDriverFactory %s\n", pInterfaceName);
	soft_knuckles::activated = true; //notify success

	static soft_knuckles::SoftKnucklesProvider s_knuckles_provider; // single instance of the provider
	static CWatchdogDriver_Sample s_watchdogDriverNull; // this is from sample code.
	
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &s_knuckles_provider;
	}
	if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
	{
		return &s_watchdogDriverNull;
	}

	return nullptr;
}
