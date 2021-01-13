//////////////////////////////////////////////////////////////////////////////
// soft_knuckles_device.cpp
//
// See header for description
//
#define _HAS_STD_BYTE 0
#ifdef BOOST_OS_WINDOWS
#define _WIN32_WINNT 0x0501
#if _WIN32_WINNT <= 0x0501
#define BOOST_ASIO_DISABLE_IOCP
#define BOOST_ASIO_ENABLE_CANCELIO
#endif
#endif

#pragma warning( disable : 56812 )
#define BOOST_USE_WINDOWS_H
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include "SimpleSerial.h"
#include <boost/thread.hpp>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include "boost/lexical_cast.hpp"

#include <openvr_driver.h>
#include <thread>
#include <mutex>
#include <string.h>
#include <vector>
#include <string>
#include "dprintf.h"

#include <mutex>          // std::mutex

std::mutex mtx; // mutex for critical section
std::unordered_map<std::string, uint32_t> inputstring2index;
std::unordered_map<std::string, uint32_t> iinputstring2index;
bool dn[15];
int dni[15];

bool mdn[15];
int mdni[15];

#define BUFSIZE 512
#define pipeichi L"\\\\.\\pipe\\LogPipeIchi"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace boost;
#include "soft_knuckles_device.h"
#include "soft_knuckles_config.h"
#include "soft_knuckles_debug_handler.h"

using namespace vr;

std::mutex gpm;
DriverPose_t pipePSM;
glm::vec3 VpipePSM, VpipePSH;
glm::vec3 rtM, rtH;
glm::vec3 VpipePSHOF, VpipePSMOF;
glm::vec3 VpipePSHROF, VpipePSMROF;
glm::vec3 elr, ehr, ell, ehl;
DriverPose_t pipePSH;
glm::vec3 lastp, nowp, deltap;
glm::vec3 lastpm, nowpm, deltapm;
bool sc, ac, scm, acm;

float recHX, recHY, recHZ;
float rCX, rCY, rCZ;
float lCX, lCY, lCZ;
float mplacM[6], mplacMOF[8], mplacMF[5], mplacMROT[4], mplacMBUT[12];
float mplacH[6], mplacHOF[8], mplacHF[5], mplacHROT[4], mplacHBUT[12];

float in1 = 0;
float abend = 100;

float abendt = 100;
float abendi = 100;
float abendm = 100;
float abendr = 100;
float abendl = 100;

float bendt = 0;
float bendi = 0;
float bendm = 0;
float bendr = 0;
float bendl = 0;

struct Fingers
{
	float bendt;
	float bendi;
	float bendm;
	float bendr;
	float bendl;
};

typedef std::unique_ptr<Fingers> FingersPtr;
FingersPtr pfingers(new Fingers),
           lpfingers(new Fingers),
           rpfingers(new Fingers);

float chc;
#define Pi 3.14159265358979323846

#if defined(_WIN32)
#define strtok_r strtok_s
#pragma warning(disable: 4996)
#endif

using namespace interprocess;
namespace pt = property_tree;

VRBoneTransform_t* leftpose;
VRBoneTransform_t* rightpose;

VRBoneTransform_t* leftposeev;
VRBoneTransform_t* rightposeev;

std::string ExtractString(std::string source, std::string start, std::string end)
{
	std::size_t startIndex = source.find(start);
	if (startIndex == std::string::npos)
	{
		return std::string("0");
	}

	startIndex += start.length();
	std::string::size_type endIndex = source.find(end, startIndex);

	return source.substr(startIndex, endIndex - startIndex);
}

int nstrn(string s, const char* whatc)
{
	s = ExtractString(s, whatc, "/");
	stringstream str_strm;
	str_strm << s;
	string temp_str;
	int temp_int;
	while (!str_strm.eof())
	{
		str_strm >> temp_str;
		if (stringstream(temp_str) >> temp_int)
		{
			return temp_int;
		}
		temp_str = "";
	}
}

namespace soft_knuckles
{
	DriverPose_t pos, hposex, mposex;
	DriverPose_t mposet, hposet, pposet;
	TrackedDeviceIndex_t hmdid = 0;

	BaseStation* m_station1 = new BaseStation(static_cast<int>(1));

	VRBoneTransform_t lbone_t[31U];
	VRBoneTransform_t rbone_t[31U];

	SoftKnucklesDebugHandler::SoftKnucklesDebugHandler()
		: m_device(nullptr)
	{
	}

	void SoftKnucklesDebugHandler::Init(SoftKnucklesDevice* d)
	{
		m_device = d;
	}

	double t0, t1, t2, t3, t4, t5;
	double yaw, pitch, roll;
	int nexti;

	static const int NUM_BONES = 31;

	static void tokenize(const char* const_input, const char* delim, vector<string>* ret)
	{
		char input[1024];
		strcpy(input, const_input);
		ret->resize(0);
		char* context;
		char* token = strtok_r(input, delim, &context);
		while (token)
		{
			ret->push_back(token);
			token = strtok_r(nullptr, delim, &context);
		}
	}

	void ConvertQuaternion(const glm::quat& f_glmQuat, HmdQuaternionf_t& f_vrQuat)
	{
		f_vrQuat.x = f_glmQuat.x;
		f_vrQuat.y = f_glmQuat.y;
		f_vrQuat.z = f_glmQuat.z;
		f_vrQuat.w = f_glmQuat.w;
	}

	void ConvertQuaternion(const HmdQuaternionf_t& f_vrQuat, glm::quat& f_glmQuat)
	{
		f_glmQuat.x = f_vrQuat.x;
		f_glmQuat.y = f_vrQuat.y;
		f_glmQuat.z = f_vrQuat.z;
		f_glmQuat.w = f_vrQuat.w;
	}

	void ConvertVector3(const HmdVector4_t& f_vrVec, glm::vec3& f_glmVec)
	{
		for (size_t i = 0U; i < 3U; i++) f_glmVec[i] = f_vrVec.v[i];
	}

	void ConvertVector3(const glm::vec3& f_glmVec, HmdVector4_t& f_vrVec)
	{
		for (size_t i = 0U; i < 3U; i++) f_vrVec.v[i] = f_glmVec[i];
	}

	VRBoneTransform_t lhp[NUM_BONES];

	VRBoneTransform_t* SoftKnucklesDevice::GetLfPose(float t, float i, float m, float r, float l)
	{
		transformleftthumb(t);
		transformleftindex(i);
		transformleftmiddle(m);
		transformleftring(r);
		transformleftpinky(l);

		for (int ix = 0; ix == 30;)
		{
			lhp[ix].position = {
				hidari[ix][0], hidari[ix][1], hidari[ix][2], hidari[ix][3]
			};

			lhp[ix].orientation = {
				hidari[ix + 1][0], hidari[ix + 1][1], hidari[ix + 1][2], hidari[ix + 1][3]
			};

			ix += 2;
		}

		return lhp;
	}

	void setlthumbend(float bend)
	{
		lpfingers->bendt = bend;
	}

	void setlindexbend(float bend)
	{
		lpfingers->bendi = bend;
	}

	void setlmiddlebend(float bend)
	{
		lpfingers->bendm = bend;
	}

	void setlringbend(float bend)
	{
		lpfingers->bendr = bend;
	}

	void setllittlebend(float bend)
	{
		lpfingers->bendl = bend;
	}

	void setrthumbend(float bend)
	{
		rpfingers->bendt = bend;
	}

	void setrindexbend(float bend)
	{
		rpfingers->bendi = bend;
	}

	void setrmiddlebend(float bend)
	{
		rpfingers->bendm = bend;
	}

	void setrringbend(float bend)
	{
		rpfingers->bendr = bend;
	}

	void setrlittlebend(float bend)
	{
		rpfingers->bendl = bend;
	}

	float getlthumbend()
	{
		return (lpfingers->bendt);
	}

	float getlindexbend()
	{
		return (lpfingers->bendi);
	}

	float getlmiddlebend()
	{
		return (lpfingers->bendm);
	}

	float getlringbend()
	{
		return (lpfingers->bendr);
	}

	float getllittlebend()
	{
		return (lpfingers->bendl);
	}

	float getrthumbend()
	{
		return (rpfingers->bendt);
	}

	float getrindexbend()
	{
		return (rpfingers->bendi);
	}

	float getrmiddlebend()
	{
		return (rpfingers->bendm);
	}

	float getrringbend()
	{
		return (rpfingers->bendr);
	}

	float getrlittlebend()
	{
		return (rpfingers->bendl);
	}

	void dlPipeTR()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif
		using namespace Eigen;

		while (true)
		{
			HANDLE pipeOf = CreateNamedPipe(
				TEXT("\\\\.\\pipe\\LogPipeTracker"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
				PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE, 1, 1024, 1024, 120 * 1000, nullptr);

			char OfD[1024];
			DWORD Of = DWORD();

			ConnectNamedPipe(pipeOf, nullptr);
			ReadFile(pipeOf, OfD, 1024, &Of, nullptr);

			//DisconnectNamedPipe(pipeOf);
			CloseHandle(pipeOf);
			std::string OfS = OfD;

			hposet.vecPosition[0] = static_cast<float>(nstrn(OfS, "HX")) / static_cast<float>(10000);
			hposet.vecPosition[1] = static_cast<float>(nstrn(OfS, "HY")) / static_cast<float>(10000);
			hposet.vecPosition[2] = static_cast<float>(nstrn(OfS, "HZ")) / static_cast<float>(10000);
			mposet.vecPosition[0] = static_cast<float>(nstrn(OfS, "MX")) / static_cast<float>(10000);
			mposet.vecPosition[1] = static_cast<float>(nstrn(OfS, "MY")) / static_cast<float>(10000);
			mposet.vecPosition[2] = static_cast<float>(nstrn(OfS, "MZ")) / static_cast<float>(10000);
			pposet.vecPosition[0] = static_cast<float>(nstrn(OfS, "PX")) / static_cast<float>(10000);
			pposet.vecPosition[1] = static_cast<float>(nstrn(OfS, "PY")) / static_cast<float>(10000);
			pposet.vecPosition[2] = static_cast<float>(nstrn(OfS, "PZ")) / static_cast<float>(10000);

			float vhrotw = static_cast<float>(nstrn(OfS, "HRW")) / static_cast<float>(10000);
			float vhrotx = static_cast<float>(nstrn(OfS, "HRX")) / static_cast<float>(10000);
			float vhroty = static_cast<float>(nstrn(OfS, "HRY")) / static_cast<float>(10000);
			float vhrotz = static_cast<float>(nstrn(OfS, "HRZ")) / static_cast<float>(10000);

			float vmrotw = static_cast<float>(nstrn(OfS, "MRW")) / static_cast<float>(10000);
			float vmrotx = static_cast<float>(nstrn(OfS, "MRX")) / static_cast<float>(10000);
			float vmroty = static_cast<float>(nstrn(OfS, "MRY")) / static_cast<float>(10000);
			float vmrotz = static_cast<float>(nstrn(OfS, "MRZ")) / static_cast<float>(10000);

			float vprotw = static_cast<float>(nstrn(OfS, "PRW")) / static_cast<float>(10000);
			float vprotx = static_cast<float>(nstrn(OfS, "PRX")) / static_cast<float>(10000);
			float vproty = static_cast<float>(nstrn(OfS, "PRY")) / static_cast<float>(10000);
			float vprotz = static_cast<float>(nstrn(OfS, "PRZ")) / static_cast<float>(10000);

			std::string en = ExtractString(OfS, "ENABLED", "/");

			if (en == "0")
			{
				hposet.poseIsValid = false;
				hposet.deviceIsConnected = false;
				mposet.poseIsValid = false;
				mposet.deviceIsConnected = false;
				pposet.poseIsValid = false;
				pposet.deviceIsConnected = false;
			}
			else if (en == "1")
			{
				hposet.poseIsValid = true;
				hposet.deviceIsConnected = true;
				mposet.poseIsValid = true;
				mposet.deviceIsConnected = true;
				pposet.poseIsValid = true;
				pposet.deviceIsConnected = true;
			}

			hposet.qRotation.w = vhrotw;
			hposet.qRotation.x = vhrotx;
			hposet.qRotation.y = vhroty;
			hposet.qRotation.z = vhrotz;

			mposet.qRotation.w = vmrotw;
			mposet.qRotation.x = vmrotx;
			mposet.qRotation.y = vmroty;
			mposet.qRotation.z = vmrotz;

			pposet.qRotation.w = vprotw;
			pposet.qRotation.x = vprotx;
			pposet.qRotation.y = vproty;
			pposet.qRotation.z = vprotz;

			hposet.vecPosition[0] += static_cast<float>(nstrn(OfS, "HOX")) / static_cast<float>(10000);
			hposet.vecPosition[1] += static_cast<float>(nstrn(OfS, "HOY")) / static_cast<float>(10000);
			hposet.vecPosition[2] += static_cast<float>(nstrn(OfS, "HOZ")) / static_cast<float>(10000);
			mposet.vecPosition[0] += static_cast<float>(nstrn(OfS, "MOX")) / static_cast<float>(10000);
			mposet.vecPosition[1] += static_cast<float>(nstrn(OfS, "MOY")) / static_cast<float>(10000);
			mposet.vecPosition[2] += static_cast<float>(nstrn(OfS, "MOZ")) / static_cast<float>(10000);
			pposet.vecPosition[0] += static_cast<float>(nstrn(OfS, "POX")) / static_cast<float>(10000);
			pposet.vecPosition[1] += static_cast<float>(nstrn(OfS, "POY")) / static_cast<float>(10000);
			pposet.vecPosition[2] += static_cast<float>(nstrn(OfS, "POZ")) / static_cast<float>(10000);

			std::this_thread::sleep_for(std::chrono::milliseconds(3));
		}
	}

	void dlPipeM()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeIchi"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
		                               PIPE_WAIT, 1, 1024, 1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataM: %s\n", ichiS.c_str());

			mplacM[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacM[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacM[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);

			mplacM[3] = static_cast<float>(nstrn(ichiS, "EX")) / static_cast<float>(10000);
			mplacM[4] = static_cast<float>(nstrn(ichiS, "EY")) / static_cast<float>(10000);
			mplacM[5] = static_cast<float>(nstrn(ichiS, "EZ")) / static_cast<float>(10000);
			bool isV = nstrn(ichiS, "DISABLE") / static_cast<float>(10000);

			if (isV)
			{
				pipePSM.deviceIsConnected = false;
				pipePSM.poseIsValid = false;
			}
			else
			{
				pipePSM.deviceIsConnected = true;
				pipePSM.poseIsValid = true;
			}

			pipePSM.vecPosition[0] = mplacM[0];
			pipePSM.vecPosition[1] = mplacM[1];
			pipePSM.vecPosition[2] = mplacM[2];

			pipePSM.vecPosition[0] += VpipePSMOF.x;
			pipePSM.vecPosition[1] += VpipePSMOF.y;
			pipePSM.vecPosition[2] += VpipePSMOF.z;

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeMOF()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeIchiOf"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
		                               PIPE_WAIT, 1, 1024, 1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				/*while (ReadFile(hPipe, &ichiD, sizeof(ichiD), 0, NULL) != FALSE) {
				}*/
				ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr);
			}
			std::string ichiS(ichiD);
			//dprintf("DataM: %s\n", ichiS.c_str());

			mplacMOF[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacMOF[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacMOF[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);
			mplacMOF[3] = static_cast<float>(nstrn(ichiS, "R")) / static_cast<float>(10000);

			mplacMOF[4] = static_cast<float>(nstrn(ichiS, "RX")) / static_cast<float>(10000);
			mplacMOF[5] = static_cast<float>(nstrn(ichiS, "RY")) / static_cast<float>(10000);
			mplacMOF[6] = static_cast<float>(nstrn(ichiS, "RZ")) / static_cast<float>(10000);

			VpipePSMOF.x = mplacMOF[0];
			VpipePSMOF.y = mplacMOF[1];
			VpipePSMOF.z = mplacMOF[2];

			VpipePSMROF.x = mplacMOF[4];
			VpipePSMROF.y = mplacMOF[5];
			VpipePSMROF.z = mplacMOF[6];

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeMF()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(
			TEXT("\\\\.\\pipe\\LogPipeIchiFinger"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_WAIT, 1, 1024,
			1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataM: %s\n", ichiS.c_str());

			mplacMF[0] = static_cast<float>(nstrn(ichiS, "T")) / static_cast<float>(10000);
			mplacMF[1] = static_cast<float>(nstrn(ichiS, "I")) / static_cast<float>(10000);
			mplacMF[2] = static_cast<float>(nstrn(ichiS, "M")) / static_cast<float>(10000);
			mplacMF[3] = static_cast<float>(nstrn(ichiS, "P")) / static_cast<float>(10000);
			mplacMF[4] = static_cast<float>(nstrn(ichiS, "L")) / static_cast<float>(10000);

			setrthumbend(mplacMF[0]);
			setrindexbend(mplacMF[1]);
			setrmiddlebend(mplacMF[2]);
			setrringbend(mplacMF[3]);
			setrlittlebend(mplacMF[4]);

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeMROT()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(
			TEXT("\\\\.\\pipe\\LogPipeIchiRot"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_WAIT, 1, 1024, 1024,
			120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataM: %s\n", ichiS.c_str());

			mplacMROT[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacMROT[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacMROT[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);
			mplacMROT[3] = static_cast<float>(nstrn(ichiS, "W")) / static_cast<float>(10000);

			if (mplacMROT[3] != 0.f && mplacMROT[0] != 0.f && mplacMROT[2] != 0.f)
			{
				pipePSM.qRotation.w = mplacMROT[3];
				pipePSM.qRotation.x = mplacMROT[0];
				pipePSM.qRotation.y = mplacMROT[1];
				pipePSM.qRotation.z = mplacMROT[2];
			}
			DisconnectNamedPipe(hPipe);
		}
	}

	void SoftKnucklesDevice::dlPipeMBUT(SoftKnucklesDevice* pthis)
	{
		while (true)
		{
			if (pthis->m_role == TrackedControllerRole_RightHand)
			{
				for (int component = 0; component <= 9; component++)
				{
					string inpath, val;

					switch (component)
					{
					case 0:
						inpath = "/input/thumbstick/x";
						val = boost::lexical_cast<std::string>(mplacMBUT[0]);
						break;

					case 1:
						inpath = "/input/thumbstick/y";
						val = boost::lexical_cast<std::string>(mplacMBUT[1]);
						break;

					case 2:
						inpath = "/input/thumbstick/touch";
						if (mplacMBUT[0] != 0.f || mplacMBUT[1] != 0.f)
						{
							val = "1";
						}
						else
						{
							val = "0";
						}
						break;

					case 3:
						inpath = "/input/system/click";
						val = boost::lexical_cast<std::string>(mplacMBUT[2]);
						break;

					case 4:
						inpath = "/input/trigger/value";
						val = boost::lexical_cast<std::string>(mplacMBUT[3]);
						break;

					case 5:
						inpath = "/input/trigger/touch";
						mplacMBUT[3] != 0.f ? val = "1" : val = "0";
						break;

					case 6:
						inpath = "/input/a/click";
						val = boost::lexical_cast<std::string>(mplacMBUT[4]);
						break;

					case 7:
						inpath = "/input/b/click";
						val = boost::lexical_cast<std::string>(mplacMBUT[5]);
						break;

					case 8:
						inpath = "/input/grip/click";
						val = boost::lexical_cast<std::string>(mplacMBUT[6]);
						break;

					case 9:
						inpath = "/input/grip/touch";
						val = boost::lexical_cast<std::string>(mplacMBUT[6]);
						break;
					}

					if (inputstring2index.size() == 0)
					{
						inputstring2index.reserve(pthis->m_num_component_definitions);
						for (uint32_t i = 0; i < pthis->m_num_component_definitions; i++)
						{
							inputstring2index[pthis->m_component_definitions[i].full_path] = i;
						}
					}

					string input_state_path = inpath;
					auto iter = inputstring2index.find(input_state_path);
					if (iter != inputstring2index.end())
					{
						uint32_t index = (*iter).second;
						VRInputComponentHandle_t component_handle = pthis->m_component_handles[index];
						ComponentType component_type = pthis->m_component_definitions[index].component_type;
						if (component_type == CT_BOOLEAN)
						{
							bool new_value = (val == "1");
							EVRInputError err = VRDriverInput()->UpdateBooleanComponent(component_handle, new_value, 0);
							if (err != VRInputError_None)
							{
								dprintf("error %d\n", err);
							}
						}
						else if (component_type == CT_SCALAR)
						{
							float new_value = static_cast<float>(atof(val.c_str()));
							EVRInputError err = VRDriverInput()->UpdateScalarComponent(component_handle, new_value, 0);
							if (err != VRInputError_None)
							{
								dprintf("error %d\n", err);
							}
						}
					}
				}
			}
		}
	}

	void mbutpipe()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif
		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(
			TEXT("\\\\.\\pipe\\LogPipeIchiButton"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_WAIT, 1, 1024,
			1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);

			mplacMBUT[0] = static_cast<float>(nstrn(ichiS, "JX")) / static_cast<float>(10000);
			mplacMBUT[1] = static_cast<float>(nstrn(ichiS, "JY")) / static_cast<float>(10000);
			mplacMBUT[2] = static_cast<float>(nstrn(ichiS, "SY")) / static_cast<float>(10000);
			mplacMBUT[3] = static_cast<float>(nstrn(ichiS, "TR")) / static_cast<float>(10000);
			mplacMBUT[4] = static_cast<float>(nstrn(ichiS, "AB")) / static_cast<float>(10000);
			mplacMBUT[5] = static_cast<float>(nstrn(ichiS, "BB")) / static_cast<float>(10000);
			mplacMBUT[6] = static_cast<float>(nstrn(ichiS, "GR")) / static_cast<float>(10000);

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeH()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeNi"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
		                               PIPE_WAIT, 1, 1024, 1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataH: %s\n", ichiS.c_str());

			mplacH[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacH[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacH[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);

			mplacH[3] = static_cast<float>(nstrn(ichiS, "EX")) / static_cast<float>(10000);
			mplacH[4] = static_cast<float>(nstrn(ichiS, "EY")) / static_cast<float>(10000);
			mplacH[5] = static_cast<float>(nstrn(ichiS, "EZ")) / static_cast<float>(10000);
			bool isV = nstrn(ichiS, "DISABLE") / static_cast<float>(10000);

			if (isV)
			{
				pipePSH.deviceIsConnected = false;
				pipePSH.poseIsValid = false;
			}
			else
			{
				pipePSH.deviceIsConnected = true;
				pipePSH.poseIsValid = true;
			}

			pipePSH.vecPosition[0] = mplacH[0];
			pipePSH.vecPosition[1] = mplacH[1];
			pipePSH.vecPosition[2] = mplacH[2];

			pipePSH.vecPosition[0] += VpipePSHOF.x;
			pipePSH.vecPosition[1] += VpipePSHOF.y;
			pipePSH.vecPosition[2] += VpipePSHOF.z;

			m_station1->Update();

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeHOF()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeNiOf"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
		                               PIPE_WAIT, 1, 1024, 1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				/*while (ReadFile(hPipe, &ichiD, sizeof(ichiD), 0, NULL) != FALSE) {
					dprintf("read!?!\n");
				}*/
				ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr);
			}
			std::string ichiS(ichiD);
			//dprintf("DataHiOF: %s\n", ichiS.c_str());

			mplacHOF[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacHOF[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacHOF[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);
			mplacHOF[3] = static_cast<float>(nstrn(ichiS, "R")) / static_cast<float>(10000);

			mplacHOF[4] = static_cast<float>(nstrn(ichiS, "RX")) / static_cast<float>(10000);
			mplacHOF[5] = static_cast<float>(nstrn(ichiS, "RY")) / static_cast<float>(10000);
			mplacHOF[6] = static_cast<float>(nstrn(ichiS, "RZ")) / static_cast<float>(10000);

			VpipePSHOF.x = mplacHOF[0];
			VpipePSHOF.y = mplacHOF[1];
			VpipePSHOF.z = mplacHOF[2];

			VpipePSHROF.x = mplacHOF[4];
			VpipePSHROF.y = mplacHOF[5];
			VpipePSHROF.z = mplacHOF[6];

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeHF()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(
			TEXT("\\\\.\\pipe\\LogPipeNiFinger"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_WAIT, 1, 1024, 1024,
			120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataH: %s\n", ichiS.c_str());

			mplacHF[0] = static_cast<float>(nstrn(ichiS, "T")) / static_cast<float>(10000);
			mplacHF[1] = static_cast<float>(nstrn(ichiS, "I")) / static_cast<float>(10000);
			mplacHF[2] = static_cast<float>(nstrn(ichiS, "M")) / static_cast<float>(10000);
			mplacHF[3] = static_cast<float>(nstrn(ichiS, "P")) / static_cast<float>(10000);
			mplacHF[4] = static_cast<float>(nstrn(ichiS, "L")) / static_cast<float>(10000);

			setlthumbend(mplacHF[0]);
			setlindexbend(mplacHF[1]);
			setlmiddlebend(mplacHF[2]);
			setlringbend(mplacHF[3]);
			setllittlebend(mplacHF[4]);

			DisconnectNamedPipe(hPipe);
		}
	}

	void dlPipeHROT()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif

		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeNiRot"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
		                               PIPE_WAIT, 1, 1024, 1024, 120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);
			//dprintf("DataH: %s\n", ichiS.c_str());

			mplacHROT[0] = static_cast<float>(nstrn(ichiS, "X")) / static_cast<float>(10000);
			mplacHROT[1] = static_cast<float>(nstrn(ichiS, "Y")) / static_cast<float>(10000);
			mplacHROT[2] = static_cast<float>(nstrn(ichiS, "Z")) / static_cast<float>(10000);
			mplacHROT[3] = static_cast<float>(nstrn(ichiS, "W")) / static_cast<float>(10000);

			if (mplacHROT[3] != 0.f && mplacHROT[0] != 0.f && mplacHROT[2] != 0.f)
			{
				pipePSH.qRotation.w = mplacHROT[3];
				pipePSH.qRotation.x = mplacHROT[0];
				pipePSH.qRotation.y = mplacHROT[1];
				pipePSH.qRotation.z = mplacHROT[2];
			}
			DisconnectNamedPipe(hPipe);
		}
	}

	void SoftKnucklesDevice::dlPipeHBUT(SoftKnucklesDevice* pthis)
	{
		while (true)
		{
			if (pthis->m_role == TrackedControllerRole_LeftHand)
			{
				for (int component = 0; component <= 9; component++)
				{
					string inpath, val;

					switch (component)
					{
					case 0:
						inpath = "/input/thumbstick/x";
						val = boost::lexical_cast<std::string>(mplacHBUT[0]);
						break;

					case 1:
						inpath = "/input/thumbstick/y";
						val = boost::lexical_cast<std::string>(mplacHBUT[1]);
						break;

					case 2:
						inpath = "/input/thumbstick/touch";
						if (mplacHBUT[0] != 0.f || mplacHBUT[1] != 0.f)
						{
							val = "1";
						}
						else
						{
							val = "0";
						}
						break;

					case 3:
						inpath = "/input/system/click";
						val = boost::lexical_cast<std::string>(mplacHBUT[2]);
						break;

					case 4:
						inpath = "/input/trigger/value";
						val = boost::lexical_cast<std::string>(mplacHBUT[3]);
						break;

					case 5:
						inpath = "/input/trigger/touch";
						mplacHBUT[3] != 0.f ? val = "1" : val = "0";
						break;

					case 6:
						inpath = "/input/a/click";
						val = boost::lexical_cast<std::string>(mplacHBUT[4]);
						break;

					case 7:
						inpath = "/input/b/click";
						val = boost::lexical_cast<std::string>(mplacHBUT[5]);
						break;

					case 8:
						inpath = "/input/grip/click";
						val = boost::lexical_cast<std::string>(mplacHBUT[6]);
						break;

					case 9:
						inpath = "/input/grip/touch";
						val = boost::lexical_cast<std::string>(mplacHBUT[6]);
						break;
					}

					if (inputstring2index.size() == 0)
					{
						inputstring2index.reserve(pthis->m_num_component_definitions);
						for (uint32_t i = 0; i < pthis->m_num_component_definitions; i++)
						{
							inputstring2index[pthis->m_component_definitions[i].full_path] = i;
						}
					}

					string input_state_path = inpath;
					auto iter = inputstring2index.find(input_state_path);
					if (iter != inputstring2index.end())
					{
						uint32_t index = (*iter).second;
						VRInputComponentHandle_t component_handle = pthis->m_component_handles[index];
						ComponentType component_type = pthis->m_component_definitions[index].component_type;
						if (component_type == CT_BOOLEAN)
						{
							bool new_value = (val == "1");
							EVRInputError err = VRDriverInput()->UpdateBooleanComponent(component_handle, new_value, 0);
							if (err != VRInputError_None)
							{
								dprintf("error %d\n", err);
							}
						}
						else if (component_type == CT_SCALAR)
						{
							float new_value = static_cast<float>(atof(val.c_str()));
							EVRInputError err = VRDriverInput()->UpdateScalarComponent(component_handle, new_value, 0);
							if (err != VRInputError_None)
							{
								dprintf("error %d\n", err);
							}
						}
					}
				}
			}
		}
	}

	void hbutpipe()
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pipe_thread");
#endif
		DWORD dwRead;
		char ichiD[1024];

		HANDLE hPipe = CreateNamedPipe(
			TEXT("\\\\.\\pipe\\LogPipeNiButton"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_WAIT, 1, 1024, 1024,
			120 * 1000, nullptr);
		while (hPipe != INVALID_HANDLE_VALUE)
		{
			if (ConnectNamedPipe(hPipe, nullptr) != FALSE)
			{
				while (ReadFile(hPipe, &ichiD, sizeof(ichiD), nullptr, nullptr) != FALSE)
				{
				}
			}
			std::string ichiS(ichiD);

			mplacHBUT[0] = static_cast<float>(nstrn(ichiS, "JX")) / static_cast<float>(10000);
			mplacHBUT[1] = static_cast<float>(nstrn(ichiS, "JY")) / static_cast<float>(10000);
			mplacHBUT[2] = static_cast<float>(nstrn(ichiS, "SY")) / static_cast<float>(10000);
			mplacHBUT[3] = static_cast<float>(nstrn(ichiS, "TR")) / static_cast<float>(10000);
			mplacHBUT[4] = static_cast<float>(nstrn(ichiS, "AB")) / static_cast<float>(10000);
			mplacHBUT[5] = static_cast<float>(nstrn(ichiS, "BB")) / static_cast<float>(10000);
			mplacHBUT[6] = static_cast<float>(nstrn(ichiS, "GR")) / static_cast<float>(10000);

			DisconnectNamedPipe(hPipe);
		}
	}

	static void set_response(const char* response_source, char* response, uint32_t response_buffer_size)
	{
		if (response_buffer_size > 1)
		{
			size_t strlen_src = strlen(response_source);
			if (strlen_src + 1 > response_buffer_size)
			{
				memcpy(response, response_source, response_buffer_size - 1);
				response[response_buffer_size - 1] = 0;
			}
			else
			{
				memcpy(response, response_source, strlen_src + 1);
			}
		}
	}

	void SoftKnucklesDebugHandler::InitializeLookupTable()
	{
		m_inputstring2index.reserve(m_device->m_num_component_definitions);
		for (uint32_t i = 0; i < m_device->m_num_component_definitions; i++)
		{
			m_inputstring2index[m_device->m_component_definitions[i].full_path] = i;
		}
	}

	void SoftKnucklesDebugHandler::DebugRequest(const char* request, char* response, uint32_t response_buffer_size)
	{
		if (m_inputstring2index.size() == 0)
			InitializeLookupTable();

		dprintf("device_id %d received request: %s\n", m_device->m_id, request);

		vector<string> tokens;
		tokenize(request, " \r\t\n,", &tokens);
		bool success = false;
		if (tokens.size() > 1) // need at least two params
		{
			if (tokens[0] == "fpose")
			{
				bendt = atof(tokens[1].c_str());
				bendi = atof(tokens[2].c_str());
				bendm = atof(tokens[3].c_str());
				bendr = atof(tokens[4].c_str());
				bendl = atof(tokens[5].c_str());

				//bendin = atof(tokens[2].c_str());;
				if (m_device->m_role == TrackedControllerRole_LeftHand)
				{
					setlthumbend(bendt);

					setlindexbend(bendi);

					setlmiddlebend(bendm);

					setlringbend(bendr);

					setllittlebend(bendl);

					//		static VRBoneTransform_t left_hand_pose[NUM_BONES] = {
					//{ { hidari[0][0],  hidari[0][1],  hidari[0][2],  hidari[0][3]}, { hidari[1][0],  hidari[1][1],  hidari[1][2],  hidari[1][3]} }, //Root
					//{ { hidari[2][0],  hidari[2][1],  hidari[2][2],  hidari[2][3]}, { hidari[3][0],  hidari[3][1],  hidari[3][2],  hidari[3][3]} }, //Wrist
					//{ { hidari[4][0],  hidari[4][1],  hidari[4][2],  hidari[4][3]}, { hidari[5][0],  hidari[5][1],  hidari[5][2],  hidari[5][3]} }, //Thumb0
					//{ { hidari[6][0],  hidari[6][1],  hidari[6][2],  hidari[6][3]}, { hidari[7][0],  hidari[7][1],  hidari[7][2],  hidari[7][3]} }, //Thumb1
					//{ { hidari[8][0],  hidari[8][1],  hidari[8][2],  hidari[8][3]}, { hidari[9][0],  hidari[9][1],  hidari[9][2],  hidari[9][3]} }, //Thumb2
					//{ { hidari[10][0], hidari[10][1],  hidari[10][2],  hidari[10][3]}, { hidari[11][0],  hidari[11][1],  hidari[11][2],  hidari[11][3]} }, //Thumb3
					//{ { hidari[12][0], hidari[12][1],  hidari[12][2],  hidari[12][3]}, { hidari[13][0],  hidari[13][1],  hidari[13][2],  hidari[13][3]} }, //Index0
					//{ { hidari[14][0], hidari[14][1],  hidari[14][2],  hidari[14][3]}, { hidari[15][0],  hidari[15][1],  hidari[15][2],  hidari[15][3]} }, //Index1
					//{ { hidari[16][0], hidari[16][1],  hidari[16][2],  hidari[16][3]}, { hidari[17][0],  hidari[17][1],  hidari[17][2],  hidari[17][3]} }, //Index2
					//{ { hidari[18][0], hidari[18][1],  hidari[18][2],  hidari[18][3]}, { hidari[19][0],  hidari[19][1],  hidari[19][2],  hidari[19][3]} }, //Index3
					//{ { hidari[20][0], hidari[20][1],  hidari[20][2],  hidari[20][3]}, { hidari[21][0],  hidari[21][1],  hidari[21][2],  hidari[21][3]} }, //Index4
					//{ { hidari[22][0], hidari[22][1],  hidari[22][2],  hidari[22][3]}, { hidari[23][0],  hidari[23][1],  hidari[23][2],  hidari[23][3]} }, //Middle0
					//{ { hidari[24][0], hidari[24][1],  hidari[24][2],  hidari[24][3]}, { hidari[25][0],  hidari[25][1],  hidari[25][2],  hidari[25][3]} }, //Middle1
					//{ { hidari[26][0], hidari[26][1],  hidari[26][2],  hidari[26][3]}, { hidari[27][0],  hidari[27][1],  hidari[27][2],  hidari[27][3]} }, //Middle2
					//{ { hidari[28][0], hidari[28][1],  hidari[28][2],  hidari[28][3]}, { hidari[29][0],  hidari[29][1],  hidari[29][2],  hidari[29][3]} }, //Middle3
					//{ { hidari[30][0], hidari[30][1],  hidari[30][2],  hidari[30][3]}, { hidari[31][0],  hidari[31][1],  hidari[31][2],  hidari[31][3]} }, //Middle4
					//{ { hidari[32][0], hidari[32][1],  hidari[32][2],  hidari[32][3]}, { hidari[33][0],  hidari[33][1],  hidari[33][2],  hidari[33][3]} }, //Ring0
					//{ { hidari[34][0], hidari[34][1],  hidari[34][2],  hidari[34][3]}, { hidari[35][0],  hidari[35][1],  hidari[35][2],  hidari[35][3]} }, //Ring1
					//{ { hidari[36][0], hidari[36][1],  hidari[36][2],  hidari[36][3]}, { hidari[37][0],  hidari[37][1],  hidari[37][2],  hidari[37][3]} }, //Ring2
					//{ { hidari[38][0], hidari[38][1],  hidari[38][2],  hidari[38][3]}, { hidari[39][0],  hidari[39][1],  hidari[39][2],  hidari[39][3]} }, //Ring3
					//{ { hidari[40][0], hidari[40][1],  hidari[40][2],  hidari[40][3]}, { hidari[41][0],  hidari[41][1],  hidari[41][2],  hidari[41][3]} }, //Ring4
					//{ { hidari[42][0], hidari[42][1],  hidari[42][2],  hidari[42][3]}, { hidari[43][0],  hidari[43][1],  hidari[43][2],  hidari[43][3]} }, //Pinky0
					//{ { hidari[44][0], hidari[44][1],  hidari[44][2],  hidari[44][3]}, { hidari[45][0],  hidari[45][1],  hidari[45][2],  hidari[45][3]} }, //Pinky1
					//{ { hidari[46][0], hidari[46][1],  hidari[46][2],  hidari[46][3]}, { hidari[47][0],  hidari[47][1],  hidari[47][2],  hidari[47][3]} }, //Pinky2
					//{ { hidari[48][0], hidari[48][1],  hidari[48][2],  hidari[48][3]}, { hidari[49][0],  hidari[49][1],  hidari[49][2],  hidari[49][3]} }, //Pinky3
					//{ { hidari[50][0], hidari[50][1],  hidari[50][2],  hidari[50][3]}, { hidari[51][0],  hidari[51][1],  hidari[51][2],  hidari[51][3]} }, //Pinky4
					//{ { hidari[52][0], hidari[52][1],  hidari[52][2],  hidari[52][3]}, { hidari[53][0],  hidari[53][1],  hidari[53][2],  hidari[53][3]} }, //AuxThumb
					//{ { hidari[54][0], hidari[54][1],  hidari[54][2],  hidari[54][3]}, { hidari[55][0],  hidari[55][1],  hidari[55][2],  hidari[55][3]} }, //AuxIndex
					//{ { hidari[56][0], hidari[56][1],  hidari[56][2],  hidari[56][3]}, { hidari[57][0],  hidari[57][1],  hidari[57][2],  hidari[57][3]} }, //AuxMiddle
					//{ { hidari[58][0], hidari[58][1],  hidari[58][2],  hidari[58][3]}, { hidari[59][0],  hidari[59][1],  hidari[59][2],  hidari[59][3]} }, //AuxRing
					//{ { hidari[60][0], hidari[60][1],  hidari[60][2],  hidari[60][3]}, { hidari[61][0],  hidari[61][1],  hidari[61][2],  hidari[61][3]} }, //AuxPinky
					//		};
					//		leftpose = left_hand_pose;
					//		leftposeev = left_hand_pose;
				}
				if (m_device->m_role == TrackedControllerRole_RightHand)
				{
					setrthumbend(bendt);

					setrindexbend(bendi);

					setrmiddlebend(bendm);

					setrringbend(bendr);

					setrlittlebend(bendl);

					//		static VRBoneTransform_t right_hand_pose[NUM_BONES] = {
					//{ { migi[0][0],  migi[0][1],  migi[0][2],  migi[0][3]}, { migi[1][0],  migi[1][1],  migi[1][2],  migi[1][3]} }, //Root
					//{ { migi[2][0],  migi[2][1],  migi[2][2],  migi[2][3]}, { migi[3][0],  migi[3][1],  migi[3][2],  migi[3][3]} }, //Wrist
					//{ { migi[4][0],  migi[4][1],  migi[4][2],  migi[4][3]}, { migi[5][0],  migi[5][1],  migi[5][2],  migi[5][3]} }, //Thumb0
					//{ { migi[6][0],  migi[6][1],  migi[6][2],  migi[6][3]}, { migi[7][0],  migi[7][1],  migi[7][2],  migi[7][3]} }, //Thumb1
					//{ { migi[8][0],  migi[8][1],  migi[8][2],  migi[8][3]}, { migi[9][0],  migi[9][1],  migi[9][2],  migi[9][3]} }, //Thumb2
					//{ { migi[10][0], migi[10][1],  migi[10][2],  migi[10][3]}, { migi[11][0],  migi[11][1],  migi[11][2],  migi[11][3]} }, //Thumb3
					//{ { migi[12][0], migi[12][1],  migi[12][2],  migi[12][3]}, { migi[13][0],  migi[13][1],  migi[13][2],  migi[13][3]} }, //Index0
					//{ { migi[14][0], migi[14][1],  migi[14][2],  migi[14][3]}, { migi[15][0],  migi[15][1],  migi[15][2],  migi[15][3]} }, //Index1
					//{ { migi[16][0], migi[16][1],  migi[16][2],  migi[16][3]}, { migi[17][0],  migi[17][1],  migi[17][2],  migi[17][3]} }, //Index2
					//{ { migi[18][0], migi[18][1],  migi[18][2],  migi[18][3]}, { migi[19][0],  migi[19][1],  migi[19][2],  migi[19][3]} }, //Index3
					//{ { migi[20][0], migi[20][1],  migi[20][2],  migi[20][3]}, { migi[21][0],  migi[21][1],  migi[21][2],  migi[21][3]} }, //Index4
					//{ { migi[22][0], migi[22][1],  migi[22][2],  migi[22][3]}, { migi[23][0],  migi[23][1],  migi[23][2],  migi[23][3]} }, //Middle0
					//{ { migi[24][0], migi[24][1],  migi[24][2],  migi[24][3]}, { migi[25][0],  migi[25][1],  migi[25][2],  migi[25][3]} }, //Middle1
					//{ { migi[26][0], migi[26][1],  migi[26][2],  migi[26][3]}, { migi[27][0],  migi[27][1],  migi[27][2],  migi[27][3]} }, //Middle2
					//{ { migi[28][0], migi[28][1],  migi[28][2],  migi[28][3]}, { migi[29][0],  migi[29][1],  migi[29][2],  migi[29][3]} }, //Middle3
					//{ { migi[30][0], migi[30][1],  migi[30][2],  migi[30][3]}, { migi[31][0],  migi[31][1],  migi[31][2],  migi[31][3]} }, //Middle4
					//{ { migi[32][0], migi[32][1],  migi[32][2],  migi[32][3]}, { migi[33][0],  migi[33][1],  migi[33][2],  migi[33][3]} }, //Ring0
					//{ { migi[34][0], migi[34][1],  migi[34][2],  migi[34][3]}, { migi[35][0],  migi[35][1],  migi[35][2],  migi[35][3]} }, //Ring1
					//{ { migi[36][0], migi[36][1],  migi[36][2],  migi[36][3]}, { migi[37][0],  migi[37][1],  migi[37][2],  migi[37][3]} }, //Ring2
					//{ { migi[38][0], migi[38][1],  migi[38][2],  migi[38][3]}, { migi[39][0],  migi[39][1],  migi[39][2],  migi[39][3]} }, //Ring3
					//{ { migi[40][0], migi[40][1],  migi[40][2],  migi[40][3]}, { migi[41][0],  migi[41][1],  migi[41][2],  migi[41][3]} }, //Ring4
					//{ { migi[42][0], migi[42][1],  migi[42][2],  migi[42][3]}, { migi[43][0],  migi[43][1],  migi[43][2],  migi[43][3]} }, //Pinky0
					//{ { migi[44][0], migi[44][1],  migi[44][2],  migi[44][3]}, { migi[45][0],  migi[45][1],  migi[45][2],  migi[45][3]} }, //Pinky1
					//{ { migi[46][0], migi[46][1],  migi[46][2],  migi[46][3]}, { migi[47][0],  migi[47][1],  migi[47][2],  migi[47][3]} }, //Pinky2
					//{ { migi[48][0], migi[48][1],  migi[48][2],  migi[48][3]}, { migi[49][0],  migi[49][1],  migi[49][2],  migi[49][3]} }, //Pinky3
					//{ { migi[50][0], migi[50][1],  migi[50][2],  migi[50][3]}, { migi[51][0],  migi[51][1],  migi[51][2],  migi[51][3]} }, //Pinky4
					//{ { migi[52][0], migi[52][1],  migi[52][2],  migi[52][3]}, { migi[53][0],  migi[53][1],  migi[53][2],  migi[53][3]} }, //AuxThumb
					//{ { migi[54][0], migi[54][1],  migi[54][2],  migi[54][3]}, { migi[55][0],  migi[55][1],  migi[55][2],  migi[55][3]} }, //AuxIndex
					//{ { migi[56][0], migi[56][1],  migi[56][2],  migi[56][3]}, { migi[57][0],  migi[57][1],  migi[57][2],  migi[57][3]} }, //AuxMiddle
					//{ { migi[58][0], migi[58][1],  migi[58][2],  migi[58][3]}, { migi[59][0],  migi[59][1],  migi[59][2],  migi[59][3]} }, //AuxRing
					//{ { migi[60][0], migi[60][1],  migi[60][2],  migi[60][3]}, { migi[61][0],  migi[61][1],  migi[61][2],  migi[61][3]} }, //AuxPinky
					//		};
					//		rightpose = right_hand_pose;
					//		rightposeev = right_hand_pose;
				}

				chc = bendi;

				success = true;
			}
			else if (tokens[0] == "pos")
			{
				// set the position of this controller
				double x = atof(tokens[1].c_str());
				double y = atof(tokens[2].c_str());
				double z = atof(tokens[3].c_str());

				if (m_device->m_role == TrackedControllerRole_RightHand)
				{
					pipePSM.vecPosition[0] = x;
					pipePSM.vecPosition[1] = y;
					pipePSM.vecPosition[2] = z;
				}
				else
				{
					pipePSH.vecPosition[0] = x;
					pipePSH.vecPosition[1] = y;
					pipePSH.vecPosition[2] = z;
				}

				// the controller has an update thread, so it'll get posted on the next update
				success = true;
			}
			else if (tokens[0] == "rot")
			{
				double yaw = atof(tokens[1].c_str());
				double roll = atof(tokens[2].c_str());
				double pitch = atof(tokens[3].c_str());

				yaw = yaw / (180 / Pi);
				roll = roll / (180 / Pi);
				pitch = pitch / (180 / Pi);

				t0 = cos(yaw * 0.5);
				t1 = sin(yaw * 0.5);
				t2 = cos(roll * 0.5);
				t3 = sin(roll * 0.5);
				t4 = cos(pitch * 0.5);
				t5 = sin(pitch * 0.5);

				if (m_device->m_role == TrackedControllerRole_RightHand)
				{
					pipePSM.qRotation.w = t0 * t2 * t4 + t1 * t3 * t5;
					pipePSM.qRotation.x = t0 * t3 * t4 - t1 * t2 * t5;
					pipePSM.qRotation.y = t0 * t2 * t5 + t1 * t3 * t4;
					pipePSM.qRotation.z = t1 * t2 * t4 - t0 * t3 * t5;
				}
				else
				{
					pipePSH.qRotation.w = t0 * t2 * t4 + t1 * t3 * t5;
					pipePSH.qRotation.x = t0 * t3 * t4 - t1 * t2 * t5;
					pipePSH.qRotation.y = t0 * t2 * t5 + t1 * t3 * t4;
					pipePSH.qRotation.z = t1 * t2 * t4 - t0 * t3 * t5;
				}

				success = true;
			}
			else
			{
				// tokens[0] is an input state path
				string input_state_path = tokens[0];

				dprintf("INPUT: %s\n", input_state_path.c_str());

				auto iter = m_inputstring2index.find(input_state_path);
				if (iter != m_inputstring2index.end())
				{
					uint32_t index = (*iter).second;
					VRInputComponentHandle_t component_handle = m_device->m_component_handles[index];
					ComponentType component_type = m_device->m_component_definitions[index].component_type;
					if (component_type == CT_BOOLEAN)
					{
						bool new_value = (tokens[1] == "1");
						dprintf("setting %s to %d\n", input_state_path.c_str(), new_value);
						EVRInputError err = VRDriverInput()->UpdateBooleanComponent(component_handle, new_value, 0);
						if (err != VRInputError_None)
						{
							dprintf("error %d\n", err);
							success = false;
						}
						else
						{
							success = true;
						}
					}
					else if (component_type == CT_SCALAR)
					{
						float new_value = static_cast<float>(atof(tokens[1].c_str()));
						dprintf("setting %s to %f\n", input_state_path.c_str(), new_value);
						EVRInputError err = VRDriverInput()->UpdateScalarComponent(component_handle, new_value, 0);
						if (err != VRInputError_None)
						{
							dprintf("error %d\n", err);
						}
						else
						{
							success = true;
						}
					}
				}
				else
				{
					dprintf("could not find component named %s\n", input_state_path.c_str());
				}
			}
		}
		else
		{
			dprintf("not enough tokens: %d\n", tokens.size());
		}

		if (success)
		{
			set_response("ok", response, response_buffer_size);
		}
		else
		{
			set_response("fail", response, response_buffer_size);
		}
	}

	float map(float value, float start1, float stop1, float start2, float stop2)
	{
		float outgoing =
			start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));

		return outgoing;
	}

	float migi[62][4];
	float hidari[62][4];

	float hidariopen[62][4] = {
		{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}, //Root
		{-0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f}, //Wrist
		{-0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.464112f, 0.567418f, 0.272106f, 0.623374f}, //Thumb0
		{0.040406f, 0.000000f, -0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}, //Thumb1
		{0.032517f, 0.000000f, 0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}, //Thumb2
		{0.030464f, -0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}, //Thumb3
		{0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}, //Index0
		{0.074204f, -0.005002f, 0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}, //Index1
		{0.043930f, -0.000000f, -0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}, //Index2
		{0.028695f, 0.000000f, 0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}, //Index3
		{0.022821f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}, //Index4
		{0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.546723f, 0.541276f, -0.442520f, 0.460749f}, //Middle0
		{0.070953f, 0.000779f, 0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}, //Middle1
		{0.043108f, 0.000000f, 0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}, //Middle2
		{0.033266f, 0.000000f, 0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}, //Middle3
		{0.025892f, -0.000000f, 0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}, //Middle4
		{0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.516692f, 0.550143f, -0.495548f, 0.429888f}, //Ring0
		{0.065876f, 0.001786f, 0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}, //Ring1
		{0.040697f, 0.000000f, 0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}, //Ring2
		{0.028747f, -0.000000f, -0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}, //Ring3
		{0.022430f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}, //Ring4
		{-0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.526918f, 0.523940f, -0.584025f, 0.326740f}, //Pinky0
		{0.062878f, 0.002844f, 0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}, //Pinky1
		{0.030220f, 0.000000f, 0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}, //Pinky2
		{0.018187f, 0.000000f, 0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}, //Pinky3
		{0.018018f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}, //Pinky4
		{-0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, 0.594267f, 0.249441f}, //AuxThumb
		{-0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, -0.663809f, -0.293734f}, //AuxIndex
		{-0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, -0.659285f, -0.265683f}, //AuxMiddle
		{-0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, -0.634757f, -0.143936f}, //AuxRing
		{-0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, -0.639342f, -0.126678f}, //AuxPinky
	};

	float hidariclosed[62][4] = {
		{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{-0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f},
		{-0.016305f, 0.027529f, 0.017800f, 1.000000f}, {0.225703f, 0.483332f, 0.126413f, 0.836342f},
		{0.040406f, 0.000000f, -0.000000f, 1.000000f}, {0.894335f, -0.013302f, -0.082902f, 0.439448f},
		{0.032517f, 0.000000f, 0.000000f, 1.000000f}, {0.842428f, 0.000655f, 0.001244f, 0.538807f},
		{0.030464f, -0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{0.003802f, 0.021514f, 0.012803f, 1.000000f}, {0.617314f, 0.395175f, -0.510874f, 0.449185f},
		{0.074204f, -0.005002f, 0.000234f, 1.000000f}, {0.737291f, -0.032006f, -0.115013f, 0.664944f},
		{0.043287f, -0.000000f, -0.000000f, 1.000000f}, {0.611381f, 0.003287f, 0.003823f, 0.791321f},
		{0.028275f, 0.000000f, 0.000000f, 1.000000f}, {0.745388f, -0.000684f, -0.000945f, 0.666629f},
		{0.022821f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f},
		{0.005787f, 0.006806f, 0.016534f, 1.000000f}, {0.514203f, 0.522315f, -0.478348f, 0.483700f},
		{0.070953f, 0.000779f, 0.000997f, 1.000000f}, {0.723653f, -0.097901f, 0.048546f, 0.681458f},
		{0.043108f, 0.000000f, 0.000000f, 1.000000f}, {0.637464f, -0.002366f, -0.002831f, 0.770472f},
		{0.033266f, 0.000000f, 0.000000f, 1.000000f}, {0.658008f, 0.002610f, 0.003196f, 0.753000f},
		{0.025892f, -0.000000f, 0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f},
		{0.004123f, -0.006858f, 0.016563f, 1.000000f}, {0.489609f, 0.523374f, -0.520644f, 0.463997f},
		{0.065876f, 0.001786f, 0.000693f, 1.000000f}, {0.759970f, -0.055609f, 0.011571f, 0.647471f},
		{0.040331f, 0.000000f, 0.000000f, 1.000000f}, {0.664315f, 0.001595f, 0.001967f, 0.747449f},
		{0.028489f, -0.000000f, -0.000000f, 1.000000f}, {0.626957f, -0.002784f, -0.003234f, 0.779042f},
		{0.022430f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{0.001131f, -0.019295f, 0.015429f, 1.000000f}, {0.479766f, 0.477833f, -0.630198f, 0.379934f},
		{0.062878f, 0.002844f, 0.000332f, 1.000000f}, {0.827001f, 0.034282f, 0.003440f, 0.561144f},
		{0.029874f, 0.000000f, 0.000000f, 1.000000f}, {0.702185f, -0.006716f, -0.009289f, 0.711903f},
		{0.017979f, 0.000000f, 0.000000f, 1.000000f}, {0.676853f, 0.007956f, 0.009917f, 0.736009f},
		{0.018018f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{0.019716f, 0.002802f, 0.093937f, 1.000000f}, {0.377286f, -0.540831f, 0.150446f, -0.736562f},
		{0.000171f, 0.016473f, 0.096515f, 1.000000f}, {-0.006456f, 0.022747f, -0.932927f, -0.359287f},
		{0.000448f, 0.001536f, 0.116543f, 1.000000f}, {-0.039357f, 0.105143f, -0.928833f, -0.353079f},
		{0.003949f, -0.014869f, 0.130608f, 1.000000f}, {-0.055071f, 0.068695f, -0.944016f, -0.317933f},
		{0.003263f, -0.034685f, 0.139926f, 1.000000f}, {0.019690f, -0.100741f, -0.957331f, -0.270149f},
	};

	float migiopen[62][4] = {
		{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, 0.920279f, -0.379296f},
		{0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.567418f, -0.464112f, 0.623374f, -0.272106f},
		{-0.040406f, -0.000000f, 0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f},
		{-0.032517f, -0.000000f, -0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f},
		{-0.030464f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f},
		{-0.074204f, 0.005002f, -0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f},
		{-0.043930f, 0.000000f, 0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f},
		{-0.028695f, -0.000000f, -0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f},
		{-0.022821f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f},
		{-0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.541276f, -0.546723f, 0.460749f, 0.442520f},
		{-0.070953f, -0.000779f, -0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f},
		{-0.043108f, -0.000000f, -0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f},
		{-0.033266f, -0.000000f, -0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f},
		{-0.025892f, 0.000000f, -0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f},
		{-0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.550143f, -0.516692f, 0.429888f, 0.495548f},
		{-0.065876f, -0.001786f, -0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f},
		{-0.040697f, -0.000000f, -0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f},
		{-0.028747f, 0.000000f, 0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f},
		{-0.022430f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.523940f, -0.526918f, 0.326740f, 0.584025f},
		{-0.062878f, -0.002844f, -0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f},
		{-0.030220f, -0.000000f, -0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f},
		{-0.018187f, -0.000000f, -0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f},
		{-0.018018f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, -0.594267f, -0.249441f},
		{0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, 0.663809f, 0.293734f},
		{0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, 0.659285f, 0.265683f},
		{0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, 0.634757f, 0.143936f},
		{0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, 0.639342f, 0.126678f},
	};

	float migiclosed[62][4] = {
		{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, 0.920279f, -0.379296f},
		{0.016305f, 0.027529f, 0.017800f, 1.000000f}, {0.483332f, -0.225703f, 0.836342f, -0.126413f},
		{-0.040406f, -0.000000f, 0.000000f, 1.000000f}, {0.894335f, -0.013302f, -0.082902f, 0.439448f},
		{-0.032517f, -0.000000f, -0.000000f, 1.000000f}, {0.842428f, 0.000655f, 0.001244f, 0.538807f},
		{-0.030464f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f},
		{-0.003802f, 0.021514f, 0.012803f, 1.000000f}, {0.395174f, -0.617314f, 0.449185f, 0.510874f},
		{-0.074204f, 0.005002f, -0.000234f, 1.000000f}, {0.737291f, -0.032006f, -0.115013f, 0.664944f},
		{-0.043287f, 0.000000f, 0.000000f, 1.000000f}, {0.611381f, 0.003287f, 0.003823f, 0.791321f},
		{-0.028275f, -0.000000f, -0.000000f, 1.000000f}, {0.745388f, -0.000684f, -0.000945f, 0.666629f},
		{-0.022821f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f},
		{-0.005787f, 0.006806f, 0.016534f, 1.000000f}, {0.522315f, -0.514203f, 0.483700f, 0.478348f},
		{-0.070953f, -0.000779f, -0.000997f, 1.000000f}, {0.723653f, -0.097901f, 0.048546f, 0.681458f},
		{-0.043108f, -0.000000f, -0.000000f, 1.000000f}, {0.637464f, -0.002366f, -0.002831f, 0.770472f},
		{-0.033266f, -0.000000f, -0.000000f, 1.000000f}, {0.658008f, 0.002610f, 0.003196f, 0.753000f},
		{-0.025892f, 0.000000f, -0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f},
		{-0.004123f, -0.006858f, 0.016563f, 1.000000f}, {0.523374f, -0.489609f, 0.463997f, 0.520644f},
		{-0.065876f, -0.001786f, -0.000693f, 1.000000f}, {0.759970f, -0.055609f, 0.011571f, 0.647471f},
		{-0.040331f, -0.000000f, -0.000000f, 1.000000f}, {0.664315f, 0.001595f, 0.001967f, 0.747449f},
		{-0.028489f, 0.000000f, 0.000000f, 1.000000f}, {0.626957f, -0.002784f, -0.003234f, 0.779042f},
		{-0.022430f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{-0.001131f, -0.019295f, 0.015429f, 1.000000f}, {0.477833f, -0.479766f, 0.379935f, 0.630198f},
		{-0.062878f, -0.002844f, -0.000332f, 1.000000f}, {0.827001f, 0.034282f, 0.003440f, 0.561144f},
		{-0.029874f, -0.000000f, -0.000000f, 1.000000f}, {0.702185f, -0.006716f, -0.009289f, 0.711903f},
		{-0.017979f, -0.000000f, -0.000000f, 1.000000f}, {0.676853f, 0.007956f, 0.009917f, 0.736009f},
		{-0.018018f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f},
		{-0.019716f, 0.002802f, 0.093937f, 1.000000f}, {0.377286f, -0.540831f, -0.150446f, 0.736562f},
		{-0.000171f, 0.016473f, 0.096515f, 1.000000f}, {-0.006456f, 0.022747f, 0.932927f, 0.359287f},
		{-0.000448f, 0.001536f, 0.116543f, 1.000000f}, {-0.039357f, 0.105143f, 0.928833f, 0.353079f},
		{-0.003949f, -0.014869f, 0.130608f, 1.000000f}, {-0.055071f, 0.068695f, 0.944016f, 0.317933f},
		{-0.003263f, -0.034685f, 0.139926f, 1.000000f}, {0.019690f, -0.100741f, 0.957331f, 0.270149f},
	};

	void transformleftroot(float bend)
	{
		for (int i2 = 0; i2 < 2; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftwrist(float bend)
	{
		for (int i2 = 2; i2 < 4; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftthumb(float bend)
	{
		for (int i0 = 4; i0 < 12; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				hidari[i0][i1] = map(bend, 0, 100, hidariclosed[i0][i1], hidariopen[i0][i1]);
			}
		}

		for (int i2 = 52; i2 < 54; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftindex(float bend)
	{
		for (int i0 = 12; i0 < 22; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				hidari[i0][i1] = map(bend, 0, 100, hidariclosed[i0][i1], hidariopen[i0][i1]);
			}
		}

		for (int i2 = 54; i2 < 56; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftmiddle(float bend)
	{
		for (int i0 = 22; i0 < 32; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				hidari[i0][i1] = map(bend, 0, 100, hidariclosed[i0][i1], hidariopen[i0][i1]);
			}
		}

		for (int i2 = 56; i2 < 58; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftring(float bend)
	{
		for (int i0 = 32; i0 < 42; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				hidari[i0][i1] = map(bend, 0, 100, hidariclosed[i0][i1], hidariopen[i0][i1]);
			}
		}

		for (int i2 = 58; i2 < 60; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformleftpinky(float bend)
	{
		for (int i0 = 42; i0 < 52; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				hidari[i0][i1] = map(bend, 0, 100, hidariclosed[i0][i1], hidariopen[i0][i1]);
			}
		}

		for (int i2 = 60; i2 < 62; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				hidari[i2][i3] = map(bend, 0, 100, hidariclosed[i2][i3], hidariopen[i2][i3]);
			}
		}
	}

	void transformrightroot(float bend)
	{
		for (int i2 = 0; i2 < 2; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightwrist(float bend)
	{
		for (int i2 = 2; i2 < 4; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightthumb(float bend)
	{
		for (int i0 = 4; i0 < 12; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				migi[i0][i1] = map(bend, 0, 100, migiclosed[i0][i1], migiopen[i0][i1]);
			}
		}

		for (int i2 = 52; i2 < 54; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightindex(float bend)
	{
		for (int i0 = 12; i0 < 22; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				migi[i0][i1] = map(bend, 0, 100, migiclosed[i0][i1], migiopen[i0][i1]);
			}
		}

		for (int i2 = 54; i2 < 56; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightmiddle(float bend)
	{
		for (int i0 = 22; i0 < 32; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				migi[i0][i1] = map(bend, 0, 100, migiclosed[i0][i1], migiopen[i0][i1]);
			}
		}

		for (int i2 = 56; i2 < 58; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightring(float bend)
	{
		for (int i0 = 32; i0 < 42; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				migi[i0][i1] = map(bend, 0, 100, migiclosed[i0][i1], migiopen[i0][i1]);
			}
		}

		for (int i2 = 58; i2 < 60; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformrightpinky(float bend)
	{
		for (int i0 = 42; i0 < 52; i0++)
		{
			for (int i1 = 0; i1 < 4; i1++)
			{
				migi[i0][i1] = map(bend, 0, 100, migiclosed[i0][i1], migiopen[i0][i1]);
			}
		}

		for (int i2 = 60; i2 < 62; i2++)
		{
			for (int i3 = 0; i3 < 4; i3++)
			{
				migi[i2][i3] = map(bend, 0, 100, migiclosed[i2][i3], migiopen[i2][i3]);
			}
		}
	}

	void transformallleft(float bend)
	{
		transformleftroot(bend);
		transformleftwrist(bend);
		transformleftthumb(bend);
		transformleftindex(bend);
		transformleftmiddle(bend);
		transformleftring(bend);
		transformleftpinky(bend);
	}

	void transformallright(float bend)
	{
		transformrightroot(bend);
		transformrightwrist(bend);
		transformrightthumb(bend);
		transformrightindex(bend);
		transformrightmiddle(bend);
		transformrightring(bend);
		transformrightpinky(bend);
	}

	void transformall(float bend)
	{
		transformallleft(bend);
		transformallright(bend);
	}

	void UpdateSkeleton(const char* un_WhichHand, const char* un_WhichFinger, float bend)
	{
		if (un_WhichHand == "LEFT")
		{
			if (un_WhichFinger == "THUMB")
				transformleftthumb(bend);
			if (un_WhichFinger == "INDEX")
				transformleftindex(bend);
			if (un_WhichFinger == "MIDDLE")
				transformleftmiddle(bend);
			if (un_WhichFinger == "RING")
				transformleftring(bend);
			if (un_WhichFinger == "PINKY")
				transformleftpinky(bend);
		}
		else if (un_WhichHand == "RIGHT")
		{
			if (un_WhichFinger == "THUMB")
				transformrightthumb(bend);
			if (un_WhichFinger == "INDEX")
				transformrightindex(bend);
			if (un_WhichFinger == "MIDDLE")
				transformrightmiddle(bend);
			if (un_WhichFinger == "RING")
				transformrightring(bend);
			if (un_WhichFinger == "PINKY")
				transformrightpinky(bend);
		}
	}

	SoftKnucklesDevice::SoftKnucklesDevice()
		: m_id(k_unTrackedDeviceIndexInvalid),
		  m_activated(false),
		  m_driver_context(nullptr),
		  m_tracked_device_container(k_unTrackedDeviceIndexInvalid),
		  m_role(TrackedControllerRole_Invalid),
		  m_running(false)
	{
		dprintf("New Device\n");
		m_pose = {0};
		m_pose.poseIsValid = true;
		m_pose.result = TrackingResult_Running_OK;
		m_pose.deviceIsConnected = true;
		m_pose.qWorldFromDriverRotation.w = 1;
		m_pose.qWorldFromDriverRotation.x = 0;
		m_pose.qWorldFromDriverRotation.y = 0;
		m_pose.qWorldFromDriverRotation.z = 0;
		m_pose.qDriverFromHeadRotation.w = 1;
		m_pose.qDriverFromHeadRotation.x = 0;
		m_pose.qDriverFromHeadRotation.y = 0;
		m_pose.qDriverFromHeadRotation.z = 0;

		m_pose.vecPosition[0] = 0;
		m_pose.vecPosition[1] = 0;
		m_pose.vecPosition[2] = 0;

		pipePSM = {0};
		pipePSM.poseIsValid = true;
		pipePSM.result = TrackingResult_Running_OK;
		pipePSM.deviceIsConnected = true;
		pipePSM.qWorldFromDriverRotation.w = 1;
		pipePSM.qWorldFromDriverRotation.x = 0;
		pipePSM.qWorldFromDriverRotation.y = 0;
		pipePSM.qWorldFromDriverRotation.z = 0;
		pipePSM.qDriverFromHeadRotation.w = 1;
		pipePSM.qDriverFromHeadRotation.x = 0;
		pipePSM.qDriverFromHeadRotation.y = 0;
		pipePSM.qDriverFromHeadRotation.z = 0;

		pipePSM.vecPosition[0] = 0;
		pipePSM.vecPosition[1] = 0;
		pipePSM.vecPosition[2] = 0;

		pipePSH = {0};
		pipePSH.poseIsValid = true;
		pipePSH.result = TrackingResult_Running_OK;
		pipePSH.deviceIsConnected = true;
		pipePSH.qWorldFromDriverRotation.w = 1;
		pipePSH.qWorldFromDriverRotation.x = 0;
		pipePSH.qWorldFromDriverRotation.y = 0;
		pipePSH.qWorldFromDriverRotation.z = 0;
		pipePSH.qDriverFromHeadRotation.w = 1;
		pipePSH.qDriverFromHeadRotation.x = 0;
		pipePSH.qDriverFromHeadRotation.y = 0;
		pipePSH.qDriverFromHeadRotation.z = 0;

		pipePSH.vecPosition[0] = 0;
		pipePSH.vecPosition[1] = 0;
		pipePSH.vecPosition[2] = 0;

		hposet = {0};
		hposet.poseIsValid = true;
		hposet.result = TrackingResult_Running_OK;
		hposet.deviceIsConnected = true;
		hposet.qWorldFromDriverRotation.w = 1;
		hposet.qWorldFromDriverRotation.x = 0;
		hposet.qWorldFromDriverRotation.y = 0;
		hposet.qWorldFromDriverRotation.z = 0;
		hposet.qDriverFromHeadRotation.w = 1;
		hposet.qDriverFromHeadRotation.x = 0;
		hposet.qDriverFromHeadRotation.y = 0;
		hposet.qDriverFromHeadRotation.z = 0;

		hposet.vecPosition[0] = 0;
		hposet.vecPosition[1] = 0;
		hposet.vecPosition[2] = 0;

		mposet = {0};
		mposet.poseIsValid = true;
		mposet.result = TrackingResult_Running_OK;
		mposet.deviceIsConnected = true;
		mposet.qWorldFromDriverRotation.w = 1;
		mposet.qWorldFromDriverRotation.x = 0;
		mposet.qWorldFromDriverRotation.y = 0;
		mposet.qWorldFromDriverRotation.z = 0;
		mposet.qDriverFromHeadRotation.w = 1;
		mposet.qDriverFromHeadRotation.x = 0;
		mposet.qDriverFromHeadRotation.y = 0;
		mposet.qDriverFromHeadRotation.z = 0;

		mposet.vecPosition[0] = 0;
		mposet.vecPosition[1] = 0;
		mposet.vecPosition[2] = 0;

		pposet = {0};
		pposet.poseIsValid = true;
		pposet.result = TrackingResult_Running_OK;
		pposet.deviceIsConnected = true;
		pposet.qWorldFromDriverRotation.w = 1;
		pposet.qWorldFromDriverRotation.x = 0;
		pposet.qWorldFromDriverRotation.y = 0;
		pposet.qWorldFromDriverRotation.z = 0;
		pposet.qDriverFromHeadRotation.w = 1;
		pposet.qDriverFromHeadRotation.x = 0;
		pposet.qDriverFromHeadRotation.y = 0;
		pposet.qDriverFromHeadRotation.z = 0;

		pposet.vecPosition[0] = 0;
		pposet.vecPosition[1] = 0;
		pposet.vecPosition[2] = 0;

		transformleftthumb(bendt);
		transformleftindex(bendi);
		transformleftmiddle(bendm);
		transformleftring(bendr);
		transformleftpinky(bendl);
		static VRBoneTransform_t left_hand_pose[NUM_BONES] = {
			{
				{hidari[0][0], hidari[0][1], hidari[0][2], hidari[0][3]},
				{hidari[1][0], hidari[1][1], hidari[1][2], hidari[1][3]}
			}, //Root
			{
				{hidari[2][0], hidari[2][1], hidari[2][2], hidari[2][3]},
				{hidari[3][0], hidari[3][1], hidari[3][2], hidari[3][3]}
			}, //Wrist
			{
				{hidari[4][0], hidari[4][1], hidari[4][2], hidari[4][3]},
				{hidari[5][0], hidari[5][1], hidari[5][2], hidari[5][3]}
			}, //Thumb0
			{
				{hidari[6][0], hidari[6][1], hidari[6][2], hidari[6][3]},
				{hidari[7][0], hidari[7][1], hidari[7][2], hidari[7][3]}
			}, //Thumb1
			{
				{hidari[8][0], hidari[8][1], hidari[8][2], hidari[8][3]},
				{hidari[9][0], hidari[9][1], hidari[9][2], hidari[9][3]}
			}, //Thumb2
			{
				{hidari[10][0], hidari[10][1], hidari[10][2], hidari[10][3]},
				{hidari[11][0], hidari[11][1], hidari[11][2], hidari[11][3]}
			}, //Thumb3
			{
				{hidari[12][0], hidari[12][1], hidari[12][2], hidari[12][3]},
				{hidari[13][0], hidari[13][1], hidari[13][2], hidari[13][3]}
			}, //Index0
			{
				{hidari[14][0], hidari[14][1], hidari[14][2], hidari[14][3]},
				{hidari[15][0], hidari[15][1], hidari[15][2], hidari[15][3]}
			}, //Index1
			{
				{hidari[16][0], hidari[16][1], hidari[16][2], hidari[16][3]},
				{hidari[17][0], hidari[17][1], hidari[17][2], hidari[17][3]}
			}, //Index2
			{
				{hidari[18][0], hidari[18][1], hidari[18][2], hidari[18][3]},
				{hidari[19][0], hidari[19][1], hidari[19][2], hidari[19][3]}
			}, //Index3
			{
				{hidari[20][0], hidari[20][1], hidari[20][2], hidari[20][3]},
				{hidari[21][0], hidari[21][1], hidari[21][2], hidari[21][3]}
			}, //Index4
			{
				{hidari[22][0], hidari[22][1], hidari[22][2], hidari[22][3]},
				{hidari[23][0], hidari[23][1], hidari[23][2], hidari[23][3]}
			}, //Middle0
			{
				{hidari[24][0], hidari[24][1], hidari[24][2], hidari[24][3]},
				{hidari[25][0], hidari[25][1], hidari[25][2], hidari[25][3]}
			}, //Middle1
			{
				{hidari[26][0], hidari[26][1], hidari[26][2], hidari[26][3]},
				{hidari[27][0], hidari[27][1], hidari[27][2], hidari[27][3]}
			}, //Middle2
			{
				{hidari[28][0], hidari[28][1], hidari[28][2], hidari[28][3]},
				{hidari[29][0], hidari[29][1], hidari[29][2], hidari[29][3]}
			}, //Middle3
			{
				{hidari[30][0], hidari[30][1], hidari[30][2], hidari[30][3]},
				{hidari[31][0], hidari[31][1], hidari[31][2], hidari[31][3]}
			}, //Middle4
			{
				{hidari[32][0], hidari[32][1], hidari[32][2], hidari[32][3]},
				{hidari[33][0], hidari[33][1], hidari[33][2], hidari[33][3]}
			}, //Ring0
			{
				{hidari[34][0], hidari[34][1], hidari[34][2], hidari[34][3]},
				{hidari[35][0], hidari[35][1], hidari[35][2], hidari[35][3]}
			}, //Ring1
			{
				{hidari[36][0], hidari[36][1], hidari[36][2], hidari[36][3]},
				{hidari[37][0], hidari[37][1], hidari[37][2], hidari[37][3]}
			}, //Ring2
			{
				{hidari[38][0], hidari[38][1], hidari[38][2], hidari[38][3]},
				{hidari[39][0], hidari[39][1], hidari[39][2], hidari[39][3]}
			}, //Ring3
			{
				{hidari[40][0], hidari[40][1], hidari[40][2], hidari[40][3]},
				{hidari[41][0], hidari[41][1], hidari[41][2], hidari[41][3]}
			}, //Ring4
			{
				{hidari[42][0], hidari[42][1], hidari[42][2], hidari[42][3]},
				{hidari[43][0], hidari[43][1], hidari[43][2], hidari[43][3]}
			}, //Pinky0
			{
				{hidari[44][0], hidari[44][1], hidari[44][2], hidari[44][3]},
				{hidari[45][0], hidari[45][1], hidari[45][2], hidari[45][3]}
			}, //Pinky1
			{
				{hidari[46][0], hidari[46][1], hidari[46][2], hidari[46][3]},
				{hidari[47][0], hidari[47][1], hidari[47][2], hidari[47][3]}
			}, //Pinky2
			{
				{hidari[48][0], hidari[48][1], hidari[48][2], hidari[48][3]},
				{hidari[49][0], hidari[49][1], hidari[49][2], hidari[49][3]}
			}, //Pinky3
			{
				{hidari[50][0], hidari[50][1], hidari[50][2], hidari[50][3]},
				{hidari[51][0], hidari[51][1], hidari[51][2], hidari[51][3]}
			}, //Pinky4
			{
				{hidari[52][0], hidari[52][1], hidari[52][2], hidari[52][3]},
				{hidari[53][0], hidari[53][1], hidari[53][2], hidari[53][3]}
			}, //AuxThumb
			{
				{hidari[54][0], hidari[54][1], hidari[54][2], hidari[54][3]},
				{hidari[55][0], hidari[55][1], hidari[55][2], hidari[55][3]}
			}, //AuxIndex
			{
				{hidari[56][0], hidari[56][1], hidari[56][2], hidari[56][3]},
				{hidari[57][0], hidari[57][1], hidari[57][2], hidari[57][3]}
			}, //AuxMiddle
			{
				{hidari[58][0], hidari[58][1], hidari[58][2], hidari[58][3]},
				{hidari[59][0], hidari[59][1], hidari[59][2], hidari[59][3]}
			}, //AuxRing
			{
				{hidari[60][0], hidari[60][1], hidari[60][2], hidari[60][3]},
				{hidari[61][0], hidari[61][1], hidari[61][2], hidari[61][3]}
			}, //AuxPinky
		};
		leftpose = left_hand_pose;
		leftposeev = left_hand_pose;

		transformrightthumb(bendt);
		transformrightindex(bendi);
		transformrightmiddle(bendm);
		transformrightring(bendr);
		transformrightpinky(bendl);
		static VRBoneTransform_t right_hand_pose[NUM_BONES] = {
			{{migi[0][0], migi[0][1], migi[0][2], migi[0][3]}, {migi[1][0], migi[1][1], migi[1][2], migi[1][3]}}, //Root
			{{migi[2][0], migi[2][1], migi[2][2], migi[2][3]}, {migi[3][0], migi[3][1], migi[3][2], migi[3][3]}},
			//Wrist
			{{migi[4][0], migi[4][1], migi[4][2], migi[4][3]}, {migi[5][0], migi[5][1], migi[5][2], migi[5][3]}},
			//Thumb0
			{{migi[6][0], migi[6][1], migi[6][2], migi[6][3]}, {migi[7][0], migi[7][1], migi[7][2], migi[7][3]}},
			//Thumb1
			{{migi[8][0], migi[8][1], migi[8][2], migi[8][3]}, {migi[9][0], migi[9][1], migi[9][2], migi[9][3]}},
			//Thumb2
			{
				{migi[10][0], migi[10][1], migi[10][2], migi[10][3]},
				{migi[11][0], migi[11][1], migi[11][2], migi[11][3]}
			}, //Thumb3
			{
				{migi[12][0], migi[12][1], migi[12][2], migi[12][3]},
				{migi[13][0], migi[13][1], migi[13][2], migi[13][3]}
			}, //Index0
			{
				{migi[14][0], migi[14][1], migi[14][2], migi[14][3]},
				{migi[15][0], migi[15][1], migi[15][2], migi[15][3]}
			}, //Index1
			{
				{migi[16][0], migi[16][1], migi[16][2], migi[16][3]},
				{migi[17][0], migi[17][1], migi[17][2], migi[17][3]}
			}, //Index2
			{
				{migi[18][0], migi[18][1], migi[18][2], migi[18][3]},
				{migi[19][0], migi[19][1], migi[19][2], migi[19][3]}
			}, //Index3
			{
				{migi[20][0], migi[20][1], migi[20][2], migi[20][3]},
				{migi[21][0], migi[21][1], migi[21][2], migi[21][3]}
			}, //Index4
			{
				{migi[22][0], migi[22][1], migi[22][2], migi[22][3]},
				{migi[23][0], migi[23][1], migi[23][2], migi[23][3]}
			}, //Middle0
			{
				{migi[24][0], migi[24][1], migi[24][2], migi[24][3]},
				{migi[25][0], migi[25][1], migi[25][2], migi[25][3]}
			}, //Middle1
			{
				{migi[26][0], migi[26][1], migi[26][2], migi[26][3]},
				{migi[27][0], migi[27][1], migi[27][2], migi[27][3]}
			}, //Middle2
			{
				{migi[28][0], migi[28][1], migi[28][2], migi[28][3]},
				{migi[29][0], migi[29][1], migi[29][2], migi[29][3]}
			}, //Middle3
			{
				{migi[30][0], migi[30][1], migi[30][2], migi[30][3]},
				{migi[31][0], migi[31][1], migi[31][2], migi[31][3]}
			}, //Middle4
			{
				{migi[32][0], migi[32][1], migi[32][2], migi[32][3]},
				{migi[33][0], migi[33][1], migi[33][2], migi[33][3]}
			}, //Ring0
			{
				{migi[34][0], migi[34][1], migi[34][2], migi[34][3]},
				{migi[35][0], migi[35][1], migi[35][2], migi[35][3]}
			}, //Ring1
			{
				{migi[36][0], migi[36][1], migi[36][2], migi[36][3]},
				{migi[37][0], migi[37][1], migi[37][2], migi[37][3]}
			}, //Ring2
			{
				{migi[38][0], migi[38][1], migi[38][2], migi[38][3]},
				{migi[39][0], migi[39][1], migi[39][2], migi[39][3]}
			}, //Ring3
			{
				{migi[40][0], migi[40][1], migi[40][2], migi[40][3]},
				{migi[41][0], migi[41][1], migi[41][2], migi[41][3]}
			}, //Ring4
			{
				{migi[42][0], migi[42][1], migi[42][2], migi[42][3]},
				{migi[43][0], migi[43][1], migi[43][2], migi[43][3]}
			}, //Pinky0
			{
				{migi[44][0], migi[44][1], migi[44][2], migi[44][3]},
				{migi[45][0], migi[45][1], migi[45][2], migi[45][3]}
			}, //Pinky1
			{
				{migi[46][0], migi[46][1], migi[46][2], migi[46][3]},
				{migi[47][0], migi[47][1], migi[47][2], migi[47][3]}
			}, //Pinky2
			{
				{migi[48][0], migi[48][1], migi[48][2], migi[48][3]},
				{migi[49][0], migi[49][1], migi[49][2], migi[49][3]}
			}, //Pinky3
			{
				{migi[50][0], migi[50][1], migi[50][2], migi[50][3]},
				{migi[51][0], migi[51][1], migi[51][2], migi[51][3]}
			}, //Pinky4
			{
				{migi[52][0], migi[52][1], migi[52][2], migi[52][3]},
				{migi[53][0], migi[53][1], migi[53][2], migi[53][3]}
			}, //AuxThumb
			{
				{migi[54][0], migi[54][1], migi[54][2], migi[54][3]},
				{migi[55][0], migi[55][1], migi[55][2], migi[55][3]}
			}, //AuxIndex
			{
				{migi[56][0], migi[56][1], migi[56][2], migi[56][3]},
				{migi[57][0], migi[57][1], migi[57][2], migi[57][3]}
			}, //AuxMiddle
			{
				{migi[58][0], migi[58][1], migi[58][2], migi[58][3]},
				{migi[59][0], migi[59][1], migi[59][2], migi[59][3]}
			}, //AuxRing
			{
				{migi[60][0], migi[60][1], migi[60][2], migi[60][3]},
				{migi[61][0], migi[61][1], migi[61][2], migi[61][3]}
			}, //AuxPinky
		};
		rightpose = right_hand_pose;
		rightposeev = right_hand_pose;

		setrindexbend(25);

		gtpt = 0;
	}

	void SoftKnucklesDevice::Init(
		ETrackedControllerRole role,
		const KnuckleComponentDefinition* component_definitions,
		uint32_t num_component_definitions,
		SoftKnucklesDebugHandler* debug_handler)
	{
		dprintf("Init for role: %d num_definitions %d\n", role, num_component_definitions);

		m_component_definitions = component_definitions;
		m_num_component_definitions = num_component_definitions;
		m_debug_handler = debug_handler;
		m_role = role;

		// look up config from soft_knuckles/resources/settings/default.vrsettings.
		char buf[1024];
		VRSettings()->GetString(kSettingsSection, "serialNumber", buf, sizeof(buf));
		m_serial_number = buf;
		VRSettings()->GetString(kSettingsSection, "modelNumber", buf, sizeof(buf));
		m_model_number = buf;

		if (m_role == TrackedControllerRole_LeftHand)
		{
			m_serial_number += "L";
			m_render_model_name = "{KinectToVR}/rendermodels/valve_controller_knu_ev2_0_left";
		}
		else if (m_role == TrackedControllerRole_RightHand)
		{
			m_serial_number += "R";
			m_render_model_name = "{KinectToVR}/rendermodels/valve_controller_knu_ev2_0_right";
		}

		dprintf("serial: %s\n", m_serial_number.c_str());
		dprintf("model_number: %s\n", m_model_number.c_str());

		if (m_debug_handler)
		{
			m_debug_handler->Init(this);
		}
	}

	void SoftKnucklesDevice::EnterStandby()
	{
		dprintf("EnterStandby()\n");
		if (m_running)
		{
			m_running = false;
		}
	}

	VRInputComponentHandle_t SoftKnucklesDevice::CreateBooleanComponent(const char* full_path)
	{
		dprintf("CreateBooleanComponent for %s on %d\n", full_path, m_tracked_device_container);
		VRInputComponentHandle_t input_handle = k_ulInvalidInputComponentHandle;
		EVRInputError input_error = VRDriverInput()->
			CreateBooleanComponent(m_tracked_device_container, full_path, &input_handle);
		// note it goes into the container specific to this instance of the device
		if (input_error != VRInputError_None)
		{
			dprintf("error %d\n", input_error);
		}
		else
		{
			dprintf("ok\n");
		}
		return input_handle;
	}

	VRInputComponentHandle_t SoftKnucklesDevice::CreateScalarComponent(const char* full_path, EVRScalarType scalar_type,
	                                                                   EVRScalarUnits scalar_units)
	{
		dprintf("CreateScalarComponent for %s on %d\n", full_path, m_tracked_device_container);
		VRInputComponentHandle_t input_handle = k_ulInvalidInputComponentHandle;
		EVRInputError input_error = VRDriverInput()->CreateScalarComponent(
			m_tracked_device_container, full_path, &input_handle,
			scalar_type, scalar_units);

		if (input_error != VRInputError_None)
		{
			dprintf("error %d\n", input_error);
		}
		else
		{
			dprintf("ok\n");
		}
		return input_handle;
	}

	VRInputComponentHandle_t SoftKnucklesDevice::CreateHapticComponent(const char* name)
	{
		dprintf("CreateHapticComponent for %s\n", name);
		VRInputComponentHandle_t input_handle = k_ulInvalidInputComponentHandle;
		EVRInputError input_error = VRDriverInput()->
			CreateHapticComponent(m_tracked_device_container, name, &input_handle);
		// note it goes into the container specific to this instance of the device
		if (input_error != VRInputError_None)
		{
			dprintf("error %d\n", input_error);
		}
		else
		{
			dprintf("ok\n");
		}
		return input_handle;
	}

	VRInputComponentHandle_t SoftKnucklesDevice::CreateSkeletonComponent(
		const char* name, const char* skeleton_path, const char* base_pose_path,
		const VRBoneTransform_t* pGripLimitTransforms, uint32_t unGripLimitTransformCount)
	{
		dprintf("CreateSkeletonComponent for %s\n", name);
		VRInputComponentHandle_t input_handle = k_ulInvalidInputComponentHandle;
		EVRInputError input_error = VRDriverInput()->CreateSkeletonComponent(
			m_tracked_device_container, name, skeleton_path, base_pose_path,
			VRSkeletalTracking_Partial, pGripLimitTransforms, unGripLimitTransformCount, &input_handle);

		if (input_error != VRInputError_None)
		{
			dprintf("error %d\n", input_error);
		}
		else
		{
			dprintf("ok\n");
		}
		return input_handle;
	}

	void SoftKnucklesDevice::SetProperty(ETrackedDeviceProperty prop_key, const char* prop_value)
	{
		VRProperties()->SetStringProperty(m_tracked_device_container, prop_key, prop_value);
	}

	void SoftKnucklesDevice::SetInt32Property(ETrackedDeviceProperty prop_key, int32_t value)
	{
		VRProperties()->SetInt32Property(m_tracked_device_container, prop_key, value);
	}

	void SoftKnucklesDevice::SetBoolProperty(ETrackedDeviceProperty prop_key, int32_t value)
	{
		VRProperties()->SetBoolProperty(m_tracked_device_container, prop_key, value);
	}

	void SoftKnucklesDevice::update_pose_thread(SoftKnucklesDevice* pthis)
	{
#ifdef _WIN32
		HRESULT hr = SetThreadDescription(GetCurrentThread(), L"update_pose_thread");
#endif
		bool m_show_open_hand_pose = true;

		int i;
		static int x, xr;
		int i1, i2;
		int cdx = 0;
		x = 0;
		bool outgo = true;
		bool outgor = true;

		while (pthis->m_running)
		{
			auto t1 = std::chrono::high_resolution_clock::now();

			if (pthis->m_role == TrackedControllerRole_RightHand)
			{
				VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->m_id, pipePSM, sizeof(DriverPose_t));
			}
			else if (pthis->m_role == TrackedControllerRole_LeftHand)
			{
				VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->m_id, pipePSH, sizeof(DriverPose_t));
			}

			float ltbend = getlthumbend();
			float libend = getlindexbend();
			float lmbend = getlmiddlebend();
			float lrbend = getlringbend();
			float llbend = getllittlebend();

			float rtbend = getrthumbend();
			float ribend = getrindexbend();
			float rmbend = getrmiddlebend();
			float rrbend = getrringbend();
			float rlbend = getrlittlebend();

			if (pthis->m_role == TrackedControllerRole_LeftHand)
			{
				glm::vec3 l_position;
				glm::quat l_rotation;

				glm::vec3 l_positionaux;
				glm::quat l_rotationaux;

				transformleftthumb(ltbend);
				//transformleftthumb(x);
				if (pthis->m_role == TrackedControllerRole_LeftHand)
				{
					i = 2;
					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_positionaux.x = hidari[52][0];
					l_positionaux.y = hidari[52][1];
					l_positionaux.z = hidari[52][2];

					l_rotationaux.w = hidari[53][0];
					l_rotationaux.x = hidari[53][1];
					l_rotationaux.y = hidari[53][2];
					l_rotationaux.z = hidari[53][3];

					ConvertVector3(l_positionaux, lbone_t[26].position);
					ConvertQuaternion(l_rotationaux, lbone_t[26].orientation);
				}

				transformleftindex(libend);
				//transformleftindex(x);
				if (pthis->m_role == TrackedControllerRole_LeftHand)
				{
					i = 6;
					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_positionaux.x = hidari[54][0];
					l_positionaux.y = hidari[54][1];
					l_positionaux.z = hidari[54][2];

					l_rotationaux.w = hidari[55][0];
					l_rotationaux.x = hidari[55][1];
					l_rotationaux.y = hidari[55][2];
					l_rotationaux.z = hidari[55][3];

					ConvertVector3(l_positionaux, lbone_t[27].position);
					ConvertQuaternion(l_rotationaux, lbone_t[27].orientation);
				}

				transformleftmiddle(lmbend);
				//transformleftmiddle(x);
				if (pthis->m_role == TrackedControllerRole_LeftHand)
				{
					i = 11;
					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_positionaux.x = hidari[56][0];
					l_positionaux.y = hidari[56][1];
					l_positionaux.z = hidari[56][2];

					l_rotationaux.w = hidari[57][0];
					l_rotationaux.x = hidari[57][1];
					l_rotationaux.y = hidari[57][2];
					l_rotationaux.z = hidari[57][3];

					ConvertVector3(l_positionaux, lbone_t[28].position);
					ConvertQuaternion(l_rotationaux, lbone_t[28].orientation);
				}

				transformleftring(lrbend);
				//transformleftring(x);
				if (pthis->m_role == TrackedControllerRole_LeftHand)
				{
					i = 16;
					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_positionaux.x = hidari[58][0];
					l_positionaux.y = hidari[58][1];
					l_positionaux.z = hidari[58][2];

					l_rotationaux.w = hidari[59][0];
					l_rotationaux.x = hidari[59][1];
					l_rotationaux.y = hidari[59][2];
					l_rotationaux.z = hidari[59][3];

					ConvertVector3(l_positionaux, lbone_t[29].position);
					ConvertQuaternion(l_rotationaux, lbone_t[29].orientation);
				}

				transformleftpinky(llbend);
				//transformleftpinky(x);
				if (pthis->m_role == TrackedControllerRole_LeftHand)
				{
					i = 21;
					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_rotation.w = hidari[i2][0];
					l_rotation.x = hidari[i2][1];
					l_rotation.y = hidari[i2][2];
					l_rotation.z = hidari[i2][3];

					ConvertVector3(l_position, lbone_t[i].position);
					ConvertQuaternion(l_rotation, lbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					l_position.x = hidari[i1][0];
					l_position.y = hidari[i1][1];
					l_position.z = hidari[i1][2];

					l_positionaux.x = hidari[60][0];
					l_positionaux.y = hidari[60][1];
					l_positionaux.z = hidari[60][2];

					l_rotationaux.w = hidari[61][0];
					l_rotationaux.x = hidari[61][1];
					l_rotationaux.y = hidari[61][2];
					l_rotationaux.z = hidari[61][3];

					ConvertVector3(l_positionaux, lbone_t[30].position);
					ConvertQuaternion(l_rotationaux, lbone_t[30].orientation);
				}

				if (outgo)x += 1;
				else x -= 1;

				if (x >= 100)outgo = false;
				if (x <= 0)outgo = true;
			}

			if (pthis->m_role == TrackedControllerRole_RightHand)
			{
				glm::vec3 r_position;
				glm::quat r_rotation;

				glm::vec3 r_positionaux;
				glm::quat r_rotationaux;

				transformrightthumb(rtbend);
				//transformrightthumb(xr);
				if (pthis->m_role == TrackedControllerRole_RightHand)
				{
					i = 2;
					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_positionaux.x = migi[52][0];
					r_positionaux.y = migi[52][1];
					r_positionaux.z = migi[52][2];

					r_rotationaux.w = migi[53][0];
					r_rotationaux.x = migi[53][1];
					r_rotationaux.y = migi[53][2];
					r_rotationaux.z = migi[53][3];

					ConvertVector3(r_positionaux, rbone_t[26].position);
					ConvertQuaternion(r_rotationaux, rbone_t[26].orientation);
				}

				transformrightindex(ribend);
				//transformrightindex(xr);
				if (pthis->m_role == TrackedControllerRole_RightHand)
				{
					i = 6;
					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_positionaux.x = migi[54][0];
					r_positionaux.y = migi[54][1];
					r_positionaux.z = migi[54][2];

					r_rotationaux.w = migi[55][0];
					r_rotationaux.x = migi[55][1];
					r_rotationaux.y = migi[55][2];
					r_rotationaux.z = migi[55][3];

					ConvertVector3(r_positionaux, rbone_t[27].position);
					ConvertQuaternion(r_rotationaux, rbone_t[27].orientation);
				}

				transformrightmiddle(rmbend);
				//transformrightmiddle(xr);
				if (pthis->m_role == TrackedControllerRole_RightHand)
				{
					i = 11;
					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_positionaux.x = migi[56][0];
					r_positionaux.y = migi[56][1];
					r_positionaux.z = migi[56][2];

					r_rotationaux.w = migi[57][0];
					r_rotationaux.x = migi[57][1];
					r_rotationaux.y = migi[57][2];
					r_rotationaux.z = migi[57][3];

					ConvertVector3(r_positionaux, rbone_t[28].position);
					ConvertQuaternion(r_rotationaux, rbone_t[28].orientation);
				}

				transformrightring(rrbend);
				//transformrightring(xr);
				if (pthis->m_role == TrackedControllerRole_RightHand)
				{
					i = 16;
					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_positionaux.x = migi[58][0];
					r_positionaux.y = migi[58][1];
					r_positionaux.z = migi[58][2];

					r_rotationaux.w = migi[59][0];
					r_rotationaux.x = migi[59][1];
					r_rotationaux.y = migi[59][2];
					r_rotationaux.z = migi[59][3];

					ConvertVector3(r_positionaux, rbone_t[29].position);
					ConvertQuaternion(r_rotationaux, rbone_t[29].orientation);
				}

				transformrightpinky(rlbend);
				//transformrightpinky(xr);
				if (pthis->m_role == TrackedControllerRole_RightHand)
				{
					i = 21;
					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_rotation.w = migi[i2][0];
					r_rotation.x = migi[i2][1];
					r_rotation.y = migi[i2][2];
					r_rotation.z = migi[i2][3];

					ConvertVector3(r_position, rbone_t[i].position);
					ConvertQuaternion(r_rotation, rbone_t[i].orientation);

					i = i + 1;

					i1 = (i * 2);
					i2 = i1 + 1;

					r_position.x = migi[i1][0];
					r_position.y = migi[i1][1];
					r_position.z = migi[i1][2];

					r_positionaux.x = migi[60][0];
					r_positionaux.y = migi[60][1];
					r_positionaux.z = migi[60][2];

					r_rotationaux.w = migi[61][0];
					r_rotationaux.x = migi[61][1];
					r_rotationaux.y = migi[61][2];
					r_rotationaux.z = migi[61][3];

					ConvertVector3(r_positionaux, rbone_t[30].position);
					ConvertQuaternion(r_rotationaux, rbone_t[30].orientation);
				}

				if (outgor)xr += 1;
				else xr -= 1;

				if (xr >= 100)outgor = false;
				if (xr <= 0)outgor = true;
			}

			if (pthis->m_role == TrackedControllerRole_LeftHand)
			{
				for (int i = 0; i < pthis->m_component_handles.size(); i++)
				{
					if (pthis->m_component_definitions[i].component_type == CT_SKELETON)
					{
						if (strcmp(pthis->m_component_definitions[i].skeleton_path, "/skeleton/hand/left") == 0)
						{
							VRDriverInput()->UpdateSkeletonComponent(
								pthis->m_component_handles[i],
								VRSkeletalMotionRange_WithoutController,
								lbone_t,
								NUM_BONES);
							VRDriverInput()->UpdateSkeletonComponent(
								pthis->m_component_handles[i],
								VRSkeletalMotionRange_WithController,
								lbone_t,
								NUM_BONES);
						}
					}
				}
			}
			else if (pthis->m_role == TrackedControllerRole_RightHand)
			{
				for (int i = 0; i < pthis->m_component_handles.size(); i++)
				{
					if (pthis->m_component_definitions[i].component_type == CT_SKELETON)
					{
						if (strcmp(pthis->m_component_definitions[i].skeleton_path, "/skeleton/hand/right") == 0)
						{
							VRDriverInput()->UpdateSkeletonComponent(
								pthis->m_component_handles[i],
								VRSkeletalMotionRange_WithoutController,
								rbone_t,
								NUM_BONES);
							VRDriverInput()->UpdateSkeletonComponent(
								pthis->m_component_handles[i],
								VRSkeletalMotionRange_WithController,
								rbone_t,
								NUM_BONES);
						}
					}
				}
			}

			auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::high_resolution_clock::now() - t1).count();
			if (duration <= 9000000.f)
			{
				std::this_thread::sleep_for(std::chrono::nanoseconds(9000000 - duration));
			}
		}
	}

	EVRInitError SoftKnucklesDevice::Activate(uint32_t unObjectId)
	{
		dprintf("Activated.  object ID: %d\n", unObjectId);
		if (m_activated)
		{
			dprintf("warning: Activate called twice\n");
			return VRInitError_Driver_Failed;
		}
		m_activated = true;
		m_id = unObjectId;
		m_tracked_device_container = VRProperties()->TrackedDeviceToPropertyContainer(m_id);

		//SetProperty(Prop_SerialNumber_String, m_serial_number.c_str());
		//SetProperty(Prop_ModelNumber_String, m_model_number.c_str());
		//SetProperty(Prop_RenderModelName_String, m_render_model_name.c_str());
		//SetProperty(Prop_ManufacturerName_String, "Valve");
		//SetInt32Property(Prop_ControllerRoleHint_Int32, m_role);
		//SetInt32Property(Prop_DeviceClass_Int32, static_cast<int32_t>(TrackedDeviceClass_Controller));
		//SetProperty(Prop_InputProfilePath_String, "{KinectToVR}/input/index_profile.json");
		//SetProperty(Prop_ControllerType_String, "knuckles");
		////SetProperty(Prop_LegacyInputProfile_String, "{KinectToVR}/input/legacy_index_profile.json");

		//SetProperty(Prop_TrackingSystemName_String, "lighthouse");
		//SetBoolProperty(Prop_WillDriftInYaw_Bool, false);
		//SetBoolProperty(Prop_DeviceIsWireless_Bool, true);
		//SetBoolProperty(Prop_DeviceIsCharging_Bool, false);
		////SetInt32Property(Prop_DeviceBatteryPercentage_Float, int32_t(50.0f));

		//SetBoolProperty(Prop_Firmware_UpdateAvailable_Bool, false);
		//SetBoolProperty(Prop_Firmware_ManualUpdate_Bool, false);
		////SetBoolProperty(Prop_DeviceProvidesBatteryStatus_Bool, true);

		//SetBoolProperty(Prop_DeviceCanPowerOff_Bool, true);
		//SetBoolProperty(Prop_Firmware_ForceUpdateRequired_Bool, false);
		//SetBoolProperty(Prop_Identifiable_Bool, true);
		//SetInt32Property(Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
		//SetInt32Property(Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
		//SetProperty(Prop_ModelNumber_String,
		//            (m_role == TrackedControllerRole_LeftHand) ? "Knuckles Left" : "Knuckles Right");

		//SetProperty(Prop_TrackingFirmwareVersion_String,
		//            "1562916277 watchman@ValveBuilder02 2019-07-12 FPGA 538(2.26/10/2) BL 0 VRC 1562916277 Radio 1562882729");
		//SetProperty(Prop_HardwareRevision_String, "product 17 rev 14.1.9 lot 2019/4/20 0");

		//SetBoolProperty(Prop_HasDisplayComponent_Bool, false);
		//SetBoolProperty(Prop_HasCameraComponent_Bool, false);
		//SetBoolProperty(Prop_HasDriverDirectModeComponent_Bool, false);
		//SetBoolProperty(Prop_HasVirtualDisplayComponent_Bool, false);

		//SetInt32Property(Prop_ControllerHandSelectionPriority_Int32, 0);
		//SetProperty(Prop_ConnectedWirelessDongle_String, "C2F75F5986-FYI");

		//SetProperty(Prop_NamedIconPathDeviceReady_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_ready.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_ready.b4bfb144.png");
		//SetProperty(Prop_NamedIconPathDeviceOff_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_off.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_off.b4bfb144.png");
		//SetProperty(Prop_NamedIconPathDeviceSearching_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_searching.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_searching.b4bfb144.png");
		//SetProperty(Prop_NamedIconPathDeviceSearchingAlert_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_searching_alert.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_searching_alert.b4bfb144.png");
		//SetProperty(Prop_NamedIconPathDeviceReadyAlert_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_ready_alert.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_ready_alert.b4bfb144.png");
		//SetProperty(Prop_NamedIconPathDeviceAlertLow_String,
		//            (m_role == TrackedControllerRole_LeftHand)
		//	            ? "{KinectToVR}/icons/left_controller_status_ready_low.b4bfb144.png"
		//	            : "{KinectToVR}/icons/right_controller_status_ready_low.b4bfb144.png");

		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_HardwareRevision_Uint64, 286130441U);
		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_FirmwareVersion_Uint64, 1562916277U);
		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_FPGAVersion_Uint64, 538U);
		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_VRCVersion_Uint64, 1562916277U);
		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_RadioVersion_Uint64, 1562882729U);
		//VRProperties()->SetUint64Property(m_tracked_device_container, Prop_DongleVersion_Uint64, 1558748372U);

		//m_component_handles.resize(m_num_component_definitions);
		//for (uint32_t i = 0; i < m_num_component_definitions; i++)
		//{
		//	const KnuckleComponentDefinition* definition = &m_component_definitions[i];
		//	switch (definition->component_type)
		//	{
		//	case CT_BOOLEAN:
		//		m_component_handles[i] = CreateBooleanComponent(definition->full_path);
		//		break;
		//	case CT_SCALAR:
		//		m_component_handles[i] = CreateScalarComponent(definition->full_path, definition->scalar_type,
		//		                                               definition->scalar_units);
		//		break;
		//	case CT_SKELETON:
		//		m_component_handles[i] = CreateSkeletonComponent(definition->full_path, definition->skeleton_path,
		//		                                                 definition->base_pose_path, nullptr, 0);
		//		break;
		//	case CT_HAPTIC:
		//		m_component_handles[i] = CreateHapticComponent(definition->full_path);
		//		break;
		//	}
		//}

		//setlthumbend(90);
		//setlindexbend(90);
		//setlmiddlebend(0);
		//setlringbend(00);
		//setllittlebend(90);

		//setrthumbend(90);
		//setrindexbend(90);
		//setrmiddlebend(00);
		//setrringbend(00);
		//setrlittlebend(90);

		//m_running = true;
		//m_pose_thread = boost::thread(update_pose_thread, this);
		//m_pose_thread.detach();

		//m_pipeMOF_thread = boost::thread(dlPipeMOF);
		//m_pipeMOF_thread.detach();
		//m_pipeHOF_thread = boost::thread(dlPipeHOF);
		//m_pipeHOF_thread.detach();

		//m_pipeMF_thread = boost::thread(dlPipeMF);
		//m_pipeMF_thread.detach();
		//m_pipeHF_thread = boost::thread(dlPipeHF);
		//m_pipeHF_thread.detach();

		//m_pipeMROT_thread = boost::thread(dlPipeMROT);
		//m_pipeMROT_thread.detach();
		//m_pipeHROT_thread = boost::thread(dlPipeHROT);
		//m_pipeHROT_thread.detach();

		//if (this->m_role == TrackedControllerRole_LeftHand)
		//{
		//	m_pipeHBUT_thread = boost::thread(dlPipeHBUT, this);
		//	m_pipeHBUT_thread.detach();
		//}
		//else if (this->m_role == TrackedControllerRole_RightHand)
		//{
		//	m_pipeMBUT_thread = boost::thread(dlPipeMBUT, this);
		//	m_pipeMBUT_thread.detach();
		//}

		//m_pipeMBUUT_thread = boost::thread(mbutpipe);
		//m_pipeMBUUT_thread.detach();

		//m_pipeHBUUT_thread = boost::thread(hbutpipe);
		//m_pipeHBUUT_thread.detach();

		dn[1] = true;
		mdn[1] = true;

		return VRInitError_None;
	}

	void SoftKnucklesDevice::Deactivate()
	{
		dprintf("Deactivated.  object ID: %d\n", m_id);
		if (m_running)
		{
			m_running = false; // signal to pose thread to shut down
		}
	}

	void SoftKnucklesDevice::Reactivate()
	{
		dprintf("Reactivated.  object ID: %d\n", m_id);
		if (!m_running)
		{
			m_running = true;
		}
	}

	void* SoftKnucklesDevice::GetComponent(const char* pchComponentNameAndVersion)
	{
		// GetComponent will get called for the IVRControllerComponent_001
		dprintf("GetComponent: %s\n");
		return nullptr;
	}

	void SoftKnucklesDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer,
	                                      uint32_t unResponseBufferSize)
	{
		if (m_debug_handler)
		{
			m_debug_handler->DebugRequest(pchRequest, pchResponseBuffer, unResponseBufferSize);
		}
	}

	DriverPose_t SoftKnucklesDevice::GetPose()
	{
		return m_pose;
	}

	VRBoneTransform_t* SoftKnucklesDevice::GetRfPose()
	{
		float rtbend = getrthumbend();
		float ribend = getrindexbend();
		float rmbend = getrmiddlebend();
		float rrbend = getrringbend();
		float rlbend = getrlittlebend();

		transformrightthumb(0);
		transformrightindex(0);
		transformrightmiddle(0);
		transformrightring(0);
		transformrightpinky(0);

		static VRBoneTransform_t right_hand_pose[NUM_BONES] = {
			{{migi[0][0], migi[0][1], migi[0][2], migi[0][3]}, {migi[1][0], migi[1][1], migi[1][2], migi[1][3]}}, //Root
			{{migi[2][0], migi[2][1], migi[2][2], migi[2][3]}, {migi[3][0], migi[3][1], migi[3][2], migi[3][3]}},
			//Wrist
			{{migi[4][0], migi[4][1], migi[4][2], migi[4][3]}, {migi[5][0], migi[5][1], migi[5][2], migi[5][3]}},
			//Thumb0
			{{migi[6][0], migi[6][1], migi[6][2], migi[6][3]}, {migi[7][0], migi[7][1], migi[7][2], migi[7][3]}},
			//Thumb1
			{{migi[8][0], migi[8][1], migi[8][2], migi[8][3]}, {migi[9][0], migi[9][1], migi[9][2], migi[9][3]}},
			//Thumb2
			{
				{migi[10][0], migi[10][1], migi[10][2], migi[10][3]},
				{migi[11][0], migi[11][1], migi[11][2], migi[11][3]}
			}, //Thumb3
			{
				{migi[12][0], migi[12][1], migi[12][2], migi[12][3]},
				{migi[13][0], migi[13][1], migi[13][2], migi[13][3]}
			}, //Index0
			{
				{migi[14][0], migi[14][1], migi[14][2], migi[14][3]},
				{migi[15][0], migi[15][1], migi[15][2], migi[15][3]}
			}, //Index1
			{
				{migi[16][0], migi[16][1], migi[16][2], migi[16][3]},
				{migi[17][0], migi[17][1], migi[17][2], migi[17][3]}
			}, //Index2
			{
				{migi[18][0], migi[18][1], migi[18][2], migi[18][3]},
				{migi[19][0], migi[19][1], migi[19][2], migi[19][3]}
			}, //Index3
			{
				{migi[20][0], migi[20][1], migi[20][2], migi[20][3]},
				{migi[21][0], migi[21][1], migi[21][2], migi[21][3]}
			}, //Index4
			{
				{migi[22][0], migi[22][1], migi[22][2], migi[22][3]},
				{migi[23][0], migi[23][1], migi[23][2], migi[23][3]}
			}, //Middle0
			{
				{migi[24][0], migi[24][1], migi[24][2], migi[24][3]},
				{migi[25][0], migi[25][1], migi[25][2], migi[25][3]}
			}, //Middle1
			{
				{migi[26][0], migi[26][1], migi[26][2], migi[26][3]},
				{migi[27][0], migi[27][1], migi[27][2], migi[27][3]}
			}, //Middle2
			{
				{migi[28][0], migi[28][1], migi[28][2], migi[28][3]},
				{migi[29][0], migi[29][1], migi[29][2], migi[29][3]}
			}, //Middle3
			{
				{migi[30][0], migi[30][1], migi[30][2], migi[30][3]},
				{migi[31][0], migi[31][1], migi[31][2], migi[31][3]}
			}, //Middle4
			{
				{migi[32][0], migi[32][1], migi[32][2], migi[32][3]},
				{migi[33][0], migi[33][1], migi[33][2], migi[33][3]}
			}, //Ring0
			{
				{migi[34][0], migi[34][1], migi[34][2], migi[34][3]},
				{migi[35][0], migi[35][1], migi[35][2], migi[35][3]}
			}, //Ring1
			{
				{migi[36][0], migi[36][1], migi[36][2], migi[36][3]},
				{migi[37][0], migi[37][1], migi[37][2], migi[37][3]}
			}, //Ring2
			{
				{migi[38][0], migi[38][1], migi[38][2], migi[38][3]},
				{migi[39][0], migi[39][1], migi[39][2], migi[39][3]}
			}, //Ring3
			{
				{migi[40][0], migi[40][1], migi[40][2], migi[40][3]},
				{migi[41][0], migi[41][1], migi[41][2], migi[41][3]}
			}, //Ring4
			{
				{migi[42][0], migi[42][1], migi[42][2], migi[42][3]},
				{migi[43][0], migi[43][1], migi[43][2], migi[43][3]}
			}, //Pinky0
			{
				{migi[44][0], migi[44][1], migi[44][2], migi[44][3]},
				{migi[45][0], migi[45][1], migi[45][2], migi[45][3]}
			}, //Pinky1
			{
				{migi[46][0], migi[46][1], migi[46][2], migi[46][3]},
				{migi[47][0], migi[47][1], migi[47][2], migi[47][3]}
			}, //Pinky2
			{
				{migi[48][0], migi[48][1], migi[48][2], migi[48][3]},
				{migi[49][0], migi[49][1], migi[49][2], migi[49][3]}
			}, //Pinky3
			{
				{migi[50][0], migi[50][1], migi[50][2], migi[50][3]},
				{migi[51][0], migi[51][1], migi[51][2], migi[51][3]}
			}, //Pinky4
			{
				{migi[52][0], migi[52][1], migi[52][2], migi[52][3]},
				{migi[53][0], migi[53][1], migi[53][2], migi[53][3]}
			}, //AuxThumb
			{
				{migi[54][0], migi[54][1], migi[54][2], migi[54][3]},
				{migi[55][0], migi[55][1], migi[55][2], migi[55][3]}
			}, //AuxIndex
			{
				{migi[56][0], migi[56][1], migi[56][2], migi[56][3]},
				{migi[57][0], migi[57][1], migi[57][2], migi[57][3]}
			}, //AuxMiddle
			{
				{migi[58][0], migi[58][1], migi[58][2], migi[58][3]},
				{migi[59][0], migi[59][1], migi[59][2], migi[59][3]}
			}, //AuxRing
			{
				{migi[60][0], migi[60][1], migi[60][2], migi[60][3]},
				{migi[61][0], migi[61][1], migi[61][2], migi[61][3]}
			}, //AuxPinky
		};

		return right_hand_pose;
	}

	float SoftKnucklesDevice::GetThumb()
	{
		return ibendt;
	}

	string SoftKnucklesDevice::get_serial() const
	{
		return m_serial_number;
	}
} // end of namespace
