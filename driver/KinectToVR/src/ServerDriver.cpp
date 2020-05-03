#include "ServerDriver.hpp"
#include <iostream>
#include <thread>
#include <functional>
#include <Windows.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <string.h>
#include <exception>
#include <Windows.h>
#include <atlbase.h>
#include <codecvt>

//#include <boost/property_tree/json_parser.hpp>
//#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
//#include <boost/filesystem.hpp>
//namespace pt = boost::property_tree;

#pragma comment(lib, "Ws2_32.lib")
//EXTERN_C IMAGE_DOS_HEADER __ImageBase;
//
//static bool autostart = false;
//static int autosver;
static const char* listen_address = "127.0.0.1";
static const unsigned short listen_port = 5741;
bool initialised = false;
#define NUM_DEVICES 1

ServerDriver* ServerDriver::_instance = nullptr;
FakeTracker *trp, *trm, *trh;

ServerDriver::ServerDriver()
{
}

ServerDriver* ServerDriver::get()
{
	if (_instance == nullptr)
		_instance = new ServerDriver();
	return _instance;
}

void init(ServerDriver* pthis) {

    WSADATA WSAData;

    SOCKET server, client;

    SOCKADDR_IN serverAddr, clientAddr;

    WSAStartup(MAKEWORD(2, 0), &WSAData);
    server = socket(AF_INET, SOCK_STREAM, 0);

    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5782);

    bind(server, (SOCKADDR*)&serverAddr, sizeof(serverAddr));
    listen(server, 0);

    char buffer[1024];
    int clientAddrSize = sizeof(clientAddr);
    if ((client = accept(server, (SOCKADDR*)&clientAddr, &clientAddrSize)) != INVALID_SOCKET)
    {

		vr::VRServerDriverHost()->TrackedDeviceAdded(trp->get_serial().c_str(), vr::TrackedDeviceClass_GenericTracker, trp);
		vr::VRServerDriverHost()->TrackedDeviceAdded(trm->get_serial().c_str(), vr::TrackedDeviceClass_GenericTracker, trm);
		vr::VRServerDriverHost()->TrackedDeviceAdded(trh->get_serial().c_str(), vr::TrackedDeviceClass_GenericTracker, trh);

		initialised = true;

        recv(client, buffer, sizeof(buffer), 0);
        memset(buffer, 0, sizeof(buffer));
        closesocket(client);
    }
}

//std::wstring s2ws(const std::string& s)
//{
//	int len;
//	int slength = (int)s.length() + 1;
//	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
//	wchar_t* buf = new wchar_t[len];
//	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
//	std::wstring r(buf);
//	delete[] buf;
//	return r;
//}
//
//void eraseSubStr(std::string& mainStr, const std::string& toErase)
//{
//	size_t pos = std::string::npos;
//
//	// Search for the substring in string in a loop untill nothing is found
//	while ((pos = mainStr.find(toErase)) != std::string::npos)
//	{
//		// If found then erase it from string
//		mainStr.erase(pos, toErase.length());
//	}
//}

vr::EVRInitError ServerDriver::Init(vr::IVRDriverContext * driver_context)
{
	if (vr::EVRInitError init_error = vr::InitServerDriverContext(driver_context); init_error != vr::EVRInitError::VRInitError_None) {
		return init_error;
	}

	/*LPTSTR  strDLLPath = new TCHAR[_MAX_PATH];
	::GetModuleFileName((HINSTANCE)&__ImageBase, strDLLPath, _MAX_PATH);
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> conv;

	std::string PATH = conv.to_bytes(std::wstring(strDLLPath));
	eraseSubStr(PATH, "driver_k2vr\\bin\\win64\\driver_KinectToVR.dll");

	pt::ptree root;
	pt::read_json(std::string("\"" + PATH + "ConfigSettings.json\""), root);

	for (pt::ptree::value_type& v : root.get_child("settings"))
	{
		if (v.first == "astartk") {
			autostart = boost::lexical_cast<bool>(v.second.data());
		}
		if (v.first == "astartver") {
			autosver = boost::lexical_cast<int>(v.second.data());
		}
	}

	if (autostart) {
		if (autosver == 1) {
			ShellExecute(NULL, NULL, s2ws(std::string(PATH + "KinectV2Process.exe")).c_str(), NULL, NULL, SW_SHOW);
		}
		else if (autosver == 1) {
			ShellExecute(NULL, NULL, s2ws(std::string(PATH + "KinectV1Process.exe")).c_str(), NULL, NULL, SW_SHOW);
		}
	}*/

	trh = new FakeTracker("RFOOT");
	trm = new FakeTracker("LFOOT");
	trp = new FakeTracker("HIP");

	std::thread* t = new std::thread(init, this);

	return vr::EVRInitError::VRInitError_None;
}

void ServerDriver::Cleanup()
{
}

const char * const * ServerDriver::GetInterfaceVersions()
{
	return vr::k_InterfaceVersions;;
}

void ServerDriver::RunFrame()
{
	for (auto& tracker : _trackers) {
		tracker->update();
	}

	if (initialised) {
		trp->update();
		trh->update();
		trm->update();
	}

	vr::VREvent_t event;
	while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
		for (auto& tracker : _trackers) {
			if (tracker->get_index() == event.trackedDeviceIndex)
				tracker->process_event(event);
		}
	}

}

bool ServerDriver::ShouldBlockStandbyMode()
{
	return false;
}

void ServerDriver::EnterStandby()
{
}

void ServerDriver::LeaveStandby()
{
}