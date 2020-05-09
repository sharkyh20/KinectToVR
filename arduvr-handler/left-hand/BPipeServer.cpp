#include <stdio.h>
#include <iostream>
#include <ctype.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <Windows.h>
#include "SerialPort.h"
#include <string.h>
#include <sstream> 
#include <malloc.h>

using namespace std;

#define RBAUDR CBR_115200
#define MAX_DATA_LENGTH 790
#define ONPRINT
#define RECVVAL
#define ARDUINO
char incomingData[MAX_DATA_LENGTH];
#define MAX_BUF 1024

float th, in, mi, ri, li;
float jx, jy, sy;
float gx, gy, gz;
string port;
char* cmd;
char* cport;
SerialPort* arduino;

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
int nstr(string s, const char* whatc) {
	s = ExtractString(s, whatc, " ,");
	stringstream str_strm;
	str_strm << s;
	string temp_str;
	int temp_int;
	while (!str_strm.eof()) {
		str_strm >> temp_str;
		if (stringstream(temp_str) >> temp_int) {
			return temp_int;
		}
		temp_str = "";
	}
}
int nstrn(string s, const char* whatc) {
	s = ExtractString(s, whatc, "/");
	stringstream str_strm;
	str_strm << s;
	string temp_str;
	int temp_int;
	while (!str_strm.eof()) {
		str_strm >> temp_str;
		if (stringstream(temp_str) >> temp_int) {
			return temp_int;
		}
		temp_str = "";
	}
}

void exampleReceiveData(void)
{
	int readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);
	//printf("%s", incomingData)


#ifdef ONPRINT
	ios::sync_with_stdio(false);

	if (nstr(string(incomingData), "THUMB: ") != 69)cout << "TH: " << nstr(string(incomingData), "THUMB: ") << endl;
	if (nstr(string(incomingData), "INDEX: ") != 69)cout << "IN: " << nstr(string(incomingData), "INDEX: ") << endl;
	if (nstr(string(incomingData), "MIDDLE: ") != 69)cout << "MI: " << nstr(string(incomingData), "MIDDLE: ") << endl;
	if (nstr(string(incomingData), "RING: ") != 69)cout << "RI: " << nstr(string(incomingData), "RING: ") << endl;
	if (nstr(string(incomingData), "LITTLE: ") != 69)cout << "LI: " << nstr(string(incomingData), "LITTLE: ") << endl << endl;

	if (nstr(string(incomingData), "ROTX: ") != 69)cout << "GX: " << nstr(string(incomingData), "ROTX: ") << endl;
	if (nstr(string(incomingData), "ROTY: ") != 69)cout << "GY: " << nstr(string(incomingData), "ROTY: ") << endl;
	if (nstr(string(incomingData), "ROTZ: ") != 69)cout << "GZ: " << nstr(string(incomingData), "ROTZ: ") << endl;

	if (nstr(string(incomingData), "SYS: ") != 69)cout << "SYS: " << nstr(string(incomingData), "SYS: ") << endl;

	if (nstr(string(incomingData), "JOYX: ") != 69)cout << "JX: " << float(nstr(string(incomingData), "JOYX: ")) / 100 << endl;
	if (nstr(string(incomingData), "JOYY: ") != 69)cout << "JY: " << float(nstr(string(incomingData), "JOYY: ")) / 100 << endl << endl;
#endif

#ifdef RECVVAL
	if (nstr(string(incomingData), "THUMB: ") != 69)th = nstr(string(incomingData), "THUMB: ");
	if (nstr(string(incomingData), "INDEX: ") != 69)in = nstr(string(incomingData), "INDEX: ");
	if (nstr(string(incomingData), "MIDDLE: ") != 69)mi = nstr(string(incomingData), "MIDDLE: ");
	if (nstr(string(incomingData), "RING: ") != 69)ri = nstr(string(incomingData), "RING: ");
	if (nstr(string(incomingData), "LITTLE: ") != 69)li = nstr(string(incomingData), "LITTLE: ");

	if (nstr(string(incomingData), "ROTX: ") != 69)gx = nstr(string(incomingData), "ROTX: ") * 10000;
	if (nstr(string(incomingData), "ROTY: ") != 69)gy = nstr(string(incomingData), "ROTY: ") * 10000;
	if (nstr(string(incomingData), "ROTZ: ") != 69)gz = nstr(string(incomingData), "ROTZ: ") * 10000;

	if (nstr(string(incomingData), "SYS: ") != 69)sy = nstr(string(incomingData), "SYS: ");

	if (nstr(string(incomingData), "JOYX: ") != 69)jx = nstr(string(incomingData), "JOYX: ");
	if (nstr(string(incomingData), "JOYY: ") != 69)jy = nstr(string(incomingData), "JOYY: ");
#endif


	Sleep(7);
}

int main(int argc, char* argv[]) {

	if (argc >= 2)
	{
		cout << "Port number has been set by operative console argument via argv string pass.\n";
		cout << "Port is: " << argv[1];
		port = argv[1];
	}
	else {
		cout << "Port for left: ";
		cin >> port;
	}
	stringstream s;
	s << "\\\\.\\" << port;
	string portn;
	portn = s.str();
	const char* cportn(portn.c_str());
	cout << "Left Port: " << portn << endl;
	arduino = new SerialPort(cportn, RBAUDR);
	ios::sync_with_stdio(false);

	for (;;) {
		while (!arduino->isConnected()) {
			Sleep(100);
			arduino = new SerialPort(cportn, RBAUDR);
		}

		if (arduino->isConnected()) {
			std::cout << "Connection STILL established at port " << cportn << endl;
		}

		if (arduino->isConnected()) exampleReceiveData();


		if (true) {
			std::string dat = [&]()->std::string {
				std::stringstream S;
				S << "JX" << jx * 100 << "/JY" << jy * 100 << "/SC" << sy * 10000 << "/TV" << (in <= 6000 && li<=3000 && ri <= 3000 && mi <= 3000 ? (10000 - in) : 0) << "/AC" << 0 * 10000 << "/BC" << 0 * 10000 << "/";
				return S.str();
			}();
			char data[1024];

			strcpy(data, dat.c_str());

			HANDLE hPipe = CreateFile("\\\\.\\pipe\\LogPipeNiButton",
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);
			if (hPipe != INVALID_HANDLE_VALUE) {

				WriteFile(hPipe,
					data,
					sizeof(data),
					0,
					NULL);

				CloseHandle(hPipe);
			}
			else {
				printf("Error: %d", GetLastError());

			}
		}
		if (true) {
			std::string dat = [&]()->std::string {
				std::stringstream S;
				S << "X" << (int)-gy << "/Y" << (int)gx << "/Z" << (int)-gz << "/";
				return S.str();
			}();
			char data[1024];

			strcpy(data, dat.c_str());

			HANDLE hPipe = CreateFile("\\\\.\\pipe\\LogPipeNiRot",
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);
			if (hPipe != INVALID_HANDLE_VALUE) {

				WriteFile(hPipe,
					data,
					sizeof(data),
					0,
					NULL);

				CloseHandle(hPipe);
			}
			else {
				printf("Error: %d", GetLastError());

			}
		}
		if (true) {
			std::string dat = [&]()->std::string {
				std::stringstream S;
				S << "T" << 100 * th << "/I" << 100 * in << "/M" << 100 * mi << "/P" << 100 * ri << "/L" << 100 * li << "/";
				return S.str();
			}();
			char data[1024];

			strcpy(data, dat.c_str());

			HANDLE hPipe = CreateFile("\\\\.\\pipe\\LogPipeNiFinger",
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);
			if (hPipe != INVALID_HANDLE_VALUE) {

				WriteFile(hPipe,
					data,
					sizeof(data),
					0,
					NULL);

				CloseHandle(hPipe);
			}
			else {
				printf("Error: %d", GetLastError());

			}
		}

		this_thread::sleep_for(chrono::milliseconds(50));
	}
}








