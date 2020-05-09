/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/


#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 790

#include <windows.h>
#include <iostream>
#include <stdlib.h>

class SerialPort
{
private:
    HANDLE handler;
    bool connected;
    COMSTAT status;
    DWORD errors;
public:
    SerialPort(const char *portName, const int baudrate);
    ~SerialPort();

    int readSerialPort(char *buffer, unsigned int buf_size);
    bool writeSerialPort(char *buffer, unsigned int buf_size);
    bool isConnected();
    void closeSerial();
};

#endif // SERIALPORT_H
