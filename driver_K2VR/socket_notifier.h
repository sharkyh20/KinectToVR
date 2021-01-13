//////////////////////////////////////////////////////////////////////////////
// socket_listener.h
//
// call Notify if a connection is made on listen_address and listen_port
// 
// intended as a signal to the device
#pragma once

class SocketNotifierImpl;

class SocketNotifier
{
public:
	SocketNotifier();
	virtual ~SocketNotifier();

	void StartListening(const char* listen_address, unsigned short listen_port);
	void StopListening();
	virtual void Notify() = 0;

private:
	SocketNotifierImpl* m_impl;
};
