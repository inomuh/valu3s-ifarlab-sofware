//#define SIRWINDOWS
#ifdef SIRWINDOWS


#include "SIRWinConnection.h"
#include<iostream>
using namespace std;

WSAEVENT  SIRWinConnection::rwEvent=0;

SIRWinConnection::SIRWinConnection(SIRLogger *log, string _ip, unsigned int _port, int bufSize) :SIRConnection(log,_ip,_port), bufferSize(bufSize)
{
	buffer = new char[bufferSize];
}

SIRWinConnection::~SIRWinConnection(void)
{
	delete [] buffer;
	// Close the socket
	closesocket(mySocket);
	// Do the clean up
	WSACleanup();
}

bool SIRWinConnection::Connect() {

	/**
	* Indicates the Server/client address, respectively
	*/
	SOCKADDR_IN ServerAddr;

	// Initialize Winsock version 2.2
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	(*logger)<<SIRLOGTYPE::LOG_INFO
		<<"SIRWinConnection::Connect, Client: Winsock DLL status is"<<wsaData.szSystemStatus;

	// Create a new socket to make a client connection.
	// AF_INET = 2, The Internet Protocol version 4 (IPv4) address family, TCP protocol
	mySocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (mySocket == INVALID_SOCKET) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::Connect, socket() failed! Error code:" << to_string(WSAGetLastError());
		// Do the clean up
		WSACleanup();
		// Exit with error
		return false;
	}

	// Set up a SOCKADDR_IN structure that will be used to connect
	// to a listening server on port. 

	// IPv4
	ServerAddr.sin_family = AF_INET;
	// Port no.
	ServerAddr.sin_port = htons(port);
	// The IP address
	ServerAddr.sin_addr.s_addr = inet_addr(ip.c_str());

	// Make a connection to the server with mySocket.
	int retCode = connect(mySocket, (SOCKADDR *)&ServerAddr, sizeof(ServerAddr));
	if (retCode != 0) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::Connect, connect() failed! Error code:" << to_string(WSAGetLastError());
		// Close the socket
		closesocket(mySocket);
		// Do the clean up
		WSACleanup();
		// Exit with error
		return false;
	}

	setBlockingMode(1); //set non-blocking mode

	return true;
}

bool SIRWinConnection::disconnect()
{
	bool ret;
	// When you are finished sending and receiving data on socket SendingSocket,
	// you should close the socket using the closesocket API. We will
	// describe socket closure later in the chapter.
	if (closesocket(mySocket) != 0) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::disconnect, Cannot close \"SendingSocket\" socket. Error code:" << to_string(WSAGetLastError());
		ret = false;
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_INFO
			<< "SIRWinConnection::disconnect, Closing \"SendingSocket\" socket...";
		ret = true;
	}
	// When your application is finished handling the connection, call WSACleanup.
	if (WSACleanup() != 0) {
		(*logger) << SIRLOGTYPE::LOG_INFO
			<< "SIRWinConnection::disconnect, WSACleanup() failed!...";
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_INFO
			<< "SIRWinConnection::disconnect, WSACleanup() is OK...";
	}

	return ret;
}

int SIRWinConnection::Send(const string& str)
{
	// Sends some data to server/receiver...
	myBytes = send(mySocket, str.c_str(), strlen(str.c_str()), 0);

	if (myBytes == SOCKET_ERROR) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::Send, send error:" << to_string(WSAGetLastError());
		return -1;
	}
	else if (myBytes != strlen(str.c_str())) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::could not send all the characters in the packet.:";
		return myBytes;
	}
	else {
		return myBytes;
	}
}

int SIRWinConnection::Receive(string &str, unsigned int msWait)
{
	if (msWait != 0)
	{
		struct timeval tval;
		fd_set fdSet;
		tval.tv_sec = msWait / 1000;
		tval.tv_usec = (msWait % 1000) * 1000;
		FD_ZERO(&fdSet);
		FD_SET(mySocket, &fdSet);
		if (select(0, &fdSet, NULL, NULL, &tval) <= 0)
			return -1;
	}
	// Receive some data from server
	// strlen(recvbuf) can be written as the third variable
	myBytes = recv(mySocket, buffer, strlen(buffer), 0);

	if (myBytes == SOCKET_ERROR) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::Receive, recv() error.:" << to_string(WSAGetLastError());
		return -1;
	}

	str.clear();
	str.append(buffer, myBytes);
	return myBytes;
}

bool SIRWinConnection::setBlockingMode(int mode)
{
	u_long imode = (u_long)mode;
	//-------------------------
	// Set the socket I/O mode: In this case FIONBIO
	// enables or disables the blocking mode for the 
	// socket based on the numerical value of iMode.
	// If iMode = 0, blocking is enabled; 
	// If iMode != 0, non-blocking mode is enabled.

	if (ioctlsocket(mySocket, FIONBIO, &imode) != NO_ERROR) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::setBlockingMode, ioctlsocket failed with error: not changed blocking mode";
		return false;
	}
	return true;
}

bool SIRWinConnection::registerCB(void(*funcCB)(string))
{
	if (funcCB != NULL) {
		functionCB = funcCB;
		return true;
	}
	else {
		return false;
	}
}

bool SIRWinConnection::startCB(SIRConnection *con, int eventType)
{
	if (functionCB != NULL){
		// Create thread
		DWORD dwThreadID;
		threadCB = CreateThread(
			NULL,								// default security
			0,									// default stack size
			&threadCBProc,					// name of the thread function
			(LPVOID)con,								// no thread parameters
			0,									// default startup flags
			&dwThreadID);

		if (threadCB == NULL){
			(*logger) << SIRLOGTYPE::LOG_ERROR
				<< "SIRWinConnection::startCB, CreateThread failed:" << to_string(GetLastError());
			return false;
		}
		// Activate events   
		rwEvent = WSACreateEvent();
		if (eventType == 0) {
			if (WSAEventSelect(mySocket, rwEvent, FD_READ) == SOCKET_ERROR) {
				(*logger) << SIRLOGTYPE::LOG_ERROR
					<< "SIRWinConnection::startCB, WSAEventSelect() error:" << to_string(WSAGetLastError());
				TerminateThread(threadCB, 0);
				return false;
			}
		}
		else{
			if (WSAEventSelect(mySocket, rwEvent, FD_WRITE) == SOCKET_ERROR) {
				(*logger) << SIRLOGTYPE::LOG_ERROR
					<< "SIRWinConnection::startCB, WSAEventSelect() error:" << to_string(WSAGetLastError());
				TerminateThread(threadCB, 0);
				return false;
			}
		}
		return true;
	}
	else {
		return false;
	}
}
bool SIRWinConnection::stopCB()
{
	// Destroy event
	if (!WSACloseEvent(rwEvent)) {
		(*logger) << SIRLOGTYPE::LOG_ERROR
			<< "SIRWinConnection::stopCB, WSACloseEvent() error:" << to_string(WSAGetLastError());
		TerminateThread(threadCB, 0);
		return false;
	}
	// Destroy thread
	TerminateThread(threadCB, 0);
	return true;
}

DWORD WINAPI SIRWinConnection::threadCBProc(LPVOID lpParam)
{
	SIRConnection *con= (SIRConnection *)lpParam;

	while (1) {
		WSAWaitForMultipleEvents(1, &rwEvent, false, WSA_INFINITE, true);

		if (WSAResetEvent(rwEvent) == false) {
		}
		//Read message and call the callback function 
		string msg;
		con->Receive(msg,0);
		(*(con->functionCB))(msg);
	}

}

#endif
