#define SIRLINUX
#ifdef SIRLINUX

#include "SIRLinConnection.h"

#include<iostream>
using namespace std;

//WSAEVENT  SIRWinConnection::rwEvent=0;
int SIRLinConnection::mySocket=-1;
bool SIRLinConnection::isCallbackActive=false;
int SIRLinConnection::eventType=0;

SIRLinConnection::SIRLinConnection(SIRLogger *log, string _ip, unsigned int _port, int bufSize) :SIRConnection(log,_ip,_port), bufferSize(bufSize)
{
  buffer = new char[bufferSize];
}

SIRLinConnection::~SIRLinConnection(void)
{
  delete [] buffer;
  // Close the socket
  close(mySocket);
  mySocket=-1;
}

bool SIRLinConnection::Connect() {

  /**
  * Indicates the Server/client address, respectively
  */
  struct sockaddr_in ServerAddr;

  // Initialize Winsock version 2.2
  //  WSAStartup(MAKEWORD(2, 2), &wsaData);
  //  (*logger)<<SIRLOGTYPE::LOG_INFO
  //    <<"SIRWinConnection::Connect, Client: Winsock DLL status is"<<wsaData.szSystemStatus;

  // Create a new socket to make a client connection.
  // AF_INET = 2, The Internet Protocol version 4 (IPv4) address family, TCP protocol
  mySocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (mySocket == -1) {
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRLinConnection::Connect, socket() failed! Error code:" << to_string(errno);
    // Exit with error
    return false;
  }


  // Set up a SOCKADDR_IN structure that will be used to connect
  // to a listening server on port.

  // IPv4
  ServerAddr.sin_family = AF_INET;
  // Port no.
  ServerAddr.sin_port = htons(static_cast<uint16_t>(port));
  // The IP address
  if(inet_aton(ip.c_str(),&ServerAddr.sin_addr)==0){
      (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRLinConnection::inet_aton,  failed! Error code:" << to_string(errno);
      // Close the socket
      close(mySocket);
     // Exit with error
      return false;
  }
  //ServerAddr.sin_addr.s_addr = inet_addr(ip.c_str());

  // Make a connection to the server with mySocket.
  int retCode = connect(mySocket, (const struct sockaddr *)&ServerAddr, sizeof(struct sockaddr));
  if (retCode != 0) {
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRLinConnection::Connect, connect() failed! Error code:" << to_string(errno);
    // Close the socket
    close(mySocket);
    // Exit with error
    return false;
  }

  setBlockingMode(0); //set non-blocking mode




  return true;
}

bool SIRLinConnection::disconnect()
{
  bool ret;
  // When you are finished sending and receiving data on socket SendingSocket,
  // you should close the socket using the closesocket API. We will
  // describe socket closure later in the chapter.
  if (close(mySocket) != 0) {
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRLinConnection::disconnect, Cannot close \"SendingSocket\" socket. Error code:" << to_string(errno);
    ret = false;
  }
  else {
    (*logger) << SIRLOGTYPE::LOG_INFO
      << "SIRLinConnection::disconnect, Closing \"SendingSocket\" socket...";
    ret = true;
  }
  return ret;
}

int SIRLinConnection::Send(const string& str)
{
  // Sends some data to server/receiver...
  myBytes = send(mySocket, str.c_str(), strlen(str.c_str()), 0);

  if (myBytes < 0) {
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRWinConnection::Send, send error:" << to_string(errno);
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

int SIRLinConnection::Receive(string &str, unsigned int msWait)
{
  //char buff[1024];

  if (msWait != 0)
  {
    struct timeval tval;
    fd_set fdSet;
    tval.tv_sec = msWait / 1000;
    tval.tv_usec = (msWait % 1000) * 1000;
    FD_ZERO(&fdSet);
    FD_SET(mySocket, &fdSet);
    if (select(mySocket+1, &fdSet, NULL, NULL, &tval) <= 0)
      return -1;
//    tval.tv_sec = 0;
//    tval.tv_usec = 5000;
//    select(0, NULL, NULL, NULL, &tval);

  }


  // Receive some data from server
  // strlen(recvbuf) can be written as the third variable
  myBytes = recv(mySocket, buffer, bufferSize, 0);
  //myBytes = read(mySocket, buffer, strlen(buffer));
  if (myBytes < 0) {
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRWinConnection::Receive, recv() error.:" << to_string(errno);
    return -1;
  }

  str.clear();
  str.append(buffer, myBytes);
  return myBytes;
}

bool SIRLinConnection::setBlockingMode(int mode)
{
  //u_long imode = (u_long)mode;
  //-------------------------
  // Set the socket I/O mode: In this case FIONBIO
  // enables or disables the blocking mode for the
  // socket based on the numerical value of iMode.
  // If iMode = 0, blocking is enabled;
  // If iMode != 0, non-blocking mode is enabled.

//  if (ioctl(mySocket, FIONBIO, &imode) == -1) {
//    (*logger) << SIRLOGTYPE::LOG_ERROR
//      << "SIRWinConnection::setBlockingMode, ioctlsocket failed with error: not changed blocking mode";
//    return false;
//  }
//  return true;
  int flags = fcntl(mySocket, F_GETFL, nullptr);
  if (flags == -1){
    (*logger) << SIRLOGTYPE::LOG_ERROR
      << "SIRLinConnection::setBlockingMode, failed with error: get error"<<to_string(errno);
    return false;
  }

  if(mode==0){//blocking
    flags &= ~O_NONBLOCK;
  }
  else {
    flags |= O_NONBLOCK;
  }

  if (fcntl(mySocket, F_SETFL, flags) == -1) {
      (*logger) << SIRLOGTYPE::LOG_ERROR
        << "SIRLinConnection::setBlockingMode, failed with error: set error"<<to_string(errno);
      return false;
  }
  else {
     return true;
  }
}

bool SIRLinConnection::registerCB(void(*funcCB)(string))
{
  if (funcCB != nullptr) {
    functionCB = funcCB;
    return true;
  }
  else {
    return false;
  }
}

bool SIRLinConnection::startCB(SIRConnection *con, int eventType)
{
  this->eventType=eventType;
  if (functionCB != nullptr){
    // Create thread
    pthread_create(&threadCB, nullptr, &threadCBProc, (void*)con);

    if (threadCB != 0){
      (*logger) << SIRLOGTYPE::LOG_ERROR
        << "SIRWinConnection::startCB, CreateThread failed:" << to_string(errno);
      return false;
    }
    isCallbackActive=true;
    // Activate events


    //rwEvent = WSACreateEvent();
//    if (eventType == 0) {
//      if (WSAEventSelect(mySocket, rwEvent, FD_READ) == SOCKET_ERROR) {
//        (*logger) << SIRLOGTYPE::LOG_ERROR
//          << "SIRWinConnection::startCB, WSAEventSelect() error:" << to_string(WSAGetLastError());
//        TerminateThread(threadCB, 0);
//        return false;
//      }
//    }
//    else{
//      if (WSAEventSelect(mySocket, rwEvent, FD_WRITE) == SOCKET_ERROR) {
//        (*logger) << SIRLOGTYPE::LOG_ERROR
//          << "SIRWinConnection::startCB, WSAEventSelect() error:" << to_string(WSAGetLastError());
//        TerminateThread(threadCB, 0);
//        return false;
//      }
//    }
    return true;
  }
  else {
    return false;
  }
}

bool SIRLinConnection::stopCB()
{
//  // Destroy event
//  if (!WSACloseEvent(rwEvent)) {
//    (*logger) << SIRLOGTYPE::LOG_ERROR
//      << "SIRWinConnection::stopCB, WSACloseEvent() error:" << to_string(WSAGetLastError());
//    TerminateThread(threadCB, 0);
//    return false;
//  }
  // Destroy thread
  //TerminateThread(threadCB, 0);
  isCallbackActive=false;
  return true;
}

void* SIRLinConnection::threadCBProc(void *_con)
{
  SIRConnection *con= (SIRConnection *)_con;
  fd_set readfds, writefds, exceptfds;
  int nfds;
  FD_ZERO(&readfds);
  FD_ZERO(&writefds);
  FD_ZERO(&exceptfds);
  if(eventType==0)
    FD_SET(mySocket, &readfds);
  else if(eventType==1)
    FD_SET(mySocket, &writefds);
  else {
    FD_SET(mySocket, &readfds);
    FD_SET(mySocket, &writefds);
  }
  nfds = max(nfds, mySocket);

  while (isCallbackActive) {
    //WSAWaitForMultipleEvents(1, &rwEvent, false, WSA_INFINITE, true);
    if(select(nfds+1,&readfds, &writefds, &exceptfds, nullptr)>0){
      //Read message and call the callback function
      string msg;
      con->Receive(msg,0);
      (*(con->functionCB))(msg);
    }
  }

}

#endif
