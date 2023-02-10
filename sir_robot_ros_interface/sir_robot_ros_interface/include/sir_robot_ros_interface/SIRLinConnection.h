#define SIRLINUX
#ifdef SIRLINUX

#ifndef SIRLINCONNECTION_H
#define SIRLINCONNECTION_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <fcntl.h>
#include "include/SIRRobot/SIRConnection.h"
#include "include/SIRLogger.h"
using namespace std;

//! SIR Linux OS connection class
/*!
  This class is used to connect and disconnect to the robot, receive and send data
*/
class SIRLinConnection:public SIRConnection {
public:
  /** Constructor
  * @param _port Port number of the connection
  * @param _ip ip number of the connection
  * @param bufSize required buffer size for receive messages.
  */
  SIRLinConnection(SIRLogger *log, string _ip = "127.0.0.1", unsigned int _port = 7777, int bufSize = 1024);
  ~SIRLinConnection(void);


  /** this function is used to connection of the robot
  * @return the result of the connection process (true: connected, false: connection error)
  */
  bool Connect();


  /** this function is used to disconnection of the robot
  * @return the result of the disconnection process (true: connected, false: connection error)
  */
  bool disconnect();


  /** this function is used to send a string
  * @param str shows the string that will be sent
  * @return the number of characters that has been sent (if there is a error, returns -1)
  */
  int Send(const string& str);

  /**  this function is used to receive a string
  * @param str shows the string that will be received
  * @return the number of characters that has been received (if there is a error, returns -1)
  */
  int Receive(string & str, unsigned int msWait=0);

  /**  this function enable to select bocking or unblocking mode (Default: non-blocking mode)
  * @param mode connection mode (If mode = 0, blocking is enabled; If mode != 0, non-blocking mode is enabled.)
  * @return the error state, true means the mode is set, false value means error
  */
  bool setBlockingMode(int mode);

  /** this function is used to define a callback function for getting positions asynchronously.
  * @param funcCB pointer for thtide callback function
  * @return true if success, false otherwise
  */
  bool registerCB(void (*funcCB)(string));

  /** this function is used to start to wait events to call callback function asynchronously.
  * @param eventType the tyep of the event (0 for READ, 1 for WRITE)
  * @return true if success, false otherwise
  */
  bool startCB(SIRConnection *con, int eventType);

  /** this function is used to stop to wait events to call callback function asynchronously.
  * @return true if success, false otherwise
  */
  bool stopCB();

private:
  /**
  * Indicates the information about the linux Sockets implementation
  */
  struct sockaddr_in server_addr;
  struct hostent *server;
  //WSADATA  wsaData;

  /**
  * Indicates the creating socket
  */
  static int mySocket;

  /**
  * Indicates bytes to be sent/receive
  */
  int myBytes;

  /**
  * Indicates receiver buffer with an arranged array bound and initialized as char "0"
  */
  //
  char *buffer;

  /**
  * Indicates required buffer size for received packets"
  */
  //
  int bufferSize;

  /**
  * Used to call callback function when read or write event on the socket becomes
  */
  //
  //static WSAEVENT  rwEvent;

  /**
  * Used for thread which calls the callback function when an read or write event is signalled.
  */
  // HANDLE threadCB;
  pthread_t threadCB;

  //static DWORD WINAPI threadCBProc(LPVOID);
  static void* threadCBProc(void *);

  /**
  * Used for stopping callback function.
  */
  static bool isCallbackActive;
  static int eventType;
};


#endif // SIRLINCONNECTION_H

#endif
