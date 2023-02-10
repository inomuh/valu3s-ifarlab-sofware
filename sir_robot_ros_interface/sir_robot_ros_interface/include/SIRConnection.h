#ifndef SIRCONNECTION_H
#define SIRCONNECTION_H
#include <string>
#include "SIRLogger.h"
using namespace std;

//! SIR connection class
/*!
	This class is used to connect and disconnect to the robot, receive and send data
*/
class SIRConnection{
public:
    /** Constructor
	* @param _port Port number of the connection
	* @param _ip ip number of the connection
	* @param bufSize required buffer size for receive messages.
	*/
	SIRConnection(SIRLogger *log, string _ip= "192.168.5.141", unsigned int _port=11111);
  virtual ~SIRConnection(void) = 0;


	/** this function is used to connection of the robot
	* @return the result of the connection process (true: connected, false: connection error)
	*/
	virtual bool Connect()=0;


	/** this function is used to disconnection of the robot
	* @return the result of the disconnection process (true: connected, false: connection error)
	*/
	virtual bool disconnect()=0;


	/** this function is used to send a string 
	* @param str shows the string that will be sent
	* @return the number of characters that has been sent
	*/
	virtual int Send(const string& str)=0;

	/**  this function is used to receive a string
	* @param str shows the string that will be received
	* @return the number of characters that has been received
	*/
	virtual int Receive(string & str,unsigned int msWait)=0;

	/**  this function enable to select bocking or unblocking mode (Default: non-blocking mode)
	* @param mode connection mode (If mode = 0, blocking is enabled; If mode != 0, non-blocking mode is enabled.)
	* @return the error state, 0 means the mode is set, other value means error
	*/
	virtual bool setBlockingMode(int mode)=0;

	/** this function is used to define a callback function for getting positions asynchronously.
	* @param funcCB pointer for the callback function
	* @return true if success, false otherwise
	*/
	virtual bool registerCB(void(*funcCB)(string))=0;

	/** this function is used to start to wait events to call callback function asynchronously.
	* @param eventType the tyep of the event (0 for READ, 1 for WRITE)
	* @return true if success, false otherwise
	*/
	virtual bool startCB(SIRConnection *con, int eventType=0) = 0;

	/** this function is used to stop to wait events to call callback function asynchronously.
	* @return true if success, false otherwise
	*/
	virtual bool stopCB() = 0;

protected:
	/**
	* Indicates the ip of robot
	*/
	string ip;

	/**
	* Indicates the port of robot
	*/
	unsigned int port;

	SIRLogger *logger;
public:
	void (*functionCB)(string);

};

#endif

