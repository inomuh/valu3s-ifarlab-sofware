#ifndef SIRLOGGER_H
#define SIRLOGGER_H

#include <fstream>
#include <string>
using namespace std;

//! Defines type of the logging 
/*! This is used to indicate the type of the log message*/
enum class SIRLOGTYPE {
	LOG_NONE=0,				/*!< type of the log message is undefined.	*/
	LOG_ERROR=1,			/*!< type of the log message is error.	*/
	LOG_WARNING=2,			/*!< type of the log message is warning.	*/
	LOG_INFO=3,				/*!< type of the log message is info.	*/
	LOG_OTHER=4				/*!< type of the log message is other than defined ones.	*/
};

//! Defines log level 
/*! These levels defines which type of log messages are reported.*/
enum class SIRLOGLEVEL {
	LOGLEVEL_NOLOG=0,		/*!< No log is reported.	*/
	LOGLEVEL_ERROR=1,		/*!< just error messages are reported.*/
	LOGLEVEL_WARN=2,		/*!< just error and warnings messages are reported.	*/
	LOGLEVEL_INFO=3,		/*!< error, warnings, and info messages are reported.	*/
	LOGLEVEL_DEBUG=4		/*!< all of the messages are reported.	*/
};

//! Defines where to send log messages.
/*! The mesasages may be send to a file or standard output (screen).*/
enum class SIRLOGTO {
	LOGTOSTDOUT=0,			/*!< send the messages to the standart output*/
	LOGTOFILE=1				/*!< send the messages to a file*/
};

//! A logger class
/*!
This class is used to process log messages.
*/
class SIRLogger{
	friend SIRLogger &operator << (SIRLogger &logger, const SIRLOGTYPE logType);
	friend SIRLogger &operator << (SIRLogger &logger, const char *text);
	friend SIRLogger &operator << (SIRLogger &logger, const string text);
private:
	/**
	* a file object used to write logs to a file 
	*/
	ofstream logFile;
	/**
	* the number of warnings
	*/
	unsigned int numWarnings;
	/**
	* the number of errors
	*/
	unsigned int numErrors;
	/**
	* log level
	*/
	SIRLOGLEVEL logLevel;
	/**
	* Indicates to where the log messages are forwarded. 
	*/
	SIRLOGTO logTo;
	/**
	* a file name used for writing log messages
	*/
	string fileName;
	/**
	* type of the log message
	*/
	SIRLOGTYPE logType;
public:
	/** constructor
	* @param fileName file name to write the log messages
	* @param loglevel log level
	* @param logto indicates where to write log messages
	*/
	explicit SIRLogger(string filename="log.txt",SIRLOGLEVEL loglevel=SIRLOGLEVEL::LOGLEVEL_NOLOG
		,SIRLOGTO logto= SIRLOGTO::LOGTOSTDOUT);
	~SIRLogger();

	/** Sets the log level
	* @param loglevel  log level
	*/
	void setLogLevel(SIRLOGLEVEL loglevel);
	/** gets the log level
	* @return returns the current log level
	*/
	SIRLOGLEVEL getLogLevel() const;
	/** defines where to send log messages 
	* @param logto place to send the log messages (to a file or standart output)
	*/
	void setLogTo(SIRLOGTO logto);
	/** gets the info about where to send log messages
	* @return place to send the log messages (to a file or standart output)
	*/
	SIRLOGTO getLogTo() const;
	/** clears the number of warnings (sets to zero)
	* 
	*/
	void clearWarnings();
	/** clears the number of errors (sets to zero)
	*
	*/
	void clearErrors();
};

#endif