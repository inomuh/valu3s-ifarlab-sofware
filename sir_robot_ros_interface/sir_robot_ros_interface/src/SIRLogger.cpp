#include "SIRLogger.h"
#include <iostream>
using namespace std;


SIRLogger::SIRLogger(string filename, SIRLOGLEVEL loglevel, SIRLOGTO logto)
	:fileName(filename),logLevel(loglevel),logTo(logto),numWarnings(0),numErrors(0),logType(SIRLOGTYPE::LOG_NONE)
{
	logFile.open(fileName.c_str());

	// Write the first lines
	if (logFile.is_open()) {
		switch (logLevel) {
		case SIRLOGLEVEL::LOGLEVEL_NOLOG:
			(*this) << SIRLOGTYPE::LOG_INFO << "Log level: NO LOG";
			break;
		case SIRLOGLEVEL::LOGLEVEL_ERROR:
			(*this) << SIRLOGTYPE::LOG_INFO << "Log level: ERROR";
			break;
		case SIRLOGLEVEL::LOGLEVEL_WARN:
			(*this) << SIRLOGTYPE::LOG_INFO << "Log level: WARNING";
			break;
		case SIRLOGLEVEL::LOGLEVEL_INFO:
			(*this) << SIRLOGTYPE::LOG_INFO << "Log level: INFO";
			break;
		case SIRLOGLEVEL::LOGLEVEL_DEBUG:
			(*this) << SIRLOGTYPE::LOG_INFO << "Log level: DEBUG";
			break;
		default:
			break;
		}
		(*this) << SIRLOGTYPE::LOG_INFO << "Log file created";
	} // if
}


SIRLogger::~SIRLogger()
{
	if (logFile.is_open()) {
		// Report number of errors and warnings
		(*this) << to_string(numWarnings) << " warnings \n";
		(*this) << to_string(numErrors) << " errors \n";
		logFile.close();
	} // if
}

void SIRLogger::setLogLevel(SIRLOGLEVEL loglevel)
{
	if(loglevel>=SIRLOGLEVEL::LOGLEVEL_NOLOG && loglevel<=SIRLOGLEVEL::LOGLEVEL_DEBUG)
		logLevel = loglevel;
}

SIRLOGLEVEL SIRLogger::getLogLevel() const
{
	return logLevel;
}

void SIRLogger::setLogTo(SIRLOGTO logto)
{
	if (logto == SIRLOGTO::LOGTOSTDOUT || logto == SIRLOGTO::LOGTOFILE)
		logTo = logto;

}
SIRLOGTO SIRLogger::getLogTo() const
{
	return logTo;
}

void SIRLogger::clearWarnings()
{
	numWarnings = 0;
}
void SIRLogger::clearErrors()
{
	numErrors = 0;
}

// Overload << operator using log type
SIRLogger &operator << (SIRLogger &logger, const SIRLOGTYPE logtype)
{
	if (logger.logType != SIRLOGTYPE::LOG_NONE) {
		if (logger.logTo == SIRLOGTO::LOGTOFILE)
			logger.logFile << endl;
		else
			cout << endl;
	}

	//Log level 1
	if ((logger.logLevel != SIRLOGLEVEL::LOGLEVEL_NOLOG)) {
		if (logtype == SIRLOGTYPE::LOG_ERROR) {
			if(logger.logTo==SIRLOGTO::LOGTOFILE)
				logger.logFile << "[ERROR]: ";
			else
				cout << "[ERROR]: ";
			++logger.numErrors;
		}
	}
	//Log level 2
	if ((logger.logLevel != SIRLOGLEVEL::LOGLEVEL_NOLOG)&& (logger.logLevel != SIRLOGLEVEL::LOGLEVEL_ERROR)) {
		if (logtype == SIRLOGTYPE::LOG_WARNING) {
			if (logger.logTo == SIRLOGTO::LOGTOFILE)
				logger.logFile << "[WARNING]: ";
			else
				cout << "[WARNING]: ";
			++logger.numWarnings;
		}
	}
	//Log level 3
	if ((logger.logLevel != SIRLOGLEVEL::LOGLEVEL_NOLOG) && (logger.logLevel != SIRLOGLEVEL::LOGLEVEL_ERROR)
		&& (logger.logLevel != SIRLOGLEVEL::LOGLEVEL_WARN)) {
		if (logtype == SIRLOGTYPE::LOG_INFO) {
			if (logger.logTo == SIRLOGTO::LOGTOFILE)
				logger.logFile << "[INFO]: ";
			else
				cout << "[INFO]: ";
		}
	}
	//Log level 4
	if ((logger.logLevel == SIRLOGLEVEL::LOGLEVEL_DEBUG)) {
		if (logtype == SIRLOGTYPE::LOG_OTHER) {
			if (logger.logTo == SIRLOGTO::LOGTOFILE)
				logger.logFile << "[OTHER]: ";
			else
				cout << "[OTHER]: ";
		}
	}
	logger.logType = logtype;
	return logger;
}

// Overload << operator using C style strings
// No need for std::string objects here
SIRLogger &operator << (SIRLogger &logger, const char *text) {

	if ((logger.logLevel != SIRLOGLEVEL::LOGLEVEL_NOLOG)) {
		if (logger.logTo == SIRLOGTO::LOGTOFILE)
			logger.logFile << text << std::endl;
		else
			cout << text << endl;
	}
	logger.logType = SIRLOGTYPE::LOG_NONE;
	return logger;
}

SIRLogger &operator << (SIRLogger &logger, const string text) {

	if ((logger.logLevel != SIRLOGLEVEL::LOGLEVEL_NOLOG)) {
		if (logger.logTo == SIRLOGTO::LOGTOFILE)
			logger.logFile << text << std::endl;
		else
			cout << text << endl;
	}
	logger.logType = SIRLOGTYPE::LOG_NONE;
	return logger;
}
