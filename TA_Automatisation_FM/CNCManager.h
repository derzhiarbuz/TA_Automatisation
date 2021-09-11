#ifndef CNCMANAGER_H
#define CNCMANAGER_H

#include "Serial.h"

//delegate interface
class CNCManagerDelegate {
public:
	virtual void cncPosition(float x, float y, float z) = 0;
	virtual void cncAsksForCommand() = 0;
	virtual void cncEcho(const char *str) = 0;
};


//class itself
class CNCManager {
public:
	CNCManagerDelegate *delegate;

    CNCManager() : _connected(false), delegate(NULL){};
	bool connect(int port=2, int nBaud=9600);
	void disconnect();
	bool conected() {return _serial.IsOpened();}
	void checkSerial();
	void sendString(UnicodeString str);

private:
	bool _connected;
	CSerial _serial;
	char _readedData[10000];
};

#endif
