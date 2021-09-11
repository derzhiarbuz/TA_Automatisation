#ifndef CNCMANAGER_H
#define CNCMANAGER_H

#include <iostream>
#include "Serial.h"

//delegate interface
class CNCManagerDelegate {
public:
	virtual void cncPosition(float x, float y, float z) = 0;
	virtual void cncTelemetry(uint8_t *data, bool correct) = 0;
	virtual void cncAsksForCommand() = 0;
	virtual void cncAsksForCommandRepeat() = 0;
	virtual void cncEcho(const char *str) = 0;
	virtual void logString(string str) = 0;
	virtual int telemetryFrameLength() = 0;
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
	void sendData(char *data, int length);

private:
	bool _connected;
	Serial _serial;
	char _readedData[10000];
	void processTelemetry(uint8_t *data, int length);
};

#endif
