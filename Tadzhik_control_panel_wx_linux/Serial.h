//File: Serial.h
//Description: Serial communication class for Windows and Linux
//WebSite: http://cool-emerald.blogspot.sg/2017/05/serial-port-programming-in-c-with.html
//MIT License (https://opensource.org/licenses/MIT)
//Copyright (c) 2017 Yan Naing Aye

// References
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// http://www.silabs.com/documents/public/application-notes/an197.pdf
// https://msdn.microsoft.com/en-us/library/ff802693.aspx
// http://www.cplusplus.com/forum/unices/10491/

#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
using namespace std;

//---------------------------------------------------------
#if defined (__WIN32__) || defined(_WIN32) || defined(WIN32) || defined(__WINDOWS__) || defined(__TOS_WIN__)
    #define CEWIN
#endif

#ifdef CEWIN

#include <windows.h>
#define READ_TIMEOUT 10      // milliseconds
inline void delay(unsigned long ms)
{
	Sleep(ms);
}

#else

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
inline void delay(unsigned long ms)
{
	usleep(ms*1000);
}

#endif
//---------------------------------------------------------
//Class definition
class Serial {
	char rxchar;
	string port;
	long baud;
	long dsize;
	char parity;
	float stopbits;
	#ifdef CEWIN
    HANDLE hComm; //handle
	OVERLAPPED osReader;
	OVERLAPPED osWrite;
	BOOL fWaitingOnRead;
	COMMTIMEOUTS timeouts_ori;
	#else
	long fd;//serial_fd
	#endif
public:
	Serial();
	Serial(string Device, long BaudRate, long DataSize, char ParityType, float NStopBits);
	~Serial();
	long Open(void);//return 0 if success
	void Close();
	char ReadChar(bool& success);//return read char if success
	bool WriteChar(char ch);////return success flag
	bool Write(char *data);//write null terminated string and return success flag
	bool Write(char *data, long n);//write array and return success flag
	bool SetRTS(bool value);//return success flag
	bool SetDTR(bool value);//return success flag
	bool GetCTS(bool& success);
	bool GetDSR(bool& success);
	bool GetRI(bool& success);
	bool GetCD(bool& success);
	bool IsOpened();
	void SetPort(string Port);
	string GetPort();
	void SetBaudRate(long baudrate);
	long GetBaudRate();
	void SetDataSize(long nbits);
	long GetDataSize();
	void SetParity(char p);
	char GetParity();
	void SetStopBits(float nbits);
	float GetStopBits();
};

//------------------------------------------------------------------------------
#endif // SERIAL_H
