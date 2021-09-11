#include "CNCManager.h"

#include <string>
#include <stdlib.h>

bool CNCManager::connect(int port, int nBaud) {
    return _serial.Open(port, nBaud);
}

void CNCManager::disconnect() {
    _serial.Close();
}

void CNCManager::checkSerial() {

    if(!this->conected()) return;

	static int k=0;

    while(_serial.ReadDataWaiting()) {
		int bytesRead = _serial.ReadData(_readedData+k, 1);
		k+=bytesRead;
		if(_readedData[k-1]=='\n') {
			_readedData[k]=0;
			switch(_readedData[0]) {
				case 'K' :
				{
					delegate->cncAsksForCommand();
				} break;

				case 'P' :
				{
					float x, y, z;
					sscanf(_readedData+1, "%f %f %f", &x, &y, &z);
                    delegate->cncPosition(x, y, z);
				} break;

				case 'E' :
				{
					delegate->cncEcho(_readedData+1);
                } break;
            }
			k=0;
		}
    }
}

void CNCManager::sendString(UnicodeString str) {
	int a=5, b=7;
    a+=b;
	int l = str.Length();
	char *cStr = new char[l+1];
	for(int i=0; i<l; i++)
		cStr[i] = str[i+1];
	cStr[l] = '\n';
	_serial.SendData(cStr, l+1);
    delete [] cStr;
}
