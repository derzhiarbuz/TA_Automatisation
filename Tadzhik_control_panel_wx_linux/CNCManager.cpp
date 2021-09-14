#include "CNCManager.h"

#include <string>
#include <stdlib.h>
#include <fstream>
#include <sstream>

bool CNCManager::connect(int port, int nBaud) {
    string s;
    stringstream ss;
    ss<<port;
    s = "/dev/ttyACM" + ss.str();
    _serial.SetPort(s);
    _serial.SetBaudRate(nBaud);
    _serial.SetDataSize(8);
    _serial.SetParity('N');
    _serial.SetStopBits(1);
    _serial.Open();
    return _serial.IsOpened();
}

void CNCManager::disconnect() {
    _serial.Close();
}

void CNCManager::checkSerial() {

    if(!this->conected()) return;

	static int k=-1; //telemetry bytes
	static char message_type=0;
	bool success;
	char c;

	c = _serial.ReadChar(success);
    while(success) {
        if(k<0) { //if not echo or telemetry started
            switch(c) {
                case 'K' :
				{
					delegate->cncAsksForCommand();
				} break;

				case 'F' :
				{
					delegate->cncAsksForCommandRepeat();
				} break;

				case 'R' :
				{
					delegate->cncEcho("<--received");
				} break;

				case 'E' :
				{
				    k=0;
					_readedData[0]='E';
                } break;

                case 'T' :
				{
				    k=0;
					_readedData[0]='T';
                } break;
            }
        }
        else { //if echo or telemetry reading
            k++;
            _readedData[k] = c;
            if(_readedData[0] == 'E') { //echo reading
                if(_readedData[k] == '\n') { //string finished
                    _readedData[k+1] = 0;
                    delegate->cncEcho(_readedData+1);
                    k=-1;
                }
            }
            else if(_readedData[0] == 'T') { //telemetry reading
                if(k >= delegate->telemetryFrameLength()+2) { //frame finished
                    this->processTelemetry((uint8_t*)_readedData, delegate->telemetryFrameLength()+3);
                    k=-1;
                }
            }
            else
                k=-1;
        }

		c = _serial.ReadChar(success);
    }
}

//sending data as package [ 0xFF | data | checksum ], data is 8 bytes, checksum is 2 bytes
void CNCManager::sendData(char *data, int length) {
    uint8_t *dataToSend;
    int16_t checksum = 0;
    int16_t *ptr;
    dataToSend = new uint8_t[length+3];
    for(int i=0; i<length; i++) {
        dataToSend[i+1] = data[i];
        checksum += (int16_t)dataToSend[i+1];
    }
    dataToSend[0] = 0xFF; //package head flag
    ptr = (int16_t*)&(dataToSend[length+1]);
    *ptr = checksum; //checksum

    //stringstream sstr;
    //for(int i=0; i<length+3; i++)
    //    sstr<<(int)dataToSend[i]<<" ";
    //delegate->logString(sstr.str());
    //cout<<sstr.str();
    //cout<<endl;
	_serial.Write((char*)dataToSend, length+3);
}

//checking telemetry checksum and send it to delegate
void CNCManager::processTelemetry(uint8_t *data, int length) {
    int16_t checksum, *received_checksum;
    bool correct;

    for(int i=1; i<length-2; i++)
        checksum += data[i];

    received_checksum = (int16_t*)&data[length-2];
    if(checksum == *received_checksum)
        correct = true;
    else
        correct = false;

    delegate->cncTelemetry(data+1, correct);
}
