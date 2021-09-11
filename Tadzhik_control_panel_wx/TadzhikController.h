#ifndef TADZHIKCONTROLLER_H
#define TADZHIKCONTROLLER_H

#include "CNCManager.h"
#include <list>
#include <math.h>
#include <sstream>
#include <fstream>

using namespace std;

struct TALine {
    int N;
    int G;
    float fromX;
    float fromY;
    float fromZ;
    float toX;
    float toY;
    float toZ;

    void setValues(int _N=0, int _G=5, float _fromX=.0, float _fromY=.0, float _fromZ=.0, float _toX=.0, float _toY=.0, float _toZ=.0) {
        N=_N;
        G=_G;
        fromX = _fromX;
        fromY = _fromY;
        fromZ = _fromZ;
        toX = _toX;
        toY = _toY;
        toZ = _toZ;
    }

    float length() {
        return sqrt(pow(toX-fromX,2)+pow(toY-fromY,2)+pow(toZ-fromZ,2));
    }
};



struct TACommand {
    int N;
    int G;
    float X;
    float Y;
    float Z;

    TACommand(int n=0, int g=0, float x=.0, float y=.0, float z=.0) : N(n), G(g), X(x), Y(y), Z(z) {};
};

struct TATelemetry {
    float X;
    float Y;
    float Z;
    float V[2];

    void setValues(float _x=.0, float _y=.0, float _z=.0, float _vx=.0, float _vy=.0) {
        X = _x;
        Y = _y;
        Z = _z;
        V[0] = _vx;
        V[1] = _vy;
    }
};


class TadzhikControllerDelegate {
    public:
        virtual void logString(string &str) = 0;
        virtual void handleTelemetry(TATelemetry &telemetry) = 0;
};


class TadzhikController : public CNCManagerDelegate
{
    public:
        list<TALine> path;
        list<TALine>::iterator lineIter;
        TadzhikControllerDelegate *delegate;
        bool logCommands;
        bool logFeedback;
        bool logEcho;
        bool logTelemetry;

        TadzhikController();
        ~TadzhikController();
        void communicationLoop();
        void connectToPrinter(int port, int baudRate);
        bool connectedToPrinter();
        void disconnect();
        bool convertGCodeToPath(string filename);
        void smoothPath(float tolerance);
        bool sendCommand(string command);
        void setSpeed(int speed);
        void startPrinthead();
        void stopPrinthead();
        bool start();

        //CNCManagerDelegate
        void cncPosition(float x, float y, float z);
        void cncTelemetry(uint8_t *data, bool correct);
        void cncAsksForCommand();
        void cncAsksForCommandRepeat();
        void cncEcho(const char *str);
        void logString(string str);
        int telemetryFrameLength();

    protected:

    private:
        CNCManager _printerManager;
        bool started;

        bool interpretGCodeLine(stringstream &line, TACommand &command);
        void sendCommandFromLine(TALine line);
        void sendCommand(TACommand command);
        void smoothRegion(list<TALine> &region, list<TALine> &result, float tolerance);
};

#endif // TADZHIKCONTROLLER_H
