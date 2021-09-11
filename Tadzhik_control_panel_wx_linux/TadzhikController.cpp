#include "TadzhikController.h"

//#include <conio.h>

using namespace std;

TadzhikController::TadzhikController()
{
    logCommands = true;
    logFeedback = true;
    logEcho = true;
    logTelemetry = true;
    started = false;
    delegate = 0;
    _printerManager.delegate = this;
}

TadzhikController::~TadzhikController()
{
    _printerManager.delegate = 0;
}

void TadzhikController::communicationLoop() {
    if(_printerManager.conected())
        _printerManager.checkSerial();
}

//establish connection to printer
void TadzhikController::connectToPrinter(int port, int baudRate) {
    _printerManager.connect(port, baudRate);
}

bool TadzhikController::connectedToPrinter() {
    return _printerManager.conected();
}

void TadzhikController::disconnect() {
    _printerManager.disconnect();
}

bool TadzhikController::convertGCodeToPath(string filename) {
    ifstream input_stream(filename);
    if (!input_stream.is_open())
    {
        cout<<"Unable to open "<<filename<<endl;
        return false;
    }

    char c, str_line[1000];
    TACommand com;
    TALine origin, prev;
    origin.setValues();
    prev.setValues();
    bool success = true;
    int n=0;

    while(!input_stream.eof()) {

        input_stream.getline(str_line, 1000);
        stringstream sstr(str_line);

        if(this->interpretGCodeLine(sstr, com)) {
            TALine line;
            line.setValues(n, com.G, origin.toX + prev.toX, origin.toY + prev.toY, origin.toY + prev.toZ, origin.toX + com.X, origin.toY + com.Y, origin.toZ + com.Z);
            if(line.G == 5)
                origin = line;

            this->path.push_back(line);
            n++;

            prev = line;
        }
    }

    input_stream.close();
}

bool TadzhikController::sendCommand(string command) {
    TACommand com;
    stringstream sstr(command);
    bool result = true;

    if(this->interpretGCodeLine(sstr, com))
        this->sendCommand(com);
    else
        result = false;

    return result;
}

bool TadzhikController::interpretGCodeLine(stringstream &line, TACommand &command) {
    char c;
    int num;
    float val;
    bool success = true;

    line>>c;
    if(c=='G' || c=='g') { //this line has command
        if(!line.eof()) {
            line>>num;
            if(num!=0 && num!=1 && num!=5 && num!=20 && num!=21 && num!=22) {
                success = false;
            }
            else
                command.G = num;
        }

        while(!line.eof()) {
            line>>c;
            if(!line.eof()) {
                line>>val;
                if(c=='X' || c=='x') command.X = val;
                else if(c=='Y' || c=='y') command.Y = val;
                else if(c=='Z' || c=='z') command.Z = val;
            }
        }
    }
    else {
        success = false;
    }

    return success;
}

void TadzhikController::setSpeed(int speed) {
    stringstream sstr;
    sstr<<"G20X"<<speed;
    this->sendCommand(sstr.str());
}

void TadzhikController::startPrinthead() {
    stringstream sstr;
    sstr<<"G21";
    this->sendCommand(sstr.str());
}

void TadzhikController::stopPrinthead() {
    stringstream sstr;
    sstr<<"G22";
    this->sendCommand(sstr.str());
}

void TadzhikController::sendCommandFromLine(TALine line) {
    TACommand cmd;

    cmd.N = line.N;
    cmd.G = line.G;
    cmd.X = line.toX;
    cmd.Y = line.toY;
    cmd.Z = line.toZ;

    this->sendCommand(cmd);
}

void TadzhikController::sendCommand(TACommand command) {
    stringstream sstr;
    if(logCommands) {
        sstr<<"-->"<<command.N<<" G"<<command.G<<" X"<<(int16_t)command.X<<" Y"<<(int16_t)command.Y<<" Z"<<(int16_t)command.Z;
        this->logString(sstr.str());
    }

    char frame[9];
    int16_t *coords;
    coords = (int16_t*)&frame[1];

    frame[0] = command.G;
    coords[0] = (int16_t)command.X;
    coords[1] = (int16_t)command.Y;
    coords[2] = (int16_t)command.Z;
    coords[3] = (int16_t)command.N;

    _printerManager.sendData(frame, 9);
}

bool TadzhikController::start() {
    if(!path.size()) return false;

    lineIter = path.begin();
    started = true;

    this->sendCommandFromLine(*lineIter);
}

void TadzhikController::smoothPath(float tolerance) {
    if(!path.size()) return;

    TALine *line, *line2;
    list<TALine> region, smoothed_region;
    list<TALine>::iterator left, right, iter, iter2;
    int currentG = -1, smoothed = 0;

    iter = path.begin();

    while(!smoothed) {
        if(iter != path.end()) line = &(*iter);
        //if short segments region finished or there is another G, smoothing existing region
        if((iter==path.end() || line->length() >= tolerance || line->G != currentG) && region.size()) {
            //cout<<"Smooth"<<region.size()<<endl;
            this->smoothRegion(region, smoothed_region, tolerance);
            //if smoothed region has non zero length
            if(smoothed_region.size()) {
                iter = path.insert(iter, smoothed_region.begin(), smoothed_region.end());
                //cout<<smoothed_region.size()<<" "<<path.size()<<endl;
            }
            //if smoothed region has zero length, connect left and right section at the middle
            else {
                if(path.size() <= 1) {

                }
                else if(iter == path.begin()) {
                    line = &(path.front());
                    line2 = &(*(region.begin()));
                    line->fromX = line2->fromX;
                    line->fromY = line2->fromY;
                    line->fromZ = line2->fromZ;
                }
                else if(iter == path.end()) {
                    line = &(path.back());
                    line2 = &(region.back());
                    line->toX = line2->toX;
                    line->toY = line2->toY;
                    line->toZ = line2->toZ;
                }
                else {
                    iter2 = iter;
                    iter2--;
                    line2 = &(*iter2);
                    line2->toX = (line2->toX + line->fromX) / 2.;
                    line->fromX = line2->toX;
                    line2->toY = (line2->toY + line->fromY) / 2.;
                    line->fromY = line2->toY;
                    line2->toZ = (line2->toZ + line->fromZ) / 2.;
                    line->fromZ = line2->toZ;
                }
            }
            region.clear();
            smoothed_region.clear();
            currentG = -1;
        }

        //if path fibished, finish
        if(iter == path.end())
            smoothed = 1;
        else {
                line = &(*iter);
                //if line short enough, add it to region and remove from path
                if(line->length() < tolerance) {
                    //cout<<line->length()<<" "<<path.size()<<endl;
                    if(region.size()==0)
                        currentG = line->G;

                    region.push_back(*line);
                    iter = path.erase(iter);
                }
                else {
                    //cout<<line->length()<<" iter++"<<endl;
                    iter++;
                }
        }
    }
}

void TadzhikController::smoothRegion(list<TALine> &region, list<TALine> &result, float tolerance) {
    result.clear();
    if(!region.size()) return;

    float total_length = .0, segment_length, prev_length, lambda;
    TALine *line, *prev_line;
    TALine newLine;
    int n_segments;
    list<TALine>::iterator first_iter, last_iter, iter, prev_iter;

    for(TALine element : region) {
        total_length += element.length();
    }

    if(total_length < tolerance) return; //return empty result, that means region smoothed to single vertex

    n_segments = total_length/tolerance; //calculating number of segments on smoothed path
    segment_length = total_length/n_segments;

    first_iter = region.begin();
    last_iter = region.end();
    line = &(*first_iter);
    newLine.setValues(line->N, line->G, line->fromX, line->fromY, line->fromZ);

    total_length = .0;
    prev_length = .0;
    for(iter = region.begin(); iter != region.end(); iter++) {
        line = &(*iter);
        total_length += line->length();

        if(total_length >= segment_length - 1e-12) {
            //making new point on path as linear combination of points at left and right
            lambda = (total_length - segment_length)/(total_length-prev_length);
            newLine.toX = line->fromX * lambda + line->toX * (1. - lambda);
            newLine.toY = line->fromY * lambda + line->toY * (1. - lambda);
            newLine.toZ = line->fromZ * lambda + line->toZ * (1. - lambda);
            result.push_back(newLine);
            //set next line segment "from" point to prev line "to" point
            newLine.fromX = newLine.toX;
            newLine.fromY = newLine.toY;
            newLine.fromZ = newLine.toZ;
            total_length -= segment_length;
        }

        prev_length = total_length;
    }
}

//CNCManagerDelegate
void TadzhikController::cncPosition(float x, float y, float z) {
    if(logTelemetry) {
        stringstream sstr;
        sstr<<"<--X"<<x<<" Y"<<y<<" Z"<<z;
        this->logString(sstr.str());
    }
}

void TadzhikController::cncTelemetry(uint8_t *data, bool correct) {
    uint16_t *n;
    float *values;
    static TATelemetry tele;

    n = (uint16_t*)data;
    values = (float*)&data[2];

    if(correct) {
        tele.setValues(values[0], values[1], values[2], values[3], values[4]);
        delegate->handleTelemetry(tele);
    }
    else
        this->logString("Bad telemetry");
}

void TadzhikController::cncAsksForCommand() {
    if(logFeedback)
        this->logString("<--asks for command");

    if(!started) return;

    lineIter++;
    if(lineIter != path.end())
        this->sendCommandFromLine(*lineIter);
}

void TadzhikController::cncAsksForCommandRepeat() {
    if(logFeedback)
        this->logString("<--asks for command repeat");

    if(!started) return;

    if(lineIter != path.end())
        this->sendCommandFromLine(*lineIter);
}

void TadzhikController::cncEcho(const char *str) {
    if(logEcho)
        this->logString(string(str));
}

void TadzhikController::logString(string str) {
    if(delegate)
        delegate->logString(str);
}

int TadzhikController::telemetryFrameLength() {
    const int TELEMETRY_FRAME_LENGTH = 2+9*4;
    return TELEMETRY_FRAME_LENGTH;
}
