#ifndef TA1GCODEMANAGER
#define TA1GCODEMANAGER

class Command {
  public:
  float G;
  float X;
  float Y;
  float Z;
  float F;
  Command(float g=.0, float x=.0, float y=.0, float z=.0, float f=.0) {
    setValues(g,x,y,z,f);
  }
  void setValues(float g=.0, float x=.0, float y=.0, float z=.0, float f=.0) {
    G=g;
    X=x;
    Y=y;
    Z=z;
    F=f;
  }
};


class ComandManager {
	public:
	int nCommands;
	Command c_buffer[C_BUFFER_LENGTH+5];

	ComandManager() : nCommans(0){}
	void readSerial();
	int canScheduleCommand();
	void scheduleCommand(float G, float X, float Y, float Z, float F);
	int canTakeCommand();
	Command takeCommand();
};


#endif
