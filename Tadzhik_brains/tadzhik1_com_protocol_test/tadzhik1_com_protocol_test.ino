//======================================GCode Manager=========================================================

const int C_BUFFER_LENGTH = 30;
const int C_FRAME_LENGTH = 10;
const char C_FRAME_START_TOKEN = 0xFF;

class Command {
  public:
  int G;
  float X;
  float Y;
  float Z;

  Command(int g=0, float x=.0, float y=.0, float z=.0) {
    setValues(g,x,y,z);
  }
  void setValues(int g=0, float x=.0, float y=.0, float z=.0) {
    G=g;
    X=x;
    Y=y;
    Z=z;
  }
};


class CommandManager {
  public:
  int nCommands;
  int waitingForCommand;
  Command c_buffer[C_BUFFER_LENGTH+10];

  CommandManager() : nCommands(0), waitingForCommand(0) {}
  void readSerial(); //read frame 0xFF | G(1b) | X(2b) | Y(2b) | Z(2b) | checksum(2b)
  int checkFrame(char frame[C_FRAME_LENGTH]);
  Command commandFromFrame(char frame[C_FRAME_LENGTH]);
  void answerReceived();
  void answerFailed();
  void askForCommand();
  int canScheduleCommand();
  void scheduleCommand(Command cmd);
  int canTakeCommand();
  Command takeCommand();
};

void CommandManager::readSerial() {
  static char frame[C_FRAME_LENGTH];
  static int current_byte_num = -1; //-1 means frame reading haven't started yet
  static char incomingByte = 0;
  

  while(Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if(current_byte_num == -1 && incomingByte == C_FRAME_START_TOKEN) {
      current_byte_num = 0;
      frame[current_byte_num] = incomingByte;
    }
    else if(current_byte_num > -1) {
      current_byte_num++;
      frame[current_byte_num] = incomingByte;
      if(current_byte_num >= C_FRAME_LENGTH-1) { //frame finished
        if(this->checkFrame(frame)) {
          waitingForCommand = 0;
          //process frame
          this->scheduleCommand(this->commandFromFrame(frame));
          this->answerReceived();
        }
        else {
          waitingForCommand = 0;
          //frame corrupted, send repeat request
          this->answerFailed();
        }
        current_byte_num = -1;
      }
    }
  }
}

int CommandManager::checkFrame(char frame[C_FRAME_LENGTH]) {
  static int16_t checksum;
  static int16_t *frame_checksum;
  static uint8_t *uint_frame;

  uint_frame = (uint8_t*)frame;
  checksum = 0;
  for(int i=1; i<C_FRAME_LENGTH-2; i++) //first byte is FF and last two are checksum
    checksum += uint_frame[i];
  frame_checksum = (int16_t*)&uint_frame[C_FRAME_LENGTH-2]; 
  return checksum == *frame_checksum;
}

Command CommandManager::commandFromFrame(char frame[C_FRAME_LENGTH]) {
  Command com;
  static int16_t *values;
  
  com.G = frame[1];
  values = (int16_t*)&frame[2];
  com.X = values[0];
  com.Y = values[1];
  com.Z = values[2];

  return com;
}

void CommandManager::answerReceived() {
  Serial.println("R");
}

void CommandManager::answerFailed() {
  if(waitingForCommand) return;
  Serial.println("F");
  waitingForCommand = 1;
}

void CommandManager::askForCommand() {
  if(waitingForCommand) return;
  if(this->canScheduleCommand()) {
    Serial.println("K");
  }
  waitingForCommand = 1;
}

int CommandManager::canScheduleCommand() {
  if(nCommands<C_BUFFER_LENGTH) return 1;
    return 0;
}

void CommandManager::scheduleCommand(Command cmd) {
  c_buffer[nCommands].setValues(cmd.G, cmd.X, cmd.Y, cmd.Z);
  nCommands++;
}

int CommandManager::canTakeCommand() {
  if(nCommands<=0) return 0;
  return 1;
}

Command CommandManager::takeCommand() {
  nCommands--;

  Command retCom = c_buffer[0];

  for(int i=0; i<nCommands; i++) {
    c_buffer[i].G = c_buffer[i+1].G;
    c_buffer[i].X = c_buffer[i+1].X;
    c_buffer[i].Y = c_buffer[i+1].Y;
    c_buffer[i].Z = c_buffer[i+1].Z;
  }
  return retCom;
}

//============================================================================================================
//============================================================================================================
//============================================================================================================


CommandManager cm;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    cm.readSerial();
  }

  cm.askForCommand();
}
