#include <math.h>
#include <vector3d.h>

//======================================GCode Manager=========================================================

const int C_BUFFER_LENGTH = 20;
const int C_FRAME_LENGTH = 12;
const char C_FRAME_START_TOKEN = 0xFF;

class Command {
  public:
  int N;
  int G;
  float X;
  float Y;
  float Z;

  Command(int n=0, int g=0, float x=.0, float y=.0, float z=.0) {
    setValues(n,g,x,y,z);
  }
  void setValues(int n=0, int g=0, float x=.0, float y=.0, float z=.0) {
    N=n;
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
  Command c_buffer[C_BUFFER_LENGTH+5];
  int setSpeedReceived;
  int startPrintheadReceived;
  int stopPrintheadReceived;

  CommandManager() : nCommands(0), waitingForCommand(0), setSpeedReceived(-1), startPrintheadReceived(0), stopPrintheadReceived(0) {}
  void readSerial(); //read frame 0xFF | G(1b) | X(2b) | Y(2b) | Z(2b) | N(2b) | checksum(2b)
  int checkFrame(char frame[C_FRAME_LENGTH]);
  Command commandFromFrame(char frame[C_FRAME_LENGTH]);
  void answerReceived();
  void answerFailed();
  void askForCommand();
  void sendTelemetry(int16_t n, float x, float y, float z, float vx, float vy, float lx, float ly, float qx, float qy);
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
  com.N = values[3];

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
    waitingForCommand = 1;
  }
}

void CommandManager::sendTelemetry(int16_t n, float x, float y, float z, float vx, float vy, float lx, float ly, float qx, float qy) {
  const int C_TELEMETRY_FRAME_LENGTH = 1+2+9*4+2;
  static uint8_t frame[1+2+9*4+2];
  static int16_t *intptr, *checksum;
  static float *floatptr;

  frame[0] = 'T';
  intptr = (int16_t*)&frame[1];
  floatptr = (float*)&frame[3];
  checksum = (int16_t*)&frame[C_TELEMETRY_FRAME_LENGTH-2];
  intptr[0] = n;
  floatptr[0] = x;
  floatptr[1] = y;
  floatptr[2] = z;
  floatptr[3] = vx;
  floatptr[4] = vy;
  floatptr[5] = lx;
  floatptr[6] = ly;
  floatptr[7] = qx;
  floatptr[8] = qy;

  *checksum = 0;
  for(int i=1; i<C_TELEMETRY_FRAME_LENGTH-2; i++)
    *checksum += frame[i];

  Serial.write((char*)frame, C_TELEMETRY_FRAME_LENGTH);
}

int CommandManager::canScheduleCommand() {
  if(nCommands<C_BUFFER_LENGTH) return 1;
    return 0;
}

void CommandManager::scheduleCommand(Command cmd) {
  if(cmd.G == 20)
    setSpeedReceived = cmd.X;
  else if(cmd.G == 21) {
    startPrintheadReceived = 1;
    Serial.println("EHead ON R");
  }
  else if(cmd.G == 22) {
    stopPrintheadReceived = 1;
     Serial.println("EHead OFF R");
  }
  else {
    c_buffer[nCommands].setValues(cmd.N, cmd.G, cmd.X, cmd.Y, cmd.Z);
    nCommands++;
  }
}

int CommandManager::canTakeCommand() {
  if(nCommands<=0) return 0;
  return 1;
}

Command CommandManager::takeCommand() {
  nCommands--;

  Command retCom = c_buffer[0];

  for(int i=0; i<nCommands; i++) {
    c_buffer[i].N = c_buffer[i+1].N;
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



//-------------------------------------------------------------------------------------------------------------
//                                                data types

//-------------------------------------------------------------------------------------------------------------
//                                            telemetry variables

float xyz[3] = {.0, .0, .0}; //current position (mm)
float newZ = 0;
int haveNewZ = 0;
int xyz_encode[3][2] = {{0, 0}, {0, 0}, {0, 0}}; //values of encoder outputs (2 per encoder)
int xyz_encode_pins[3][2] = { {2, 3}, {18, 19}, {20, 21} }; //pins for rotary encoders
double xyz_encode_deltas[3] = { 0.025, 0.025, 0.025}; //value of encoder increment for each axis (mm)
//const int ENCODER_TRESHOLD = 300; //treshold for analog interpritation of 0 or 1



//-------------------------------------------------------------------------------------------------------------
//                                             control variables

float alpha = 0;
float sinAlpha = 0;
float cosAlpha = 0;
unsigned long newTime = 0;
unsigned long lastTime;

vector3d p0, p1, p2; //beginning and end of current trajectory (line segment) and next trajectory
vector3d l, q, v; //orthogonal, collinear and result speed vectors
int have_p2 = 0;
//pins for motors control {ENABLE, STEP, DIR} for X, Y and {ON, SPEED, DIR} for Z (voltage inverted)
const int xyz_motors_pins[3][3] = { {43, 45, 47}, {49, 51, 53}, {11, 13, 12} };
const int xy_steps_per_revolution[2] = { 3200, 3200 };//steps per revolution
const float xy_dist_per_revolution[2] = { 105.24, 105.24 };//distance per revolution
float xy_dist_per_step[2] = { 0, 0 };//distance per step
long int xy_motors_time[2] = { 0, 0 }; //current time from last step in us
long int xy_motors_delay[2] = {0, 0 }; //0 if stopped, delay between steps in us if not

const int PUMP_PIN = 9; //pin controlling pump ON/OFF
const int AUGER_PIN = 10; //pin controlling auger ON/OFF (voltage inverted)
const int AUGER_DELAY_MS = 500; //delay between auger ON signal and auger start (because of inverter)
const int PRINT_DELAY_MS = 500;
int printheadForceOn;

float lv = .0; //start speed depends on angle between prev and current trajectories
float rv = .0; //finish speed depends on angle between current and next trajectories
float cos_l = .0;
float cos_r = .0;
float MAX_SPEED = 30; //maximu speed mm/s
const float MIN_START_SPEED = 1.5; //minimum speed at acceleration mm/s
const float MIN_SPEED = 0.005; //minimum speed at decceleration mm/s
const float MAX_ACCEL = 2; //accel mm/s2
const float ZERO_DIST = 0.3; //distance, in mm, treated as zero
const float MIN_ORT_SPEED_2 = 0; //minimum ortogonal speed to move
int router_status;


//-------------------------------------------------------------------------------------------------------------
//                                                 functions

//returns delay between steps (us) for given velocity (mm/s)
int velocityToT(float vel, int axis) {
  float second_per_step = xy_dist_per_step[axis]/vel;
  if(second_per_step>10) second_per_step = .0;
  return second_per_step*1000000;
 }

//init timer interrupt
void initTimer();
// read encoders and update distances.
//ISR(TIMER2_COMPA_vect);
//init encoders arrays
void initEncoders();
//init motors arrays
void initControl();
//calculate and update control signals for motors
int proceedControl();
//starting and stopping printhead (warning, delay() called inside)
void startPrinthead();
void stopPrinthead();

//==============================================================================================================
unsigned long timespent;
unsigned long counter;
CommandManager cManager;

void setup()
{
  counter = 0;
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  
  initControl();
  initTimer();
  initEncoders();
  
  router_status = 1;
  printheadForceOn = 0;
}


void loop()
{
/*  if (Serial.available() > 0 && router_status==1) 
  {
    Serial.print("XY:\n  ");
     readGcode();
  } 
//  Serial.print("\n");
//  Serial.print(timespent);
  proceedControl();
//  delay(1);
*/
   int statusBefore = router_status;
   if (Serial.available() > 0) 
   {
     cManager.readSerial();
   }
   cManager.askForCommand();

   if(cManager.setSpeedReceived >= 0) {
     MAX_SPEED = cManager.setSpeedReceived;
     cManager.setSpeedReceived = -1; 
   }

   if(cManager.startPrintheadReceived) {
     printheadForceOn = 1;
     cManager.startPrintheadReceived = 0;
     startPrinthead();
   }

   if(cManager.stopPrintheadReceived) {
     printheadForceOn = 0;
     cManager.stopPrintheadReceived = 0; 
     stopPrinthead();
   }
   
   
   proceedControl();
   if(router_status == 1) {
     proceedCommand();
   }
   
  /* if(router_status == 1) {
       if(statusBefore != 1)
         if(cManager.canScheduleCommand()) {
          Serial.println("K");
        }
       proceedCommand();
   }*/

   static unsigned long meas_timer = 0;
   static unsigned long new_mt = 0;
   new_mt = millis();
   if(new_mt-meas_timer > 100)
   {
      cManager.sendTelemetry(0, xyz[0], xyz[1], xyz[2], v.x, v.y, l.x, l.y, q.x, q.y);
      meas_timer = new_mt;
   }
}

//==============================================================================================================
//                                      initializing 500us timer interrupt
void initTimer() {
  //setting imer interrupt
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 4000; //for 16MHz it's once per 500us
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
}

//==============================================================================================================
//                                      500us timer interrupt
//                                making step for motor when needed
ISR(TIMER1_COMPA_vect) {
  for(int i=0; i<2; i++) {
    if(xy_motors_delay[i]) {
      xy_motors_time[i] += 500;
      if(xy_motors_time[i] >= xy_motors_delay[i] ) {
        xy_motors_time[i] = 0;
        digitalWrite(xyz_motors_pins[i][1], LOW);
      }
      else {
        digitalWrite(xyz_motors_pins[i][1], HIGH);
      }
    }
    else {
      digitalWrite(xyz_motors_pins[i][1], HIGH);
    }
  }
}

//==============================================================================================================
//                                       initializing encoders input

void initEncoders() {
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<2; j++)
      pinMode(xyz_encode_pins[0][0], INPUT);
  
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][0]), encoder_x_0_change, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][1]), encoder_x_1_change, CHANGE);  

  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][0]), encoder_y_0_change, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][1]), encoder_y_1_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][0]), encoder_z_0_change, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][1]), encoder_z_1_change, CHANGE);
}

//==============================================================================================================
//                                      encoders input interruptions  

//===================================encoder X
void encoder_x_0_change() {
  xyz_encode[0][0] = digitalRead(xyz_encode_pins[0][0]);
  xyz_encode[0][1] == xyz_encode[0][0] ? xyz[0] -= xyz_encode_deltas[0] : xyz[0] += xyz_encode_deltas[0];
}

void encoder_x_1_change() {
  xyz_encode[0][1] = digitalRead(xyz_encode_pins[0][1]);
  xyz_encode[0][0] == xyz_encode[0][1] ? xyz[0] += xyz_encode_deltas[0] : xyz[0] -= xyz_encode_deltas[0];
}

//===================================encoder Y
void encoder_y_0_change() {
  xyz_encode[1][0] = digitalRead(xyz_encode_pins[1][0]);
  xyz_encode[1][1] == xyz_encode[1][0] ? xyz[1] -= xyz_encode_deltas[1] : xyz[1] += xyz_encode_deltas[1];
}

void encoder_y_1_change() {
  xyz_encode[1][1] = digitalRead(xyz_encode_pins[1][1]);
  xyz_encode[1][0] == xyz_encode[1][1] ? xyz[1] += xyz_encode_deltas[1] : xyz[1] -= xyz_encode_deltas[1];
}

//===================================encoder Z
void encoder_z_0_change() {
  xyz_encode[2][0] = digitalRead(xyz_encode_pins[2][0]);
  xyz_encode[2][1] == xyz_encode[2][0] ? xyz[2] -= xyz_encode_deltas[2] : xyz[2] += xyz_encode_deltas[2];
}

void encoder_z_1_change() {
  xyz_encode[2][1] = digitalRead(xyz_encode_pins[2][1]);
  xyz_encode[2][0] == xyz_encode[2][1] ? xyz[2] += xyz_encode_deltas[2] : xyz[2] -= xyz_encode_deltas[2];
}


//==============================================================================================================
void initControl() {
  int i, j;
  for(i=0; i<3; i++) {

    for(j=0; j<3; j++) {
      pinMode(xyz_motors_pins[i][j], OUTPUT);
      digitalWrite(xyz_motors_pins[i][j], HIGH);
    }
  }

  for(i=0; i<2; i++) {
    xy_dist_per_step[i] = xy_dist_per_revolution[i]/xy_steps_per_revolution[i];
  }

  digitalWrite(xyz_motors_pins[0][0], LOW); //enable X motor
  digitalWrite(xyz_motors_pins[1][0], LOW); //enable Y motor

  pinMode(AUGER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  stopPrinthead(); //initial printhead OFF
}

//==============================================================================================================
vector3d p, c, c2, q2;
float sigma, norm2q, norm2l, normq, norml, norm2vort, normvort, norm2vcoll, normvcoll, maxv2, norm2vcollmax, dsigma;
unsigned long prev_time=0, _time;
float dt;
  
int proceedControl() {
  _time = micros();
  dt = (float)(_time - prev_time)*0.000001*0.1 + dt*0.9; //dt in seconds
  prev_time = _time;
  
/*  //temporary proceeding Z
  if(Ztemp > 0.1) {
    Ztemp -= dt;
    if(Ztemp <= 0.1) Ztemp = 0;
    analogWrite(xyz_motors_pins[2][0], 80);
    digitalWrite(xyz_motors_pins[2][1], HIGH);
  }
  else if(Ztemp < -0.1) {
    Ztemp += dt;
    if(Ztemp >= -0.1) Ztemp = 0;
    analogWrite(xyz_motors_pins[2][0], 80);
    digitalWrite(xyz_motors_pins[2][1], LOW);
  }
  else {
    analogWrite(xyz_motors_pins[2][0], 0);
  }
 */   
  
  if(router_status == 1) return 0;

//strange Z axis routine
  if(haveNewZ) {
    if(xyz[2] < newZ-1.52) {
      //float zmul = ((newZ-xyz[2])/5.0)+0.3;
      //if(zmul>1.0) zmul=1.0;
      digitalWrite(xyz_motors_pins[2][0], LOW);
      digitalWrite(xyz_motors_pins[2][2], LOW);
    }
    else {
      digitalWrite(xyz_motors_pins[2][0], HIGH);
      digitalWrite(xyz_motors_pins[2][2], LOW);
      haveNewZ = 0;
/*      Serial.print("\nP"); //trajectory finished, sending status 1 to get next waypoint
      Serial.print(xyz[0]);
      Serial.print(" ");
      Serial.print(xyz[1]);
      Serial.print(" ");
      Serial.println(xyz[2]);*/
    }
  }

  
//    analogWrite(xyz_motors_pins[1][0], 30);
//    analogWrite(6, 120);
//   digitalWrite(xyz_motors_pins[1][1], HIGH);
  
  //p = vectorMake(xyz[0], xyz[1], 0);
  p = vectorMake(xyz[0], xyz[1], 0);
  
  q = vectorDiff(p1, p0);
  norm2q = vectorScalarMul(q, q);
  if(norm2q < ZERO_DIST) {
    for(int i=0; i<2; i++) 
       xy_motors_delay[i] = 0; //stopping all motors
    if(!haveNewZ)
      router_status = 1;
    return 0; //last point, no way next
  }
  normq = sqrt(norm2q);
  
  c = vectorDiff(p, p0);
  sigma = vectorScalarMul(q, c)/norm2q;

  //calculate sigma2 for next traj segment if needed (shall_go_next needn't to calculate each moment)
  float sigma2 = .0, norm2q2, normq2;
  int shall_go_next = 0;
  if(have_p2) {
    q2 = vectorDiff(p2, p1);
    norm2q2 = vectorScalarMul(q2, q2);
    if(norm2q > ZERO_DIST) {
      shall_go_next = 1;
      c2 = vectorDiff(p, p1);
      sigma2 = vectorScalarMul(q2, c2)/norm2q2;
      normq2 = sqrt(norm2q2);
    }
  }

  //say we finishing current traj segment if close to final point or close to next traj
  if( (abs((1.0-sigma))*normq < ZERO_DIST || abs((1.0-sigma))*normq < abs((sigma2))*normq2)) {
    if(!haveNewZ)
      router_status = 1;
    if(!shall_go_next) 
    {
      for(int i=0; i<2; i++)
        xy_motors_delay[i] = 0; //stopping all motors
    }
 /*     Serial.print("\nP"); //trajectory finished, sending status 1 to get next waypoint
      Serial.print(xyz[0]);
      Serial.print(" ");
      Serial.print(xyz[1]);
      Serial.print(" ");
      Serial.println(xyz[2]);*/
    return 1; //end of trajectory
  }
  //l is vector from current point (p) to closest point on trajectory (p0 p1)
  l = vectorDiff( vectorMul(q, sigma), c );
  norm2l = vectorScalarMul(l, l);
  if(norm2l < 0.00001 /* ZERO_DIST*/) {
    norm2l = .0;
    norml = .0;
  }
  else norml = sqrt(norm2l);
  
  maxv2 = MAX_SPEED * MAX_SPEED;
  
  //calculating square absolute value of trajectory-ortogonal speed component
  norm2vort = MAX_ACCEL * 2 * norml/4; 
  if(norm2vort < MIN_ORT_SPEED_2) norm2vort = .0;
/*  Serial.print("\n norm2l : ");
  Serial.print(norm2l);
  Serial.print(" ");
  Serial.print(l.x);
  Serial.print(" ");
  Serial.print(l.y);
  Serial.print(" ");
  Serial.print(l.z); */
  if(norm2vort >= maxv2) {
    norm2vort = maxv2;
    normvort = MAX_SPEED;
    norm2vcoll = .0;
    normvcoll = .0;
  }
  else { //calculating square absolute value of trajectory-collinear speed component
    norm2vcoll = maxv2 - norm2vort;

    //make acc/decc treshold depends on rv and lv
    float treshold = 0.5*(1.0+(rv-lv)/(MAX_ACCEL*normq));
    if(treshold < .0) treshold = .0;
    if(treshold > 1.0) treshold = 1.0;

    //set maximum collinear speed at this trajectory point
    if(sigma > treshold) dsigma = (sigma > 1.0)?(sigma-1.0):(1.0-sigma);
    else dsigma = (sigma < .0)?(-sigma):(sigma); 
    norm2vcollmax = pow(MAX_ACCEL * 2 * normq * dsigma + MIN_SPEED + ((sigma > treshold)?rv:lv), 2);
    if(norm2vcollmax < .0) norm2vcollmax = .0;
    
    if(norm2vcoll > norm2vcollmax) norm2vcoll = norm2vcollmax;
    
    normvort = sqrt(norm2vort);
    if(normvort < ZERO_DIST) normvort = 0;
    else {
      if(normvort < MIN_SPEED) normvort = MIN_SPEED;
        normvort -= MAX_ACCEL * dt/2;
    }
    
    normvcoll = sqrt(norm2vcoll);// + (sigma>0.5)?(MIN_SPEED):(MIN_START_SPEED); 
    if(normvcoll < MIN_START_SPEED) normvcoll = MIN_START_SPEED;
   // else if(sigma>=0.5 && normvcoll < MIN_SPEED) normvcoll = MIN_SPEED;
    //if(normvcoll > MAX_SPEED) normvcoll = MAX_SPEED;
    if(0 && normvcoll < ZERO_DIST*2 && sigma>0.5) normvcoll = 0;
    else {
      if(sigma<treshold) {
       //  normvcoll += MIN_START_SPEED;
       // if(normvcoll < MIN_START_SPEED)
       //   normvcoll = MIN_START_SPEED;
       // normvcoll += MAX_ACCEL * dt/2;
      }
      else {
        //normvcoll += MIN_SPEED;
        //if(normvcoll < MIN_SPEED)
        //  normvcoll = MIN_SPEED;
        // normvcoll -= MAX_ACCEL * dt/2;
      }
    }
  }
  
/*  Serial.print("\n l : ");
  Serial.print(l.x);
  Serial.print(" ");
  Serial.print(l.y);
  Serial.print("\n q : ");
  Serial.print(q.x);
  Serial.print(" ");
  Serial.print(q.y);
*/  
/*  Serial.print("\n orts : ");
  Serial.print(normvort);
  Serial.print(" ");
  Serial.print(normvcoll);
  Serial.print("\n l : ");
  Serial.print(router_status);
*/  

  if(norml > ZERO_DIST*0.00001) l = vectorMul(l, normvort/norml);
  if(normq > ZERO_DIST*0.00001) q = vectorMul(q, normvcoll/normq * (sigma>1.0?-1.:1.));
  v = vectorSum(l, q);
  
/*
  Serial.print("\n v : ");
  Serial.print(v.x);
  Serial.print(" ");
  Serial.print(v.y);
//  delay(1000); 
*/
    
  //setting speeds for motors
  if(v.x < 0) {
    xy_motors_delay[0] = velocityToT(-v.x, 0);
    digitalWrite(xyz_motors_pins[0][2], LOW);
  }      
  else {
    xy_motors_delay[0] = velocityToT(v.x, 0);
    digitalWrite(xyz_motors_pins[0][2], HIGH);
  }
  
  if(v.y < 0) {
    xy_motors_delay[1] = velocityToT(-v.y, 1);
    digitalWrite(xyz_motors_pins[1][2], LOW);
  }      
  else {
    xy_motors_delay[1] = velocityToT(v.y, 1);
    digitalWrite(xyz_motors_pins[1][2], HIGH);
  }
  return 1;
}

//==============================================================================================================
//                      starting and stopping printhead (warning, delay() called inside)

int printhead_started = 0;

void startPrinthead() {
  if(printhead_started) return;

  digitalWrite(AUGER_PIN, HIGH); //turn auger ON
  delay(AUGER_DELAY_MS);  //wait
  digitalWrite(PUMP_PIN, HIGH); //turn pump ON
  delay(PRINT_DELAY_MS); //extrude a little

  printhead_started = 1;
}

void stopPrinthead() {
  if(!printhead_started) return;

  digitalWrite(AUGER_PIN, LOW); //turn auger OFF
  digitalWrite(PUMP_PIN, LOW); //turn pump OFF
  
  printhead_started = 0;
}


//==============================================================================================================
//                                     reading next G-code row from serial port

void proceedCommand() {
  if(!cManager.canTakeCommand()) {
    if(!printheadForceOn)
      stopPrinthead();  //holding printhead if no commands left
    return;
  }
  Command cmd = cManager.takeCommand();

  if(cmd.G != 1)
    stopPrinthead(); //holding printhead always except G1

/*   if(c_buffer[0].G==1)
  {
    if(nCommands<=0 || c_buffer[1].G !=1 ) //if there is no next point
       moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F); 
    else moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F, 1, c_buffer[1].X, c_buffer[1].Y, c_buffer[1].Z, c_buffer[1].F); 
  }
  else if(c_buffer[0].G==5)
       router.setCurrentPosition(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z);
*/
   if(cmd.G==1 || cmd.G==0) { //set new waypoints
         router_status = 2;
         
         if(cmd.G==1) startPrinthead(); //run printhead
         
         p0 = p1;
         p1 = vectorMake(cmd.X, cmd.Y, p0.z);
         
         //make projection from current position to (p0, p1)
         vector3d xyz0 = vectorMake(xyz[0], xyz[1], p0.z);
         vector3d pxyz = vectorDiff(xyz0, p0);
         float len_xyz, normp;
         p = vectorDiff(p1, p0);
         normp = sqrt(vectorScalarMul(p, p));
         len_xyz = vectorScalarMul(p, pxyz)/normp;
         if(len_xyz < .0) { //moving start point
          pxyz = vectorMul(p, len_xyz/normp);
          p0 = vectorSum(p0, pxyz);
         }
         
         cos_l = cos_r;
         lv = rv * cos_l;
         have_p2 = 0;
         if(cManager.nCommands) { //if there can be next point
            Command cmd2 = cManager.c_buffer[0];
          /*  Serial.print("E G");
            Serial.print(cmd.G);
            Serial.print("to G");
            Serial.println(cmd2.G);*/
            if(cmd2.G == cmd.G) {
              p2 = vectorMake(cmd2.X, cmd2.Y, p0.z); 
              have_p2 = 1;
              p = vectorDiff(p1, p0);
              q = vectorDiff(p2, p1);
              cos_r = vectorScalarMul(p, q)/sqrt(vectorScalarMul(p, p)*vectorScalarMul(q, q));
              if(cos_r < .0) cos_r = .0;
            }
            else
              cos_r = .0;
         }
         else cos_r = .0;
         rv = MAX_SPEED * cos_r;

         if(cmd.Z > newZ) {
            newZ = cmd.Z;
            haveNewZ = 1;
         }
      /*   if(cmd.G==1)
           Serial.print("EG1 from ");
         else
           Serial.print("EG0 from ");
         Serial.print(xyz[0]);
         Serial.print("; ");
         Serial.print(xyz[1]);
         Serial.print("; ");
         Serial.println(xyz[2]);
         Serial.print("Eto ");
         Serial.print(p1.x);
         Serial.print("; ");
         Serial.print(p1.y);
         Serial.print("; ");
         Serial.print(newZ);
         Serial.print("; ");
         Serial.println(cManager.nCommands);*/
       }
       else if(cmd.G==5) { //set current coordinates
         xyz[0] = cmd.X;
         xyz[1] = cmd.Y;
         xyz[2] = cmd.Z;
         newZ = cmd.Z;
         p0 = p1 = vectorMake(cmd.X, cmd.Y, cmd.Z);
        /* Serial.print("EG5 : ");
         Serial.print(p0.x);
         Serial.print("; ");
         Serial.print(p0.y);
         Serial.print("; ");
         Serial.println(p0.z);*/
       }
}
