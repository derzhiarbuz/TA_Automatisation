#include <math.h>
#include <vector3d.h>
#include <TA1GcodeManager.h>
//-------------------------------------------------------------------------------------------------------------
//                                                data types

//-------------------------------------------------------------------------------------------------------------
//                                            telemetry variables

float xyz[3]; //current position (mm)
float Ztemp = 0;
int xyz_encode[3][2]; //values of encoder outputs (2 per encoder)
int xyz_encode_pins[3][2] = { {4, 5}, {2, 3}, {1, 0} }; //pins for rotary encoderz analog in
double xyz_encode_deltas[3] = { 0.42, 0.42, 0.42}; //value of encoder increment for each axis (mm)
//const int ENCODER_TRESHOLD = 300; //treshold for analog interpritation of 0 or 1



//-------------------------------------------------------------------------------------------------------------
//                                             control variables

float alpha = 0;
float sinAlpha = 0;
float cosAlpha = 0;
unsigned long newTime = 0;
unsigned long lastTime;

vector3d p0, p1; //beginning and end of current trajectory (line segment)
int xyz_motors_pins[3][2] = { {6, 7}, {9, 10}, {11, 12} }; //pins for motors control
const float MAX_SPEED = 300; //maximu speed mm/s
const float MIN_START_SPEED = 35; //minimum speed at acceleration mm/s
const float MIN_SPEED = 30; //minimum speed at decceleration mm/s
const float MAX_ACCEL = 600; //accel mm/s2
const float ZERO_DIST = .3; //distance, in mm, treated as zero
int router_status;

TA1GcodeManager gCodeManager;

//-------------------------------------------------------------------------------------------------------------
//                                                 functions

//init encoders arrays
void initEncoders();
void initControl();
//calculate and update control signals for motors
int proceedControl();
//reading next command from serial port
//void readGcode();

//=======================================RS232
#define RX1 8
#define TX1 9
#define RX2 6
#define TX2 7
#define RS4800DELAY 204
#define RS4800HDELAY 102

int i;
int parity;
int nData;
unsigned short rsData[5];
long mass1;
long mass2;
int strData[55];

void rs232Setup() {
  pinMode(RX1, INPUT);
  pinMode(TX1, OUTPUT);
  pinMode(RX2, INPUT);
  pinMode(TX2, OUTPUT);
  digitalWrite(TX1, HIGH);
  digitalWrite(TX2, HIGH);
}

void rs232GetMasses(float &m1, float &m2) {
  //read mass1
  rs232Write(0x45, TX1);
  nData = rs232Read(rsData, 2, RX1);
  mass1 = (((int)(rsData[1]&0x7F))*0x100+rsData[0]);
  mass1*=((rsData[1]&0x80)?(-1):1);

  //read mass2
  rs232Write(0x45, TX2);
  nData = rs232Read(rsData, 2, RX2);
  mass2 = (((int)(rsData[1]&0x7F))*0x100+rsData[0]);
  mass2*=((rsData[1]&0x80)?(-1):1);

  m1 = (float)mass1/100;
  m2 = (float)mass2/100;
}

void rs232Write(unsigned short data, int tx) {
  digitalWrite(tx, LOW);
  delayMicroseconds(RS4800DELAY);
  parity = 0x0;
  for(i=0; i<=7; i++)
  {
    if(data & (0x01<<i))
    {
      digitalWrite(tx, HIGH);
      parity++;
    }
    else
    {
      digitalWrite(tx, LOW);
    }
    delayMicroseconds(RS4800DELAY);
  }
  if(parity & 0x01) {
    digitalWrite(tx, HIGH);
  }
  else {
    digitalWrite(tx, LOW);
  }
  delayMicroseconds(RS4800DELAY);
  digitalWrite(tx, HIGH);
}

int rs232Read(unsigned short data[], int nbytes, int rx) {
  int k=0;
  while(k<nbytes) {
    while(digitalRead(rx));

    delayMicroseconds(RS4800HDELAY);
    
    data[k] = 0x0;
    for(i=0; i<=7; i++) {
      delayMicroseconds(RS4800DELAY);
      if(digitalRead(rx))
        data[k]|=0x1<<i;
    }
    delayMicroseconds(RS4800DELAY); //parity
    delayMicroseconds(RS4800DELAY); //stop bit
    k++;
  }

  return k;
}

//==============================================================================================================
unsigned long timespent;
unsigned long counter;
void setup()
{
  counter = 0;
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  
  initControl();
  initEncoders();
  
  router_status = 1;
}


void loop()
{
  if (Serial.available() > 0 && router_status==1) 
  {
     gCodeManager.readSerial();
  } 
  proceedControl();
}

//==============================================================================================================
//                                       initializing encoders input

void initEncoders() {
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<2; j++)
      pinMode(xyz_encode_pins[0][0], INPUT);
  
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][0]), encoder_x_0_rise, RISING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][0]), encoder_x_0_fall, FALLING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][1]), encoder_x_1_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[0][1]), encoder_x_1_fall, FALLING);

  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][0]), encoder_x_0_rise, RISING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][0]), encoder_x_0_fall, FALLING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][1]), encoder_x_1_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[1][1]), encoder_x_1_fall, FALLING);

  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][0]), encoder_x_0_rise, RISING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][0]), encoder_x_0_fall, FALLING); 
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][1]), encoder_x_1_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(xyz_encode_pins[2][1]), encoder_x_1_fall, FALLING);  
}

//==============================================================================================================
//                                      encoders input interruptions  

#define _0_changed(idx, val)   xyz_encode_pins[idx][0] = val;\
          xyz_encode_pins[idx][1] == val ? xyz[idx] -= xyz_encode_deltas[idx] : xyz[idx] += xyz_encode_deltas[idx];
#define _1_changed(idx, val)   xyz_encode_pins[idx][0] = val;\
          xyz_encode_pins[idx][1] == val ? xyz[idx] += xyz_encode_deltas[idx] : xyz[idx] -= xyz_encode_deltas[idx];

//===================================encoder X
void encoder_x_0_rise() {
  _0_changed(0,1)
}

void encoder_x_0_fall() {
  _0_changed(0,0)
}

void encoder_x_1_rise() {
  _1_changed(0,1)
}

void encoder_x_1_fall() {
  _1_changed(0,0)
}

//===================================encoder Y
void encoder_y_0_rise() {
  _0_changed(1,1)
}

void encoder_y_0_fall() {
  _0_changed(1,0)
}

void encoder_y_1_rise() {
  _1_changed(1,1)
}

void encoder_y_1_fall() {
  _1_changed(1,0)
}

//====================================encoder Z
void encoder_z_0_rise() {
  _0_changed(2,1)
}

void encoder_z_0_fall() {
  _0_changed(2,0)
}

void encoder_z_1_rise() {
  _1_changed(2,1)
}

void encoder_z_1_fall() {
  _1_changed(2,0)
}

//==============================================================================================================
void initControl() {
  int i, j;
  for(i=0; i<3; i++) {

    for(j=0; j<2; j++) {
      pinMode(xyz_motors_pins[i][j], OUTPUT);
    }
  }
}

//==============================================================================================================
vector3d p, c, l, q, v;
float sigma, norm2q, norm2l, normq, norml, norm2vort, normvort, norm2vcoll, normvcoll, maxv2, norm2vcollmax, dsigma;
unsigned long prev_time=0, _time;
float dt;
  
int proceedControl() {
  _time = micros();
  dt = (float)(_time - prev_time)*0.000001*0.1 + dt*0.9; //dt in seconds
  prev_time = _time;
  
  //temporary proceeding Z
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
    
  
  if(router_status == 1) return 0;
  
  p = vectorMake(xyz[0], xyz[1], 0);
  
  q = vectorDiff(p1, p0);
  norm2q = vectorScalarMul(q, q);
  if(norm2q < ZERO_DIST) {
    for(int i=0; i<2; i++)
      analogWrite(xyz_motors_pins[i][0], 0); //stopping all motors
    router_status = 1;
    return 0; //last point, no way next
  }
  normq = sqrt(norm2q);
  
  c = vectorDiff(p, p0);
  sigma = vectorScalarMul(q, c)/norm2q;
  if(abs((1.0-sigma))*normq < ZERO_DIST) {
    router_status = 1;
    Serial.print("\nS1"); //trajectory finished, sending status 1 to get next waypoint
    Serial.print(p.x);
    Serial.print(" ");
    Serial.print(p.y);
    for(int i=0; i<2; i++)
      analogWrite(xyz_motors_pins[i][0], 0); //stopping all motors
    return 1; //end of trajectory
  }
  //l is vector from current point (p) to closest point on trajectory (p0 p1)
  l = vectorDiff( vectorMul(q, sigma), c );
  norm2l = vectorScalarMul(l, l);
  if(norm2l < ZERO_DIST) {
    norm2l = .0;
    norml = .0;
  }
  else norml = sqrt(norm2l);
  
  maxv2 = MAX_SPEED * MAX_SPEED;
  
  //calculating square absolute value of trajectory-ortogonal speed component
  norm2vort = MAX_ACCEL * 2 * norml; 
  if(norm2vort < .0) norm2vort = .0;
  if(norm2vort >= maxv2) {
    norm2vort = maxv2;
    normvort = MAX_SPEED;
    norm2vcoll = .0;
    normvcoll = .0;
  }
  else { //calculating square absolute value of trajectory-collinear speed component
    norm2vcoll = maxv2 - norm2vort;
    
    if(sigma > 0.5) dsigma = (sigma > 1.0)?(sigma-1.0):(1.0-sigma);
    else dsigma = (sigma < .0)?(-sigma):(sigma);
    norm2vcollmax = MAX_ACCEL * 2 * normq * dsigma;
    if(norm2vcollmax < .0) norm2vcollmax = .0;
    
    if(norm2vcoll > norm2vcollmax) norm2vcoll = norm2vcollmax;
    
    normvort = sqrt(norm2vort);
    if(normvort < ZERO_DIST) normvort = 0;
    else {
      if(normvort < MIN_SPEED) normvort = MIN_SPEED;
        normvort -= MAX_ACCEL * dt/2;
    }
    
    normvcoll = sqrt(norm2vcoll);
    if(0 && normvcoll < ZERO_DIST*2 && sigma>0.5) normvcoll = 0;
    else {
      if(sigma<0.5) {
        normvcoll += MIN_START_SPEED;
        normvcoll += MAX_ACCEL * dt/2;
      }
      else if(sigma>=0.5) {
        normvcoll -= MAX_ACCEL * dt/2;
      }
    }
  }
    

  if(norml > ZERO_DIST) l = vectorMul(l, normvort/norml);
  if(normq > ZERO_DIST) q = vectorMul(q, normvcoll/normq * (sigma>1.0?-1.:1.));
  v = vectorSum(l, q);
    
  //setting speeds for motors
  if(v.x < 0) {
    analogWrite(xyz_motors_pins[0][0], 0.5*(-v.x));
    digitalWrite(xyz_motors_pins[0][1], HIGH);
  }      
  else {
    analogWrite(xyz_motors_pins[0][0], 0.5*(v.x));
    digitalWrite(xyz_motors_pins[0][1], LOW);
  }
  
  if(v.y < 0) {
    analogWrite(xyz_motors_pins[1][0], 0.5*(-v.y));
    digitalWrite(xyz_motors_pins[1][1], HIGH);
  }      
  else {
    analogWrite(xyz_motors_pins[1][0], 0.5*(v.y));
    digitalWrite(xyz_motors_pins[1][1], LOW);
  }

  return 1;
}
