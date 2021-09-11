#include <math.h>
#include <vector3d.h>

//======================================GCode Manager=========================================================

const int C_BUFFER_LENGTH = 5;

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


class CommandManager {
  public:
  int nCommands;
  Command c_buffer[C_BUFFER_LENGTH+5];

  CommandManager() : nCommands(0){}
  void readSerial();
  int canScheduleCommand();
  void scheduleCommand(float G, float X, float Y, float Z, float F);
  int canTakeCommand();
  Command takeCommand();
};

void CommandManager::readSerial()
{
  static char incomingByte = ' ';
  static float lastG = 0;
  static float lastX = 0;
  static float lastY = 0;
  static float lastZ = 0;
  static float lastF = 0;
  static float X=0, Y=0, Z=0, F=0, G=0, S=0, blank=0, *current, point=0, mult=1.0;
  static int wasG=0, wasX=0, wasY=0, wasZ=0, wasF=0;
  static int wasS = 0;
  static int was_command = 0, command_finished = 0, terminated = 0;

  static unsigned int start_micros;

       if(Serial.available() > 0 && !command_finished)
       {
        incomingByte = Serial.read();

        //if(incomingByte == '?');

                if (incomingByte !='\n' && incomingByte != ';') {
                  terminated = 0;
                }


                if(incomingByte>='0' && incomingByte<='9')
                {

                  (*current)*=10;
                  (*current)+=(float)(incomingByte-'0');
                  point*=10.0;
                }
                else if(incomingByte == '-')
                {
                  mult=-1.0;
                }
                else if (incomingByte == '.' || incomingByte == ',')
                {
                  point = 1.0;
        }
        else
                {
                  if(point>0)
                      (*current)/=point;
                    point = .0;
          (*current)*=mult;
          mult=1.0;

          current=&blank;
          (*current)=0;

          if(incomingByte == 'x' || incomingByte == 'X')
          {
                  current = &X;
                  point = 0;
                  wasX=1;
                  }
                  else if (incomingByte == 'y' || incomingByte == 'Y')
                  {
                  current = &Y;
                  point = 0;
                  wasY=1;
                  }
                  else if (incomingByte == 'z' || incomingByte == 'Z')
                  {
                  current = &Z;
                  point = 0;
                  wasZ=1;
                  }
                  else if (incomingByte == 'f' || incomingByte == 'F')
                  {
                  current = &F;
                  point = 0;
                  wasF=1;
                  }
                  else if (incomingByte == 'g' || incomingByte == 'G')
                  {
                  current = &G;
                  point = 0;
                  was_command = 1;
                  wasG=1;
                  //start_micros = millis();
                  }
                   else if (incomingByte == 's' || incomingByte == 'S')
                  {
                  current = &S;
                  point = 0;
                  wasS = 1;
                  }
                  else if (incomingByte == '\n' || incomingByte == ';')
                  {
                      if(!terminated) {
                        command_finished = 1;
                      }
                      terminated = 1;
                     // was_command = 0;
                  }
                }
       }

     if(!command_finished) return;

       if(point>0)
          (*current)/=point;
        (*current)*=mult;

       if(wasG) lastG = G;
       else G = lastG;
       if(wasX) lastX = X;
       else X = lastX;
       if(wasY) lastY = Y;
       else Y = lastY;
       if(wasZ) lastZ = Z;
       else Z = lastZ;
       if(wasF) lastF = F;
       else F = lastF;

     scheduleCommand(G, X, Y, Z, F);

       if(wasS)
       {
          // router.setInstrumentPower((int)S);
     }

     command_finished = 0;
     X=0; Y=0; Z=0; F=0; G=0; S=0; blank=0; point=0; mult=1.0;
       wasG=0; wasX=0; wasY=0; wasZ=0; wasF=0;
       wasS = 0;
       was_command = 0; command_finished = 0;
     current=&blank;
}


int CommandManager::canScheduleCommand() {
  if(nCommands<C_BUFFER_LENGTH) return 1;
    return 0;
}

void CommandManager::scheduleCommand(float G, float X, float Y, float Z, float F) {
  c_buffer[nCommands].setValues(G, X, Y, Z, F);
  nCommands++;
  if(nCommands<C_BUFFER_LENGTH)
  {
  Serial.println("K");
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
    c_buffer[i].G = c_buffer[i+1].G;
    c_buffer[i].X = c_buffer[i+1].X;
    c_buffer[i].Y = c_buffer[i+1].Y;
    c_buffer[i].Z = c_buffer[i+1].Z;
    c_buffer[i].F = c_buffer[i+1].F;
  }

    return retCom;
}

//============================================================================================================


//-------------------------------------------------------------------------------------------------------------
//                                                data types

//-------------------------------------------------------------------------------------------------------------
//                                            telemetry variables

float xyz[3]; //current position (mm)
float newZ = 0;
int haveNewZ = 0;
int xyz_encode[3][2]; //values of encoder outputs (2 per encoder)
int xyz_encode_pins[3][2] = { {8, 9}, {10, 11}, {12, 13} }; //pins for rotary encoderz analog in
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
int have_p2 = 0;
int xyz_motors_pins[3][2] = { {7, 6}, {5, 4}, {3, 2} }; //pins for motors control
float lv = .0; //start speed depends on angle between prev and current trajectories
float rv = .0; //finish speed depends on angle between current and next trajectories
float cos_l = .0;
float cos_r = .0;
const float MAX_SPEED = 160; //maximu speed mm/s
const float MIN_START_SPEED = 60; //minimum speed at acceleration mm/s
const float MIN_SPEED = 30; //minimum speed at decceleration mm/s
const float MAX_ACCEL = 140; //accel mm/s2
const float ZERO_DIST = .3; //distance, in mm, treated as zero
const float MIN_ORT_SPEED_2 = 0; //minimum ortogonal speed to move
int router_status;


//-------------------------------------------------------------------------------------------------------------
//                                                 functions

//empirical function that returns U for given V
int velocityToU(float vel) {
  int u;
  u = 0.5*vel;
  if(u<0) u=0;
  if(u>255) u=255;
 // if(vel<MIN_SPEED) return .0;
  //return  0.5*(1.0+(MAX_SPEED-vel)*(MAX_SPEED-vel)/(MAX_SPEED*MAX_SPEED)*1.5)*vel;
  //return  0.5*(1.0+3.0*exp((MIN_SPEED-vel)/200.0))*vel;
  return u;
 }
//init encoders arrays
void initEncoders();
//read encoders and update distances.
//ISR(TIMER2_COMPA_vect);
//init motors arrays
void initControl();
//calculate and update control signals for motors
int proceedControl();

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
  initEncoders();
  
  router_status = 1;
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
   
   proceedControl();
   
   if(router_status == 1) {
       if(statusBefore != 1)
         if(cManager.canScheduleCommand()) {
          Serial.println("K");
        }
       proceedCommand();
   }

   static int meas_timer = 0;
   static int new_mt = 0;
   new_mt = millis();
   /*if(new_mt-meas_timer > 1000)
   {
      Serial.print(xyz[0]);
      Serial.print("; ");
      Serial.print(xyz[1]);
      Serial.print("; ");
      Serial.println(xyz[2]);
      meas_timer = new_mt;
   }*/
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

#define _changed(idx, chan) static int tmp;\
          for(int i=0; i<20; i++);\
          tmp = digitalRead(xyz_encode_pins[idx][chan]);\
          if(tmp == xyz_encode[idx][chan]) return;\ 
          xyz_encode[idx][chan] = tmp;\ 
          if(chan)\
            xyz_encode[idx][0] == tmp ? xyz[idx] += xyz_encode_deltas[idx] : xyz[idx] -= xyz_encode_deltas[idx];\
          else\
            xyz_encode[idx][1] == tmp ? xyz[idx] -= xyz_encode_deltas[idx] : xyz[idx] += xyz_encode_deltas[idx];\


//===================================encoder X

void encoder_x_0_change() {
  _changed(0,0);
}

void encoder_x_1_change() {
  _changed(0,1);
}

//===================================encoder Y

void encoder_y_0_change() {
  _changed(1,0);
}

void encoder_y_1_change() {
  _changed(1,1);
}

//====================================encoder Z

void encoder_z_0_change() {
  _changed(2,0);
}

void encoder_z_1_change() {
  _changed(2,1);
}


//==============================================================================================================
void initControl() {
  int i, j;
  for(i=0; i<3; i++) {

    for(j=0; j<2; j++) {
      pinMode(xyz_motors_pins[i][j], OUTPUT);
      digitalWrite(xyz_motors_pins[i][j], 0);
    }
  }
}

//==============================================================================================================
vector3d p, c, l, q, v, c2, q2;
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
      analogWrite(xyz_motors_pins[2][0], 255);
    }
    else {
      analogWrite(xyz_motors_pins[2][0], 0);
      haveNewZ = 0;
      Serial.print("\nP"); //trajectory finished, sending status 1 to get next waypoint
      Serial.print(xyz[0]);
      Serial.print(" ");
      Serial.print(xyz[1]);
      Serial.print(" ");
      Serial.println(xyz[2]);
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
      analogWrite(xyz_motors_pins[i][0], 0); //stopping all motors
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
        analogWrite(xyz_motors_pins[i][0], 0); //stopping all motors
    }
      Serial.print("\nP"); //trajectory finished, sending status 1 to get next waypoint
      Serial.print(xyz[0]);
      Serial.print(" ");
      Serial.print(xyz[1]);
      Serial.print(" ");
      Serial.println(xyz[2]);
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
    //if(normvcoll > MAX_SPEED) normvcoll = MAX_SPEED;
    if(0 && normvcoll < ZERO_DIST*2 && sigma>0.5) normvcoll = 0;
    else {
      if(sigma<treshold) {
       //  normvcoll += MIN_START_SPEED;
       // if(normvcoll < MIN_START_SPEED)
       //   normvcoll = MIN_START_SPEED;
        normvcoll += MAX_ACCEL * dt/2;
      }
      else {
        //normvcoll += MIN_SPEED;
        //if(normvcoll < MIN_SPEED)
        //  normvcoll = MIN_SPEED;
        normvcoll -= MAX_ACCEL * dt/2;
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

  if(norml > ZERO_DIST) l = vectorMul(l, normvort/norml);
  if(normq > ZERO_DIST) q = vectorMul(q, normvcoll/normq * (sigma>1.0?-1.:1.));
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
    analogWrite(xyz_motors_pins[0][0], velocityToU(-v.x));
    digitalWrite(xyz_motors_pins[0][1], HIGH);
  }      
  else {
    analogWrite(xyz_motors_pins[0][0], velocityToU(v.x));
    digitalWrite(xyz_motors_pins[0][1], LOW);
  }
  
  if(v.y < 0) {
    analogWrite(xyz_motors_pins[1][0], velocityToU(-v.y)*11.0/15.0);
    digitalWrite(xyz_motors_pins[1][1], HIGH);
  }      
  else {
    analogWrite(xyz_motors_pins[1][0], velocityToU(v.y)*11.0/15.0);
    digitalWrite(xyz_motors_pins[1][1], LOW);
  }

  /*if(v.z < 0) {
    //analogWrite(xyz_motors_pins[2][0], 0);
    analogWrite(xyz_motors_pins[2][0], 100);
    digitalWrite(xyz_motors_pins[2][1], HIGH);
  }      
  else {
    //analogWrite(xyz_motors_pins[2][0], 0);
    analogWrite(xyz_motors_pins[2][0], 100);
    digitalWrite(xyz_motors_pins[2][1], LOW);
  }*/

  /*if(p1.z < p0.z-(1e-12)) {
    //analogWrite(xyz_motors_pins[2][0], 0);
    if(xyz[2] > p1.z) {
      analogWrite(xyz_motors_pins[2][0], 100);
      digitalWrite(xyz_motors_pins[2][1], HIGH);
    }
    else {
      analogWrite(xyz_motors_pins[2][0], 0);
    }
  }      
  else if (p1.z > p0.z+(1e-12)) {
    //analogWrite(xyz_motors_pins[2][0], 0);
    if(xyz[2] < p1.z) {
      analogWrite(xyz_motors_pins[2][0], 100);
      digitalWrite(xyz_motors_pins[2][1], LOW);
    }
    else {
      analogWrite(xyz_motors_pins[2][0], 0);
    }
  }
  else
    analogWrite(xyz_motors_pins[2][0], 0);
*/
  //setting extrusion TEMPORARY USING Z PIN
/*  float flow = 0.5*sqrt(v.x*v.x + v.y*v.y);
  if(flow>MIN_START_SPEED*0.5) {
   analogWrite(xyz_motors_pins[2][0], 0.5*0.7*flow);
   digitalWrite(xyz_motors_pins[2][1], HIGH);
  }
  else {
    analogWrite(xyz_motors_pins[2][0], 0);
  }*/
  

  return 1;

/* sinAlpha = sin(alpha);
  cosAlpha = cos(2*alpha);
  if(sinAlpha > 0) {
    analogWrite(xyz_motors_pins[1][0], 100*sinAlpha);
    digitalWrite(xyz_motors_pins[1][1], LOW);
  }
  else {
    analogWrite(xyz_motors_pins[1][0], 100*(-sinAlpha));
    digitalWrite(xyz_motors_pins[1][1], HIGH);
  }
  
  if(cosAlpha > 0) {
    analogWrite(xyz_motors_pins[0][0], 100*cosAlpha);
    digitalWrite(xyz_motors_pins[0][1], LOW);
  }
  else {
    analogWrite(xyz_motors_pins[0][0], 100*(-cosAlpha));
    digitalWrite(xyz_motors_pins[0][1], HIGH);
  }
  
  alpha+=0.0032;
  delay(10);
  if (alpha>360) alpha = 0;
*/
}


//==============================================================================================================
//                                     reading next G-code row from serial port

void proceedCommand() {
  if(!cManager.canTakeCommand()) return;
  Command cmd = cManager.takeCommand();

/*   if(c_buffer[0].G==1)
  {
    if(nCommands<=0 || c_buffer[1].G !=1 ) //if there is no next point
       moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F); 
    else moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F, 1, c_buffer[1].X, c_buffer[1].Y, c_buffer[1].Z, c_buffer[1].F); 
  }
  else if(c_buffer[0].G==5)
       router.setCurrentPosition(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z);
*/
   if(cmd.G==1) { //set new waypoints
         router_status = 2;
         p0 = p1;
         p1 = vectorMake(cmd.X, cmd.Y, p0.z);
         cos_l = cos_r;
         lv = rv * cos_l;
         have_p2 = 0;
         if(cManager.nCommands) { //if there can be next point
            Command cmd2 = cManager.c_buffer[0];
            if(cmd2.G == 1) {
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
         Serial.print("EG1 from ");
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
         Serial.println(newZ);
       }
       else if(cmd.G==5) { //set current coordinates
         xyz[0] = cmd.X;
         xyz[1] = cmd.Y;
         xyz[2] = cmd.Z;
         newZ = cmd.Z;
         p0 = p1 = vectorMake(cmd.X, cmd.Y, cmd.Z);
         Serial.print("EG5 : ");
         Serial.print(p0.x);
         Serial.print("; ");
         Serial.print(p0.y);
         Serial.print("; ");
         Serial.println(p0.z);
       }
}
