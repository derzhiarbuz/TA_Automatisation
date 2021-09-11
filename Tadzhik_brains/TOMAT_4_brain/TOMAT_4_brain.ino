#include <DACNCRouter.h>

#define SPEEDS 30000
#define DIST_PER_SPEED 5.0
#define MIN_SPEED 30.0
#define MAX_SPEED 20.0
#define C_BUFFER_LENGTH 30

int cmdReceived = 1;

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

//Stepper my_stpr(200, 11, 12, 13, 10);
int stp = 0;
int dvd = 20;
int step_time = 10000;
int dist = 0;
int spd = 500;
int spdcount = 0;
int spdinc = 1;
int needSendStatus = 0;
char incomingByte = ' ';

int calltime=0;

DACNCRouter router = DACNCRouter(); 

Command c_buffer[C_BUFFER_LENGTH*2+30];
Command c1;
Command c2;
Command c3;
Command c4;
Command c5;
Command c6;
Command c7;
int nCommands = 0;

//act timer interrupr
static double maxTime = .0;
double currDMicros;
static unsigned long micros_1=0;
unsigned long micros_2;
  
void TC3_Handler()
{
  micros_1+=50;
  TC_GetStatus(TC1,0);
  router.act();
  if(router.status()!=CNCIdle)
  {
    if(micros_1>105)
    {
      Serial.print("EM ");
      Serial.println(micros_1);
    }
    micros_1=0;
  }
}

void setup()
{
   //attaching interrupt
  //this routine is for ArduinoDue (Atmel SMART SAM3X)
  pmc_set_writeprotect(false); 
  pmc_enable_periph_clk(ID_TC3);
  TC_Configure(TC1, 0, TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_TCCLKS_TIMER_CLOCK1);

  uint32_t rc = VARIANT_MCK/2/20000;

  TC_SetRA(TC1, 0, rc/2);
  TC_SetRC(TC1, 0, rc);
  TC_Start(TC1, 0);
  
  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(TC3_IRQn);
  //TCCR1A = 0;
 // TCCR1B = 0;
 // OCR1A = 4199; //for 84MHz it's once per 50us
 // TIMSK1 = bit (OCIE1A);

  
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  //Serial.print("One step accel: ");
  //Serial.println(_one_step_accel);
  //my_stpr.setSpeed(1);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  
// router.setAxisX(400, 6, 7, (18.58*3.141592/400.0), 390.0, -1, -1);
// router.setAxisY(400, 9, 10, (16.26*3.141592/400.0), 390.0, -1, -1);
  router.setAxisX(800, 3, 4, 2, (3./800.0), 390.0, -1, -1);
  router.setAxisY(800, 6, 7, 5, (3./800.0), 390.0, -1, -1);
  router.setAxisZ(800, 12, 13, 11, (3./800.0), 390.0, -1, -1);
  router.setMaxStartSpeed(1);

  router.setWorkingInstrument(10,0);
  router.setCurrentPosition(0, 0, 0); 
}

void loop()
{
  calltime++;
  //digitalWrite(8, HIGH);
  //digitalWrite(9, HIGH);
  //delay(1000);
  //digitalWrite(9, LOW);
  //digitalWrite(4, HIGH);

  // return;
  int statusBefore = router.status();
  needSendStatus = 0;
  
  //digitalWrite(11, HIGH);
   
   if (Serial.available() > 0 /*&& router.status()!=CNCProcessing*/) 
   {
     readSerial();
     
  //    if(/*router.status() != statusBefore ||*/ needSendStatus)
  //   {
        // Serial.print("S");
        // Serial.print(router.status());
        //if(nCommands < C_BUFFER_LENGTH) {
        //  Serial.write("K");
        //}
   //      needSendStatus = 0;
   //      statusBefore = router.status();
   //  }
   }
   
//   router.act();
   
  // if(router.status() == CNCIdle) {
       //if(statusBefore != CNCIdle)
       if(cmdReceived)
         if(nCommands < (C_BUFFER_LENGTH-1)) {
          cmdReceived = 0;
          Serial.print("K\n");
        }

   if(router.status() == CNCIdle) {
       proceedCommand();
       prepareCommand();
   }
   
   /*if(router.status() != statusBefore  || needSendStatus)
   {
     Serial.print("S");
     Serial.print(router.status());
   }*/
   return;
//      
//   digitalWrite(9, HIGH);
//   
//   if(spdcount>=spd)
//   {
//     digitalWrite(10, HIGH);
//     digitalWrite(7, HIGH);
//     spdcount=0;
//   }
//   else if(spdcount>=(spd/2))
//   {
//     digitalWrite(10, LOW);
//     digitalWrite(7, LOW);
//   }
//     
//   dist++;
//   
//   spdcount++;
//   
//   if(dist>=300 && 0)
//   {
////     stp+=50;
////     if(stp>=SPEEDS*2)
////     {
////       stp=0;
////       dist = 0;
////     }
//     dist = 0;
//       
//     spd += spdinc;
//     
//     if(spd>=MIN_SPEED || spd<=MAX_SPEED) spdinc*=-1;
//   }
  
}

void readSerial()
{     
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
                //Serial.write(incomingByte);
                
                if(incomingByte == '?') needSendStatus=1;
                
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
       
       //Serial.println("command");
       if(!command_finished) return;
       //Serial.println("command finished");
       
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
        
       //if(G==1 || G==5)
       //{
           scheduleCommand(G, X, Y, Z, F);  
           cmdReceived = 1;
       //}
        
       if(wasS)
       {
           router.setInstrumentPower((int)S);
       }
       
       //Serial.println(" ");
       //Serial.print(G);
       //Serial.print(" ");
       //Serial.print(X);
       //Serial.print(" ");
       //Serial.print(Y);
       //Serial.print(" ");
       //Serial.print(Z);
       //Serial.print(" ");
       //Serial.print(F);
       //Serial.println("");
       
       command_finished = 0;
       X=0; Y=0; Z=0; F=0; G=0; S=0; blank=0; point=0; mult=1.0;
       wasG=0; wasX=0; wasY=0; wasZ=0; wasF=0;
       wasS = 0;
       was_command = 0; command_finished = 0;
       current=&blank;
       needSendStatus = 1;
       //Serial.println(millis()-start_micros);
       //Serial.println("B");
}

void moveXY(float X, float Y)
{
  if(router.status() != CNCIdle) return;
  
  router.lineTo(-X/5.0, Y/5.0, 0);
  
  Serial.println(X);
  Serial.println(Y);
}

void moveXYZ(float X, float Y, float Z, float F, int next=0, float X1=.0, float Y1=.0, float Z1=.0, float F1=-1.0);

void moveXYZ(float X, float Y, float Z, float F, int next, float X1, float Y1, float Z1, float F1)
{
  router.setSpeed(F);
  if(router.status() != CNCIdle) return;
  
  router.lineTo(-X, Y, Z, next, -X1, Y1, Z1, F1); 
  
//  Serial.println("Lineto");
//  Serial.print(X);
//  Serial.print(" ");
//  Serial.print(Y);
//  Serial.print(" ");
//  Serial.print(Z);
//  Serial.println(" ");
}

void scheduleCommand(float G, float X, float Y, float Z, float F) {
  c_buffer[nCommands].setValues(G, X, Y, Z, F);
  nCommands++;
  if(nCommands<C_BUFFER_LENGTH)
  {
    Serial.print("K\n");
    //Serial.print(nCommands);
  }
}

void prepareCommand() {
  if(router.prepaired()) return;
  if(nCommands<=0) return;
  if(c_buffer[0].G==1)
  {
    if(nCommands<=1 || c_buffer[1].G !=1 ) //if there is no next point
       router.prepareLineTo(-c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F); 
    else router.prepareLineTo(-c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F, 1, -c_buffer[1].X, c_buffer[1].Y, c_buffer[1].Z, c_buffer[1].F);
  }
}

void proceedCommand() {
  if(nCommands<=0) return;
  
  //Serial.print("R");
  //Serial.print(nCommands);
  nCommands--;

 /* Serial.println(" ");
  Serial.print(c_buffer[0].G);
       Serial.print(" ");
       Serial.print(c_buffer[0].X);
       Serial.print(" ");
       Serial.print(c_buffer[0].Y);
       Serial.print(" ");
       Serial.print(c_buffer[0].Z);
       Serial.print(" ");
       Serial.print(c_buffer[0].F);*/
  
  if(c_buffer[0].G==1)
  {
    if(nCommands<=0 || c_buffer[1].G !=1 ) //if there is no next point
       moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F); 
    else moveXYZ(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z, c_buffer[0].F, 1, c_buffer[1].X, c_buffer[1].Y, c_buffer[1].Z, c_buffer[1].F); 
  }
  else if(c_buffer[0].G==5)
       router.setCurrentPosition(c_buffer[0].X, c_buffer[0].Y, c_buffer[0].Z);

  //Serial.println(" 8) ");
       
 
  for(int i=0; i<nCommands; i++) {
    c_buffer[i].G = c_buffer[i+1].G;
    c_buffer[i].X = c_buffer[i+1].X;
    c_buffer[i].Y = c_buffer[i+1].Y;
    c_buffer[i].Z = c_buffer[i+1].Z;
    c_buffer[i].F = c_buffer[i+1].F;
  }
}
