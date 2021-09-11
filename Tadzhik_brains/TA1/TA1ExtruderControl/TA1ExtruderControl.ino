#include <TA1Pid.h>

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
    //delayMicroseconds(RS4800DELAY); //new start bit
    //delayMicroseconds(RS4800DELAY); //new start bit
   // if(digitalRead(RX)) break; //if no next byte, then exit
    k++;
  }

  return k;
}


float m1, m2;
unsigned long mls, prev_mls, tmp_mls;


void setup() {
  Serial.begin(9600);
  prev_mls = millis();
  rs232Setup();
  pidControlSetup();
}

void loop() {
 
  static int ff=0;
  unsigned char buff[3];
  int charN = 0;
  int command_accepted = 0;
  while(Serial.available()) {
    buff[charN] = Serial.read();
    charN++;
    if(charN == 3) {
      if(buff[0] == 'P') {
        pidClear(pumpPid);
        pumpPid->targetDischarge = buff[1];
        pumpPid->enabled = 1;
        pidClear(bunkerPid);
        bunkerPid->targetDischarge = buff[2];
        bunkerPid->enabled = 1;
      }
      else {
        pumpPid->u = buff[1];
        pumpPid->enabled = 0;
        bunkerPid->u = buff[2];
        bunkerPid->enabled = 0;
      }
      charN = 0;
      command_accepted = 1;
    }
  }

  analogWrite(PUMP_F, pumpPid->u);
  analogWrite(BUNKER_F, bunkerPid->u);

  rs232GetMasses(m1, m2);
  mls = millis();
  if(pidAddMass(pumpPid, m1, mls) || pidAddMass(bunkerPid, m2, mls) || command_accepted) {
    Serial.print(m1);
    Serial.print(" ");
    Serial.print(pumpPid->estimatedDischarge);
    Serial.print(" ");
    Serial.print(pumpPid->targetDischarge);
    Serial.print(" ");
    Serial.print(pumpPid->u);
    Serial.print(" ");
    Serial.print(m2);  
    Serial.print(" ");
    Serial.print(bunkerPid->estimatedDischarge);
    Serial.print(" ");
    Serial.print(bunkerPid->targetDischarge);
    Serial.print(" ");
    Serial.print(bunkerPid->u);
    if(mls < prev_mls)
      tmp_mls = mls + (0xFFFFFFFF - prev_mls);
    else
      tmp_mls = mls - prev_mls;
    Serial.print(" ");
    Serial.print(tmp_mls);
    Serial.print("\n");
    prev_mls = mls;
  }
}
