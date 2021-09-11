const int BUFF_SIZE = 687;
int buff[BUFF_SIZE][3];

float getAngle(int val, int left, int right, float sense) {
  float angle=(val-(left+right)/2.)/(right-left)*2.;
  if(angle<-1.) angle = -1.;
  if(angle>1.) angle = 1.;
  if(angle<sense && angle>-sense) angle = .0;
  return angle;
}

void setup() {
   Serial.begin(115200);
   pinMode(2, OUTPUT);
   pinMode(3, OUTPUT);
   pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
   pinMode(7, OUTPUT);

   pinMode(8, INPUT);
   pinMode(9, INPUT);
   pinMode(10, INPUT);
   pinMode(11, INPUT);

   attachInterrupt(digitalPinToInterrupt(8), encoder_x_0_change, CHANGE); 
   attachInterrupt(digitalPinToInterrupt(9), encoder_x_1_change, CHANGE);
   attachInterrupt(digitalPinToInterrupt(10), encoder_y_0_change, CHANGE); 
   attachInterrupt(digitalPinToInterrupt(11), encoder_y_1_change, CHANGE);
}

//===================================encoder X
int Ax=0, Bx=0, nAx=0, nBx=0;
volatile float distx=.0, dDistx=0.01, oldDistx = .0;

void encoder_x_0_change() {
  nAx = digitalRead(8);
  if(nAx==Ax) return;
  Ax = nAx;
  Bx == Ax ? distx -= dDistx : distx += dDistx;
}

void encoder_x_1_change() {
  nBx = digitalRead(9);
  if(nBx==Bx) return;
  Bx = nBx;
  Ax == Bx ? distx += dDistx : distx -= dDistx;
}

int Ay=0, By=0, nAy=0, nBy=0;
volatile float disty=.0, dDisty=0.01, oldDisty = .0;

void encoder_y_0_change() {
  nAy = digitalRead(10);
  if(nAy==Ay) return;
  Ay = nAy;
  By == Ay ? disty -= dDisty : disty += dDisty;
}

void encoder_y_1_change() {
  nBy = digitalRead(11);
  if(nBy==By) return;
  By = nBy;
  Ay == By ? disty += dDisty : disty -= dDisty;
}

void loop() {
/*  // put your main code here, to run repeatedly:
  float zAng = getAngle(analogRead(2), 400, 800, 0.3);
  analogWrite(3, abs(zAng)*255);
  if(zAng<0) digitalWrite(2, LOW);
  else digitalWrite(2, HIGH);*/

  float yAng = getAngle(analogRead(1), 200, 680, 0.1);
  analogWrite(5, abs(yAng)*255);
  if(yAng<0) digitalWrite(4, LOW);
  else digitalWrite(4, HIGH);

  float xAng = getAngle(analogRead(0), 357, 675, 0.1);
  analogWrite(7, abs(xAng)*255);
  if(xAng<0) digitalWrite(6, LOW);
  else digitalWrite(6, HIGH);
  
  /*Serial.print(getAngle(analogRead(0),357,675, 0.3));
  Serial.print(" ");
  Serial.print(getAngle(analogRead(1), 200, 680, 0.3 ));
  Serial.print(" ");
  Serial.print(getAngle(analogRead(2), 400, 800, 0.3)); 
  Serial.println(" ");*/
/*  int k1, k2;
  int i;
  for(i=0; i<BUFF_SIZE; i++) {
    buff[i][0] = analogRead(4);
   // buff[i][1] = analogRead(4);
  }
  for(i=0; i<BUFF_SIZE; i++) {
      Serial.print(buff[i][0]);
      //Serial.print(" ");
      //Serial.print(buff[i][1]);
      Serial.print("\n"); 
  }
*/


/*if(idx>=100) {
  canWrite = 0;
  for(int i=0; i<100; i++) {
      Serial.print(buff[i][0]);
      Serial.print(" ");
      Serial.print(buff[i][1]);
      Serial.print(" ");
      Serial.println(buff[i][2]);
  }
    idx=0;
   canWrite = 1;
}
*/
  if((oldDistx - distx)>0.1 || (oldDistx - distx)<-0.1 || (oldDisty - disty)>0.1 || (oldDisty - disty)<-0.1)
  {
    Serial.print(distx);
    Serial.print(" ");
    Serial.println(disty);
    oldDistx = distx;
    oldDisty = disty;
  }
 // delay(100);
}
