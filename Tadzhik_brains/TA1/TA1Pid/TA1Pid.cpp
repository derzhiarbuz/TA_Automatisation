#include "TA1Pid.h"

//=====================================Data structures

void dMStructDestroy(dMStructRef s) {
  if(s) delete [] s;
}

void dMStructAddStruct(dMStructRef to, dMStructRef from) {
  to->dm+=from->dm;
  to->dt+=from->dt;
}

void dMStructSubstractStruct(dMStructRef to, dMStructRef from) {
  to->dm-=from->dm;
  to->dt-=from->dt;
}

//--------------------------------------------------------------------


pidStructRef pidStructCrate(int _max_n_dm, float _kp, float _ki, float _kd) {
  pidStructRef newPid = new pidStruct;
  newPid->max_n_dm = _max_n_dm;
  newPid->kp = _kp;
  newPid->ki = _ki;
  newPid->kd = _kd;
  newPid->totalDischarge = dMStructCrate(.0, .0, .0);
  newPid->dischargeHead = NULL;
  newPid->dischargeTail = NULL;
  newPid->nDM = 0;
  newPid->lastMillis = millis();
  newPid->lastMass = .0;
  newPid->integral = .0;
  newPid->targetDischarge = .0;
  newPid->estimatedDischarge = .0;
  newPid->u = 0;
  newPid->last_displace = .0;
  newPid->enabled = 0;
  return newPid;
}

void pidStructDestroyDMStructures(pidStructRef s) {
  dMStructDestroy(s->totalDischarge);
  while(s->dischargeHead) {
	s->totalDischarge = s->dischargeHead->next;
	dMStructDestroy(s->dischargeHead);
	s->dischargeHead = s->totalDischarge;
  }
  s->totalDischarge = s->dischargeHead = s->dischargeTail = NULL;
}

void pidStructDestroy(pidStructRef s) {
  pidStructDestroyDMStructures(s);
  if(s) delete [] s;
}

void pidStructAddObservation(pidStructRef pid, dMStructRef observ) {
  dMStructAddStruct(pid->totalDischarge, observ);
  if(!pid->dischargeTail) { //the first observation
	pid->dischargeHead = observ;
	pid->dischargeTail = observ;
	pid->nDM++;
  }
  else {
	pid->dischargeTail->next = observ;
	observ->prev = pid->dischargeTail;
    pid->dischargeTail = pid->dischargeTail->next;
	if(pid->nDM >= pid->max_n_dm) { //remove oldest observation if there is maxNPumpDM
	  observ = pid->dischargeHead;
	  pid->dischargeHead = pid->dischargeHead->next;
	  pid->dischargeHead->prev = NULL;
	  dMStructSubstractStruct(pid->totalDischarge, observ);
	  dMStructDestroy(observ);
	}
    else
	  pid->nDM++;
  }
}

dMStructRef pidStructLastObservation(pidStructRef pid) {
  return pid->dischargeTail;
}



 //=======================================PID control


void pidControlSetup() {
  pinMode(PUMP_D, OUTPUT);
  pinMode(PUMP_F, OUTPUT);
  pinMode(BUNKER_F, OUTPUT);
  pinMode(BUNKER_D, OUTPUT);
  digitalWrite(PUMP_D, HIGH);
  pumpPid = pidStructCrate(30, 0.1, 0.5, 1.0);
  bunkerPid = pidStructCrate(15, 0.4, 0.2, 1.0);
}


int pidAddMass(pidStructRef pid, float m, unsigned long mls) {
  static float newDm;
  static float newDt;
  static unsigned int dmls;

  if(mls < pid->lastMillis) dmls = mls+(0xFFFFFFFF - pid->lastMillis);
  else dmls = mls - pid->lastMillis;
  newDt = dmls/1000.0;

  newDm = pid->lastMass-m;

  if((newDm==.0 && newDt < 5.0)) {
	return 0; //if there is no changes - do nothing
  }

  pid->lastMass = m;
  pid->lastMillis = mls;

  if((newDm<0.0 || newDm>0.06)  || (newDm/newDt * 3600)>150.0) {
	return 0; //if somebody added mass to tank - do nothing
  }

  dMStructRef newS = dMStructCrate(newDm, newDt, m);
  pidStructAddObservation(pid, newS);
  pid->estimatedDischarge = pid->totalDischarge->dm/pid->totalDischarge->dt*3600.0;

  if(pid->enabled)
    pidCalculate(pid);
  return 1;
}

void pidCalculate(pidStructRef pid) {
  static float displace;
  displace = pid->targetDischarge - pid->estimatedDischarge;
  pid->integral += displace*pid->dischargeTail->dt;
  pid->u = pid->kp * (displace + pid->ki * pid->integral + pid->kd * (displace - pid->last_displace)/pid->dischargeTail->dt);
  if(pid->u<0) pid->u=0;
  if(pid->u>255) pid->u=255;
  pid->last_displace = displace;
}

void pidClear(pidStructRef pid) {
  pid->u = 0;
  pid->integral = 0;
  pid->last_displace = 0;
  pid->targetDischarge = 0;
  pid->estimatedDischarge = 0;
  pidStructDestroyDMStructures(pid);
  pid->totalDischarge = dMStructCrate(.0, .0, .0);
}
