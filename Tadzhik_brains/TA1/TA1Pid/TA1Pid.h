 #ifndef TA1PID
#define TA1PID

//=====================================Data structures
typedef struct dMStruct* dMStructRef;
typedef struct pidStruct* pidStructRef;

struct dMStruct {
  dMStructRef next;
  dMStructRef prev;
  float dm;
  float dt;
  float mass;
};

dMStructRef dMStructCrate(float dm, float dt, float mass);
void dMStructDestroy(dMStructRef s);
void dMStructAddStruct(dMStructRef to, dMStructRef from);
void dMStructSubstractStruct(dMStructRef to, dMStructRef from);


//----------------------------------------------------------------

struct pidStruct {
  int   max_n_dm; //maximum dm and dt observations tail length
  float kp;
  float ki;
  float kd;
  dMStructRef totalDischarge; //total dm and dt in tail
  dMStructRef dischargeHead; //oldest observation
  dMStructRef dischargeTail; //newest observation
  dMStructRef tempDischarge;
  int nDM; //dm and dt observations tail length
  unsigned long lastMillis;
  float lastMass;
  float integral;
  float targetDischarge;
  float estimatedDischarge;
  int u; //output control value
  float last_displace;
  int enabled;
};

pidStructRef pidStructCrate(int _max_n_dm, float _kp, float _ki, float _kd);
void pidStructDestroyDMStructures(pidStructRef s);
void pidStructDestroy(pidStructRef s);
void pidStructAddObservation(pidStructRef pid, dMStructRef observ);
dMStructRef pidStructLastObservation(pidStructRef pid);

//=======================================PID control
#define PUMP_F 11
#define PUMP_D 12
#define BUNKER_F 10
#define BUNKER_D 20

pidStructRef pumpPid;
pidStructRef bunkerPid;

void pidControlSetup();
int pidAddMass(pidStructRef pid, float m, unsigned long mls);
void pidCalculate(pidStructRef pid);
void pidClear(pidStructRef pid);


#endif
