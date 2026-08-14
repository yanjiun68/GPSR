/* *************************************************************
   diff_controller.h — 4-motor PID speed controller (TPS-based)
   - Real dt each cycle (micros)
   - Targets in ticks/sec
   - Per-motor feedforward + PID
   - Per-wheel trim (front-left/right, rear-right/left)
   - Slew-rate limit + startup kick
************************************************************* */

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include <Arduino.h>

/* ---- AVR-safe rounding helpers ---- */
static inline long  rnd_long(double x){ return (long)(x>=0.0?x+0.5:x-0.5); }
static inline int   rnd_int (double x){ return (int) (x>=0.0?x+0.5:x-0.5); }

/* ---- External functions ---- */
long readEncoder(int index);
void setMotorSpeed(int index, int spd);

/* ---- Safety ---- */
#ifndef MAX_PWM
#define MAX_PWM 255
#endif

/* ---- PID scheduler ---- */
#ifndef PID_RATE_HZ
#define PID_RATE_HZ 100
#endif
const unsigned long PID_INTERVAL_US = 1000000UL / PID_RATE_HZ;
static unsigned long lastUpdate_us = 0;

/* ---- Slew / startup ---- */
#ifndef SLEW_PWM_PER_SEC
#define SLEW_PWM_PER_SEC 250.0
#endif
#ifndef STARTUP_KICK_PWM
#define STARTUP_KICK_PWM 28
#endif

/* ---- Measured-rate LPF ---- */
#ifndef RATE_LP_ALPHA
#define RATE_LP_ALPHA 0.50
#endif

/* ---- Feed-Forward (TPS -> PWM) ---- */
#ifndef USE_FEED_FORWARD
#define USE_FEED_FORWARD 1
#endif

/* ============================================================
   Per-wheel trim (multiply final command before clamp)
   Order: [0]=FL, [1]=FR, [2]=RR, [3]=RL
   1.00 = no change; 1.02 = +2% more PWM; 0.98 = -2% less PWM
   Tune these if robot veers even when M≈T on straight runs.
   Example to pull front slightly right & back slightly left:
     TRIM = {1.02, 1.02, 0.98, 0.98};
   ============================================================ */
// keep your trims as requested
static float TRIM[4] = { 1.00f, 1.00f, 1.00f, 1.00f };
static inline void setTrim(int i, float v){ if (i>=0 && i<4) TRIM[i]=v; }
static inline float getTrim(int i){ return (i>=0 && i<4)?TRIM[i]:1.0f; }

/* M[tps] ≈ a_ff[i] * U[pwm] + b_ff[i] */
static float a_ff[4] = {
  9.343666f,  // FL
  9.774457f,  // FR
  9.803400f,  // RR
  9.737601f   // RL
};
static float b_ff[4] = {
 -51.582554f, // FL
 -66.238366f, // FR
 -76.556757f, // RR
 -68.768502f  // RL
};
/* Seeds only (MIN_PWM_FLOOR=12 still dominates very low TPS) */
static int dead_ff[4] = { 9, 8, 9, 10 };







/* Feed-forward function: TPS target -> PWM estimate (int) */
static inline int ff_pwm(int i, double target_tps){
#if USE_FEED_FORWARD
  // Linear FF (TPS per PWM)
  double u = (target_tps - (double)b_ff[i]) / (double)a_ff[i];

  // --- target-scaled deadzone ---
  double mag   = fabs(target_tps);
  double scale = mag / 300.0;              // full deadzone by ~300 TPS
  if (scale > 1.0) scale = 1.0;
  int dz = rnd_int((double)dead_ff[i] * scale);

  if (u > 0 && u <  dz) u =  dz;
  if (u < 0 && u > -dz) u = -dz;

  // --- low-speed PWM floor ---
  // below this target, guarantee at least MIN_PWM_FLOOR of push
  const double MIN_LOW_TPS    = 280.0;     // apply floor for |target| ≤ 150 TPS
  const double MIN_PWM_FLOOR  = 20.0;      // small but enough to move the wheel

  if (mag > 0.0 && mag <= MIN_LOW_TPS) {
    double au = fabs(u);
    if (au < MIN_PWM_FLOOR) u = (u >= 0 ? +MIN_PWM_FLOOR : -MIN_PWM_FLOOR);
  }

  return rnd_int(u);
#else
  (void)i; (void)target_tps; return 0;
#endif
}

/* ---- Controller state ---- */
typedef struct {
  double TargetRateTPS;
  long   Encoder;
  long   PrevEnc;
  int    PrevInputRate;
  long   ITerm;
  long   output;
} SetPointInfo;

#define NUM_MOTORS 4
static SetPointInfo pid[NUM_MOTORS];
static bool moving = false;

/* ---- PID gains ---- (your locked globals; per-motor left at 0) */
static int Kp_global = 60, Kd_global = 30, Ki_global = 4, Ko_global = 100;
static int Kp_[NUM_MOTORS] = {60,60,60,60};
static int Kd_[NUM_MOTORS] = {30,30,30,30};
static int Ki_[NUM_MOTORS] = {4,4,4,4};
static int Ko_[NUM_MOTORS] = {100,100,100,100};

static inline int gainKp(int i){ return Kp_[i] ? Kp_[i] : Kp_global; }
static inline int gainKd(int i){ return Kd_[i] ? Kd_[i] : Kd_global; }
static inline int gainKi(int i){ return Ki_[i] ? Ki_[i] : Ki_global; }
static inline int gainKo(int i){ int g = Ko_[i]?Ko_[i]:Ko_global; return g==0?100:g; }

static inline void setPIDGainsGlobal(int kp,int kd,int ki,int ko){
  Kp_global=kp; Kd_global=kd; Ki_global=ki; Ko_global=(ko==0)?100:ko;
}
static inline void setMotorPIDGains(int idx,int kp,int kd,int ki,int ko){
  if (idx<0 || idx>=NUM_MOTORS) return;
  Kp_[idx]=kp; Kd_[idx]=kd; Ki_[idx]=ki; Ko_[idx]=(ko==0)?100:ko;
}

/* ---- Targets ---- */
static inline void setTargetsTPS(double t1,double t2,double t3,double t4){
  pid[0].TargetRateTPS=t1; pid[1].TargetRateTPS=t2;
  pid[2].TargetRateTPS=t3; pid[3].TargetRateTPS=t4;
  moving = (t1||t2||t3||t4);
}
static inline void setTargets(double f1,double f2,double f3,double f4){
  const double toTPS = PID_RATE_HZ; setTargetsTPS(f1*toTPS,f2*toTPS,f3*toTPS,f4*toTPS);
}

/* ---- Helpers ---- */
static inline void zeroOne(SetPointInfo &p, long encNow){
  p.TargetRateTPS=0.0; p.Encoder=encNow; p.PrevEnc=encNow;
  p.PrevInputRate=0; p.ITerm=0; p.output=0;
}
static inline void resetPID(){
  for(int i=0;i<NUM_MOTORS;i++) zeroOne(pid[i], readEncoder(i));
  lastUpdate_us=0;
}

/* ---- One-motor step ---- */
static inline void doPIDOne(int i, SetPointInfo &p, double dt){
  const long encNow = p.Encoder;
  const long dEnc   = encNow - p.PrevEnc;
  const double inputRate_d = (dt>0.0)? (double)dEnc/dt : 0.0;

  int inputRate = rnd_int(RATE_LP_ALPHA*inputRate_d +
                          (1.0-RATE_LP_ALPHA)*(double)p.PrevInputRate);

  const long Perror = rnd_long(p.TargetRateTPS - (double)inputRate);

  long u_ff  = ff_pwm(i, p.TargetRateTPS);
  long u_inc = ((long)gainKp(i)*Perror
               - (long)gainKd(i)*(inputRate - p.PrevInputRate)
               + p.ITerm) / gainKo(i);

  long u_desired = rnd_long(0.95 * (double)u_ff) + u_inc; // K_FF = 0.95

  // Slew-rate limit
  double maxStep = SLEW_PWM_PER_SEC*dt; if (maxStep<1.0) maxStep=1.0;
  long diff = u_desired - p.output;
  if      (diff >  (long)maxStep) u_desired = p.output + (long)maxStep;
  else if (diff < -(long)maxStep) u_desired = p.output - (long)maxStep;

  // Startup kick
  if (STARTUP_KICK_PWM>0 && p.TargetRateTPS!=0.0 && p.PrevInputRate==0){
    if (u_desired>0 && u_desired< STARTUP_KICK_PWM) u_desired= STARTUP_KICK_PWM;
    if (u_desired<0 && u_desired>-STARTUP_KICK_PWM) u_desired=-STARTUP_KICK_PWM;
  }

  // ---- Apply per-wheel TRIM *before* clamp ----
  long u = rnd_long((double)u_desired * (double)TRIM[i]);

  // Saturate and anti-windup condition
  bool atHigh = (u >=  MAX_PWM);
  bool atLow  = (u <= -MAX_PWM);
  if (atHigh) u =  MAX_PWM;
  if (atLow)  u = -MAX_PWM;

  bool allowI = (!atHigh && !atLow) || (atHigh && Perror<0) || (atLow && Perror>0);
  if (allowI){
    double I = (double)p.ITerm + (double)gainKi(i) * (double)Perror * dt;
    const double Imax=3000.0; if (I> Imax) I= Imax; if (I<-Imax) I=-Imax;
    p.ITerm = rnd_long(I);
  }

  p.output=u; p.PrevInputRate=inputRate; p.PrevEnc=encNow;
}

/* ---- Main update ---- */
static inline void updatePID(){
  const unsigned long now = micros();
  double dt = (lastUpdate_us==0)? 1.0/(double)PID_RATE_HZ
                                : ((now-lastUpdate_us)>0 ? ((double)(now-lastUpdate_us))/1e6
                                                         : 1.0/(double)PID_RATE_HZ);
  lastUpdate_us = now;

  for(int i=0;i<NUM_MOTORS;i++) pid[i].Encoder = readEncoder(i);

  if (!moving){
    bool any=false; for(int i=0;i<NUM_MOTORS;i++) if (pid[i].output!=0 || pid[i].PrevInputRate!=0){ any=true; break; }
    if (any){
      for(int i=0;i<NUM_MOTORS;i++){
        pid[i].ITerm=0; pid[i].output=0; pid[i].PrevInputRate=0; pid[i].PrevEnc=pid[i].Encoder;
        setMotorSpeed(i,0);
      }
    }
    return;
  }

  for(int i=0;i<NUM_MOTORS;i++){ doPIDOne(i,pid[i],dt); setMotorSpeed(i,(int)pid[i].output); }
}
#endif
