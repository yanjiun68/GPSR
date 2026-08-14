/*********************************************************************
 * ROSArduinoBridge (Hybrid version)
 * 
 * Features:
 *  - Debug console over Serial/Serial3
 *  - ROS (rosserial) optional
 *  - Motor speed control (ticks/sec)
 *  - PID tuning per-motor or global
 *********************************************************************/

#define USE_BASE
#define ROS_MODE        0         // 1 = enable rosserial on USB (Serial)
#define DEBUG_CONSOLE   1
#define DBG_ON_USB      1

#define ROS_BAUD        57600
#define DBG_BAUD        115200

#define MAX_PWM                 255
#define AUTO_STOP_INTERVAL_MS   300

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <ctype.h>   // isdigit()
#include <string.h>

#include "commands.h"
#include "sensors.h"

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"
#endif

#if ROS_MODE
  #include <ros.h>
  #include <std_msgs/Int32MultiArray.h>
  ros::NodeHandle nh;
  std_msgs::Int32MultiArray ticks_msg;
#endif

#if DEBUG_CONSOLE
  #if DBG_ON_USB
    #define DBG Serial
  #else
    #define DBG Serial3
  #endif
#endif

// ---- PID scheduler ----
static unsigned long nextPID_us = 0;   // scheduler for updatePID()
static unsigned long last_cmd_ms = 0;

// ------------ tiny cmd parser ------------
static int  arg = 0, index_ = 0;
static char chr = 0, cmd = 0;
static char argv1[32], argv2[32];
static long arg1 = 0, arg2 = 0;

static inline void resetCommand() {
  cmd = 0;
  argv1[0] = 0; argv2[0] = 0;
  arg = 0; index_ = 0; arg1 = arg2 = 0;
}

// Allow "80 0 10 100" or "80,0,10,100" or "80:0:10:100"
static void normalize_separators(char *s) {
  for (char *p = s; *p; ++p)
    if (*p==' ' || *p==',' || *p==';') *p=':';
}

// ------------ Debug console command handler ------------
static int runCommandDBG() {
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:  DBG.println(ROS_BAUD); break;
    case ANALOG_READ:   DBG.println(analogRead(arg1)); break;
    case DIGITAL_READ:  DBG.println(digitalRead(arg1)); break;

    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      DBG.println(F("OK"));
      break;

    case DIGITAL_WRITE:
      digitalWrite(arg1, arg2==0?LOW:HIGH);
      DBG.println(F("OK"));
      break;

    case PIN_MODE:
      pinMode(arg1, arg2==0?INPUT:OUTPUT);
      DBG.println(F("OK"));
      break;

    case PING_CMD:
      DBG.println(Ping(arg1));
      break;

#ifdef USE_BASE
    case READ_ENCODERS:
      DBG.print(readEncoder(M1)); DBG.print(' ');
      DBG.print(readEncoder(M2)); DBG.print(' ');
      DBG.print(readEncoder(M3)); DBG.print(' ');
      DBG.println(readEncoder(M4));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      DBG.println(F("OK"));
      break;

    case MOTOR_SPEEDS_TPS: {   // m t1:t2:t3:t4
      last_cmd_ms = millis();
      int t[4] = {0,0,0,0};
      int k = 0; char *q = argv1; char *tok;
      while (k < 4 && (tok = strtok_r(q, ":", &q)) != NULL) t[k++] = atoi(tok);

      if (t[0]==0 && t[1]==0 && t[2]==0 && t[3]==0) {
        setTargetsTPS(0,0,0,0);
        resetPID();
        coastAll();
      } else {
        setTargetsTPS(t[0], t[1], t[2], t[3]);
      }
      DBG.println(F("OK"));
      break;
    }

    case MOTOR_SPEEDS_FRAME: {   // f t1:t2:t3:t4 (legacy ticks/frame)
      last_cmd_ms = millis();
      int t[4] = {0,0,0,0};
      int k = 0; char *q = argv1; char *tok;
      while (k < 4 && (tok = strtok_r(q, ":", &q)) != NULL) t[k++] = atoi(tok);

      if (t[0]==0 && t[1]==0 && t[2]==0 && t[3]==0) {
        setTargetsTPS(0,0,0,0);
        resetPID();
        coastAll();
      } else {
        const double toTPS = PID_RATE_HZ;
        setTargetsTPS(t[0]*toTPS, t[1]*toTPS, t[2]*toTPS, t[3]*toTPS);
      }
      DBG.println(F("OK"));
      break;
    }

    case STOP_ALL: {             // s
      last_cmd_ms = millis();
      setTargetsTPS(0,0,0,0);
      resetPID();
      coastAll();
      DBG.println(F("STOP"));
      break;
    }

    case READ_PID_OUT: {         // o
      double t0,t1,t2,t3; int m0,m1,m2,m3; long u0,u1,u2,u3;
      noInterrupts();
      t0=pid[0].TargetRateTPS; t1=pid[1].TargetRateTPS;
      t2=pid[2].TargetRateTPS; t3=pid[3].TargetRateTPS;
      m0=pid[0].PrevInputRate; m1=pid[1].PrevInputRate;
      m2=pid[2].PrevInputRate; m3=pid[3].PrevInputRate;
      u0=pid[0].output; u1=pid[1].output; u2=pid[2].output; u3=pid[3].output;
      interrupts();

      DBG.print(F("T[tps]=")); DBG.print(t0,1); DBG.print(' ');
      DBG.print(t1,1); DBG.print(' '); DBG.print(t2,1); DBG.print(' ');
      DBG.println(t3,1);

      DBG.print(F("M[tps]=")); DBG.print(m0); DBG.print(' ');
      DBG.print(m1); DBG.print(' '); DBG.print(m2); DBG.print(' ');
      DBG.println(m3);

      DBG.print(F("U[pwm]=")); DBG.print(u0); DBG.print(' ');
      DBG.print(u1); DBG.print(' '); DBG.print(u2); DBG.print(' ');
      DBG.println(u3);
      break;
    }

    case 'z': {   // Print Target, Measured in RPM + PWM output
      noInterrupts();
      double t0=pid[0].TargetRateTPS, t1=pid[1].TargetRateTPS, t2=pid[2].TargetRateTPS, t3=pid[3].TargetRateTPS;
      int m0=pid[0].PrevInputRate, m1=pid[1].PrevInputRate, m2=pid[2].PrevInputRate, m3=pid[3].PrevInputRate;
      long u0=pid[0].output, u1=pid[1].output, u2=pid[2].output, u3=pid[3].output;
      interrupts();
    
      // Convert TPS → RPM
      double rpmT0 = (t0 / 3332.0) * 60.0;  // FL
      double rpmT1 = (t1 / 3332.0) * 60.0;  // FR
      double rpmT2 = (t2 / 3332.0) * 60.0;  // RR
      double rpmT3 = (t3 / 3332.0) * 60.0;  // RL

      double rpmM0 = (m0 / 3332.0) * 60.0;
      double rpmM1 = (m1 / 3332.0) * 60.0;
      double rpmM2 = (m2 / 3332.0) * 60.0;
      double rpmM3 = (m3 / 3332.0) * 60.0;
    
      // Print results
      DBG.print(F("Target [rpm]: ")); 
      DBG.print(rpmT0,1); DBG.print(' ');
      DBG.print(rpmT1,1); DBG.print(' ');
      DBG.print(rpmT2,1); DBG.print(' ');
      DBG.println(rpmT3,1);
    
      DBG.print(F("Measured[rpm]: "));
      DBG.print(rpmM0,1); DBG.print(' ');
      DBG.print(rpmM1,1); DBG.print(' ');
      DBG.print(rpmM2,1); DBG.print(' ');
      DBG.println(rpmM3,1);
    
      DBG.print(F("PWM: "));
      DBG.print(u0); DBG.print(' ');
      DBG.print(u1); DBG.print(' ');
      DBG.print(u2); DBG.print(' ');
      DBG.println(u3);
      break;
    }


    // ===== Global PID tuning =====
    case UPDATE_PID: {    // u  Kp:Kd:Ki:Ko   (accept spaces OR colons)
      char buf[32];
      const char* gstr = argv2[0] ? argv2 : argv1;
      if (!gstr[0]) { DBG.println(F("ERR (need Kp:Kd:Ki:Ko)")); break; }
      strncpy(buf, gstr, sizeof(buf)); buf[sizeof(buf)-1] = 0;
      normalize_separators(buf);

      int a[4] = {0,0,0,0};
      int i = 0; char *r = buf; char *tok;
      while (i < 4 && (tok = strtok_r(r, ":", &r)) != NULL) a[i++] = atoi(tok);

      if (i == 4) {
        setPIDGainsGlobal(a[0], a[1], a[2], a[3]);
        DBG.println(F("OK (global PID updated)"));
      } else {
        DBG.println(F("ERR (need 4 values Kp:Kd:Ki:Ko)"));
      }
      break;
    }

    // ===== Per-motor PID tuning =====
    case 'U': {   // "U2 20:0:0:100"  OR  "U 2 20:0:0:100"  OR  "U2:20:0:0:100"
      int motor = -1;
      const char* gstr = nullptr;

      // Compact "U2..." form → argv1 begins with '2'
      if (argv1[0] >= '1' && argv1[0] <= '4') {
        motor = argv1[0] - '0';
        if (argv1[1] == ':' && argv1[2] != 0) gstr = argv1 + 2; // "2:Kp:Kd:Ki:Ko"
        else if (argv2[0] != 0)               gstr = argv2;     // "2 Kp:Kd:Ki:Ko"
      }
      // Also allow "U 2 Kp:Kd:Ki:Ko"
      if (motor == -1 && argv1[0] && isdigit((unsigned char)argv1[0])) {
        motor = atoi(argv1);
        if (argv2[0]) gstr = argv2;
      }

      if (motor >= 1 && motor <= 4 && gstr && gstr[0]) {
        char buf[32]; strncpy(buf, gstr, sizeof(buf)); buf[sizeof(buf)-1] = 0;
        normalize_separators(buf);

        int a[4] = {0,0,0,0};
        int i = 0; char *r = buf; char *tok;
        while (i < 4 && (tok = strtok_r(r, ":", &r)) != NULL) a[i++] = atoi(tok);

        if (i == 4) {
          setMotorPIDGains(motor-1, a[0], a[1], a[2], a[3]);
          DBG.print(F("OK (PID motor ")); DBG.print(motor); DBG.println(F(" updated)"));
        } else {
          DBG.println(F("ERR (need 4 values Kp:Kd:Ki:Ko)"));
        }
      } else {
        DBG.println(F("ERR (usage: U2 Kp:Kd:Ki:Ko  OR  U 2 Kp:Kd:Ki:Ko  OR  U2:Kp:Kd:Ki:Ko)"));
      }
      break;
    }
#endif // USE_BASE

    default:
      DBG.println(F("Invalid Command"));
      break;
  }
  return 0;
}

#if ROS_MODE
// ---- ROS bits (disabled if ROS_MODE==0) ----
ros::Subscriber<std_msgs::Int32MultiArray> *sub_targets_ptr = nullptr;
ros::Publisher *pub_ticks_ptr = nullptr;
static unsigned long last_ticks_us = 0;

void targetsCb(const std_msgs::Int32MultiArray& msg) {
  last_cmd_ms = millis();
  if (msg.data_length == 4) setTargetsTPS(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

void publishTicks100Hz() {
  const unsigned long now = micros();
  if ((long)(now - last_ticks_us) >= 10000) {
    last_ticks_us = now;
    static int32_t data[4];
    data[0] = (int32_t)readEncoder(M1);
    data[1] = (int32_t)readEncoder(M2);
    data[2] = (int32_t)readEncoder(M3);
    data[3] = (int32_t)readEncoder(M4);
    ticks_msg.data = (int32_t*)data;
    ticks_msg.data_length = 4;
    pub_ticks_ptr->publish(&ticks_msg);
  }
}
#endif

// ======================= Setup =====================
void setup() {
#ifdef USE_BASE
  initEncoders();
  initMotorController();
  resetPID();
#endif

#if DEBUG_CONSOLE
  DBG.begin(DBG_BAUD);
#endif

#if ROS_MODE
  Serial.begin(ROS_BAUD);
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  static ros::Subscriber<std_msgs::Int32MultiArray> sub_targets("wheel_targets", &targetsCb);
  static ros::Publisher pub_ticks("wheel_ticks", &ticks_msg);
  sub_targets_ptr = &sub_targets;
  pub_ticks_ptr   = &pub_ticks;
  nh.subscribe(sub_targets);
  nh.advertise(pub_ticks);
#endif

  last_cmd_ms = millis();
  
  nextPID_us = micros() + PID_INTERVAL_US;
}

// ======================== Loop =====================
void loop() {
#if DEBUG_CONSOLE
  // -------- robust input parser (fixes U2 invalid command) --------
  while (DBG.available() > 0) {
    chr = DBG.read();

    // end of command: CR / LF / ';'
    if (chr == 13 || chr == '\n' || chr == ';') {
      if (arg == 1) argv1[index_] = 0;
      else if (arg == 2) argv2[index_] = 0;
      runCommandDBG();
      resetCommand();
      continue;
    }

    // space splits args: cmd -> argv1 -> argv2
    if (chr == ' ') {
      if (arg == 0) { arg = 1; index_ = 0; }
      else if (arg == 1) { argv1[index_] = 0; arg = 2; index_ = 0; }
      else { // arg==2: keep single spaces in argv2 if user types more text
        if (index_ < (int)sizeof(argv2)-1) argv2[index_++] = ' ';
      }
      continue;
    }

    // regular char
    if (arg == 0) {
      if (cmd == 0) cmd = chr;          // first non-space = command letter
      else {                             // subsequent chars before first space belong to argv1 (e.g., "U2")
        arg = 1;
        index_ = 0;
        if (index_ < (int)sizeof(argv1)-1) argv1[index_++] = chr;
      }
    } else if (arg == 1) {
      if (index_ < (int)sizeof(argv1)-1) argv1[index_++] = chr;
    } else { // arg == 2
      if (index_ < (int)sizeof(argv2)-1) argv2[index_++] = chr;
    }
  }
#endif

#ifdef USE_BASE
  // run PID at fixed interval, catch up if we're late
  while ((long)(micros() - nextPID_us) >= 0) {
    updatePID();
    nextPID_us += PID_INTERVAL_US;
  }
  if ((millis() - last_cmd_ms) > AUTO_STOP_INTERVAL_MS) {
    setTargetsTPS(0,0,0,0);
    resetPID();
  }
#endif

#if ROS_MODE
  publishTicks100Hz();
  nh.spinOnce();
#endif
} //ROSArduinoBridge
