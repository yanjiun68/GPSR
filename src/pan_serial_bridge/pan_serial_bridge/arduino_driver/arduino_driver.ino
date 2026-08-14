#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ===================== PINS =====================
// Encoder pins
#define ENC_A   2
#define ENC_B   3

// Motor driver pins
#define MOT_DIR 4
#define MOT_PWM 5

// Physical HOME input on Arduino digital pin 13
// Expected behavior in this version:
//   - 5V   -> HIGH
//   - GND  -> LOW
#define HOME_IN 13

// ===================== ENCODER =====================
volatile long encCount = 0;

// ===================== HOME INPUT =====================
// This is now a REAL hardware input on DIO13.
// No more virtual switch from serial commands.
volatile bool lastHomeState = false;

// ===================== SPEEDS =====================
uint8_t homePWM    = 80;
uint8_t movePWM    = 80;
uint8_t holdPWMMax = 60;
uint8_t minMovePWM = 20;

// ===================== ANGLE / COUNTS =====================
const float COUNTS_PER_REV = 244.0f;
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0f;

const float ANGLE_MIN = 0.0f;
const float ANGLE_MAX = 180.0f;

const long COUNT_MIN = 0;
const long COUNT_MAX = lround(ANGLE_MAX * COUNTS_PER_DEG);

// ===================== STATE =====================
bool homed = false;
bool holdEnabled = false;
bool stopRequested = false;
long holdTarget = 0;

// ===================== SERIAL BUFFER =====================
static char rxLine[64];
static uint8_t rxPos = 0;

// ===================== PERIODIC REPORT TIMER =====================
// Used to publish DIO13 state continuously.
unsigned long lastHomeReportMs = 0;
const unsigned long HOME_REPORT_PERIOD_MS = 100;

// ===================== FORWARD DECLARATIONS =====================
void motorTowardHome(int pwm);
void motorAwayFromHome(int pwm);
void motorStop();
bool homePressed();
long angleToCount(float deg);
float countToAngle(long c);
void printStatus();
void printHelp();
bool doHoming();
bool moveToAngle(float deg, int pwmMax);
void holdPosition();
void processIncomingSerial(bool allowMotionCommands);
bool handleCommandLine(char *line, bool allowMotionCommands);
void reportHomeInputPeriodically();

// ============================================================================
// HELPERS
// ============================================================================
bool equalsIgnoreCase(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a;
    char cb = *b;

    if (ca >= 'A' && ca <= 'Z') ca += ('a' - 'A');
    if (cb >= 'A' && cb <= 'Z') cb += ('a' - 'A');

    if (ca != cb) return false;
    ++a;
    ++b;
  }
  return (*a == '\0' && *b == '\0');
}

void trimInPlace(char *s) {
  if (!s || !*s) return;

  // Left trim: remove spaces at the beginning
  char *start = s;
  while (*start && isspace((unsigned char)*start)) start++;

  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }

  // Right trim: remove spaces at the end
  int len = strlen(s);
  while (len > 0 && isspace((unsigned char)s[len - 1])) {
    s[len - 1] = '\0';
    len--;
  }
}

// ============================================================================
// ENCODER ISR
// Home side = 0 deg
// Moving AWAY from HOME should INCREASE encCount
// If sign is opposite on your real hardware, swap ++ and -- here only.
// ============================================================================
void isrA() {
  if (digitalRead(ENC_B) == HIGH) encCount++;
  else encCount--;
}

// ============================================================================
// MOTOR HELPERS
// motorTowardHome()  -> toward switch
// motorAwayFromHome()-> away from switch
// If actual hardware direction is opposite, swap HIGH/LOW in these two only.
// ============================================================================
void motorTowardHome(int pwm) {
  digitalWrite(MOT_DIR, HIGH);
  analogWrite(MOT_PWM, constrain(pwm, 0, 255));
}

void motorAwayFromHome(int pwm) {
  digitalWrite(MOT_DIR, LOW);
  analogWrite(MOT_PWM, constrain(pwm, 0, 255));
}

void motorStop() {
  analogWrite(MOT_PWM, 0);
}

// ============================================================================
// HOME INPUT READER
// This now reads the REAL voltage on DIO13.
//
// This version expects:
//   5V   -> HIGH -> switch active
//   GND  -> LOW  -> switch inactive
//
// If you later use INPUT_PULLUP + switch-to-GND wiring,
// this function must be inverted.
// ============================================================================
bool homePressed() {
  return (digitalRead(HOME_IN) == HIGH);
}

// ============================================================================
long angleToCount(float deg) {
  deg = constrain(deg, ANGLE_MIN, ANGLE_MAX);
  long c = lround(deg * COUNTS_PER_DEG);
  return constrain(c, COUNT_MIN, COUNT_MAX);
}

float countToAngle(long c) {
  return ((float)c) / COUNTS_PER_DEG;
}

void printStatus() {
  long c;
  bool sw;

  noInterrupts();
  c = encCount;
  interrupts();

  sw = homePressed();

  Serial.print(F("encCount = "));
  Serial.print(c);
  Serial.print(F("  angle ~= "));
  Serial.print(countToAngle(c), 1);
  Serial.print(F(" deg  homeSwitch = "));
  Serial.println(sw ? F("1") : F("0"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  h               : home"));
  Serial.println(F("  a <deg>         : move to angle using default PWM"));
  Serial.println(F("  a <deg> <pwm>   : move to angle with custom PWM"));
  Serial.println(F("  p <pwm>         : set default move PWM"));
  Serial.println(F("  e               : print encoder count / angle"));
  Serial.println(F("  z               : zero encoder here (debug)"));
  Serial.println(F("  s               : stop motor"));
  Serial.println(F("  cw              : move toward home (debug)"));
  Serial.println(F("  ccw             : move away from home (debug)"));
  Serial.println(F("  hold on         : enable hold"));
  Serial.println(F("  hold off        : disable hold"));
  Serial.println(F("Example: h then a 30 40"));
  Serial.println(F("DIO13 is used as real HOME input and is reported continuously."));
}

// ============================================================================
// Read one line into fixed buffer.
// Returns true only when a non-empty line is complete.
// ============================================================================
bool serialReadLine(char *line, size_t len) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == '\r') continue;

    if (ch == '\n') {
      rxLine[rxPos] = '\0';
      rxPos = 0;

      trimInPlace(rxLine);

      if (rxLine[0] == '\0') return false;

      strncpy(line, rxLine, len);
      line[len - 1] = '\0';
      return true;
    }

    if (rxPos < sizeof(rxLine) - 1) {
      rxLine[rxPos++] = ch;
    }
  }

  return false;
}

// ============================================================================
// Process one command line.
// During motion loops, only "s" is allowed so you can stop safely.
// ============================================================================
bool handleCommandLine(char *line, bool allowMotionCommands) {
  char *tok = strtok(line, " ");
  if (!tok) return true;

  // Always allow stop
  if (equalsIgnoreCase(tok, "s")) {
    stopRequested = true;
    motorStop();
    Serial.println(F("Stopped."));
    return true;
  }

  if (!allowMotionCommands) {
    return false;
  }

  if (equalsIgnoreCase(tok, "h")) {
    stopRequested = false;
    doHoming();
    return true;
  }

  if (equalsIgnoreCase(tok, "e")) {
    printStatus();
    return true;
  }

  if (equalsIgnoreCase(tok, "z")) {
    noInterrupts();
    encCount = 0;
    holdTarget = 0;
    interrupts();

    Serial.println(F("Encoder zeroed here."));
    printStatus();
    return true;
  }

  if (equalsIgnoreCase(tok, "cw")) {
    stopRequested = false;
    motorTowardHome(60);
    Serial.println(F("Toward HOME"));
    return true;
  }

  if (equalsIgnoreCase(tok, "ccw")) {
    stopRequested = false;
    motorAwayFromHome(60);
    Serial.println(F("Away from HOME"));
    return true;
  }

  if (equalsIgnoreCase(tok, "p")) {
    char *t1 = strtok(NULL, " ");
    if (!t1) {
      Serial.println(F("Missing pwm. Example: p 80"));
      return true;
    }

    movePWM = (uint8_t)constrain(atoi(t1), 0, 255);
    Serial.print(F("movePWM = "));
    Serial.println(movePWM);
    return true;
  }

  if (equalsIgnoreCase(tok, "hold")) {
    char *t1 = strtok(NULL, " ");
    if (!t1) {
      Serial.println(F("Use: hold on  OR  hold off"));
      return true;
    }

    if (equalsIgnoreCase(t1, "on")) {
      holdEnabled = true;
      noInterrupts();
      holdTarget = encCount;
      interrupts();
      Serial.println(F("Hold enabled."));
    } else if (equalsIgnoreCase(t1, "off")) {
      holdEnabled = false;
      motorStop();
      Serial.println(F("Hold disabled."));
    } else {
      Serial.println(F("Use: hold on  OR  hold off"));
    }
    return true;
  }

  if (equalsIgnoreCase(tok, "a")) {
    char *t1 = strtok(NULL, " ");
    if (!t1) {
      Serial.println(F("Missing deg. Example: a 30"));
      return true;
    }

    float deg = atof(t1);
    char *t2 = strtok(NULL, " ");
    int pwm = movePWM;
    if (t2) pwm = constrain(atoi(t2), 0, 255);

    stopRequested = false;
    moveToAngle(deg, pwm);
    return true;
  }

  Serial.println(F("Unknown command."));
  printHelp();
  return false;
}

void processIncomingSerial(bool allowMotionCommands) {
  char line[64];
  while (serialReadLine(line, sizeof(line))) {
    handleCommandLine(line, allowMotionCommands);
  }
}

// ============================================================================
// HOMING
// - move toward physical HOME switch on DIO13
// - as soon as HOME input becomes HIGH, stop immediately
// - set encCount = 0 there
// - no backoff
// ============================================================================
bool doHoming() {
  Serial.println(F("Homing: rotate toward HOME until DIO13 becomes HIGH..."));

  if (homePressed()) {
    motorStop();

    noInterrupts();
    encCount = 0;
    holdTarget = 0;
    interrupts();

    homed = true;

    Serial.println(F("Already at HOME switch. Zeroed here."));
    printStatus();
    return true;
  }

  motorTowardHome(homePWM);

  unsigned long t0 = millis();
  while (!homePressed()) {
    processIncomingSerial(false);
    reportHomeInputPeriodically();

    if (stopRequested) {
      motorStop();
      Serial.println(F("Homing aborted."));
      return false;
    }

    if (millis() - t0 > 12000UL) {
      motorStop();
      Serial.println(F("Homing FAILED: cannot find switch."));
      return false;
    }
  }

  motorStop();
  delay(30);

  noInterrupts();
  encCount = 0;
  holdTarget = 0;
  interrupts();

  homed = true;

  Serial.println(F("Homing DONE. Stopped exactly at switch."));
  printStatus();
  return true;
}

// ============================================================================
// MOVE TO ANGLE
// Positive angle means away from home
// ============================================================================
bool moveToAngle(float deg, int pwmMax) {
  if (!homed) {
    Serial.println(F("Not homed yet. Type: h"));
    return false;
  }

  pwmMax = constrain(pwmMax, 0, 255);
  if (pwmMax < minMovePWM) pwmMax = minMovePWM;

  long target = angleToCount(deg);

  long currentCountSnapshot;
  noInterrupts();
  currentCountSnapshot = encCount;
  interrupts();

  Serial.print(F("Go to "));
  Serial.print(deg, 1);
  Serial.print(F(" deg  targetCount="));
  Serial.print(target);
  Serial.print(F("  currentCount="));
  Serial.print(currentCountSnapshot);
  Serial.print(F("  pwmMax="));
  Serial.println(pwmMax);

  const unsigned long TIMEOUT_MS = 8000UL;
  const long TOL = 1;
  const float Kp = 3.0f;

  unsigned long t0 = millis();
  unsigned long lastPrint = 0;

  while (true) {
    processIncomingSerial(false);
    reportHomeInputPeriodically();

    if (stopRequested) {
      motorStop();
      Serial.println(F("Move aborted."));
      return false;
    }

    long cur;
    noInterrupts();
    cur = encCount;
    interrupts();

    long err = target - cur;

    if (millis() - lastPrint > 150UL) {
      Serial.print(F("target="));
      Serial.print(target);
      Serial.print(F(" cur="));
      Serial.print(cur);
      Serial.print(F(" err="));
      Serial.println(err);
      lastPrint = millis();
    }

    if (labs(err) <= TOL) {
      motorStop();

      noInterrupts();
      holdTarget = target;
      interrupts();

      Serial.println(F("Reached."));
      printStatus();
      return true;
    }

    if (millis() - t0 > TIMEOUT_MS) {
      motorStop();
      Serial.println(F("Move FAILED (timeout)."));
      printStatus();
      return false;
    }

    int cmdPWM = (int)(fabs((float)err) * Kp);
    cmdPWM = constrain(cmdPWM, minMovePWM, pwmMax);

    if (err > 0) motorAwayFromHome(cmdPWM);
    else         motorTowardHome(cmdPWM);
  }
}

// ============================================================================
void holdPosition() {
  if (!holdEnabled || !homed) return;
  if (stopRequested) return;

  long cur;
  noInterrupts();
  cur = encCount;
  interrupts();

  long target = constrain(holdTarget, COUNT_MIN, COUNT_MAX);
  long err = target - cur;

  if (labs(err) <= 1) {
    motorStop();
    return;
  }

  int cmdPWM = (int)(fabs((float)err) * 2.0f);
  cmdPWM = constrain(cmdPWM, minMovePWM, holdPWMMax);

  if (err > 0) motorAwayFromHome(cmdPWM);
  else         motorTowardHome(cmdPWM);
}

// ============================================================================
// Periodically report DIO13 state.
// This gives a constant stream like:
//   DIO13=HIGH
//   DIO13=LOW
//
// This is useful for serial bridge / ROS side monitoring.
// ============================================================================
void reportHomeInputPeriodically() {
  unsigned long now = millis();
  if (now - lastHomeReportMs < HOME_REPORT_PERIOD_MS) return;

  lastHomeReportMs = now;

  bool currentState = homePressed();
  lastHomeState = currentState;

  Serial.print(F("DIO13="));
  Serial.println(currentState ? F("HIGH") : F("LOW"));
}

// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // DIO13 used as real external digital input.
  // Use INPUT because you said you want to detect:
  //   - 5V high level
  //   - GND low level
  pinMode(HOME_IN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, RISING);

  pinMode(MOT_DIR, OUTPUT);
  pinMode(MOT_PWM, OUTPUT);
  motorStop();

  printHelp();
  Serial.println(F("Ready. Type 'h' to home."));
  Serial.println(F("HOME input now comes from physical DIO13."));
  Serial.println(F("DIO13 state will be reported continuously."));
}

// ============================================================================
void loop() {
  processIncomingSerial(true);
  holdPosition();
  reportHomeInputPeriodically();
}