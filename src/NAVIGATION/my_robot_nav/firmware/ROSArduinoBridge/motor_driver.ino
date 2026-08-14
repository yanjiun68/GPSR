/***************************************************************
   Motor driver definitions – 4 DC motors (DIR+PWM)
   Works with MDD10A or similar. No enable pins required.
 ***************************************************************/

#include "Arduino.h"
#include "motor_driver.h"

// Helper: clamp to [0..255]
// motor_driver.cpp
static inline uint8_t clamp255(int v) {
  if (v < 0)   return 0;
  if (v > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
  return (uint8_t)v;
}


// Helper: apply one motor command
static inline void applyMotor(uint8_t dirPin, uint8_t pwmPin, bool invert, int pwm) {
  bool reverse = false;
  if (pwm < 0) { pwm = -pwm; reverse = true; }
  uint8_t duty = clamp255(pwm);

  // final direction after optional inversion
  bool forwardLevel = invert ? LOW : HIGH;
  bool reverseLevel = invert ? HIGH : LOW;

  digitalWrite(dirPin, reverse ? reverseLevel : forwardLevel);
  analogWrite(pwmPin, duty);
}

void initMotorController() {
  // Direction pins
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);

  // PWM pins
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  // Safe startup: all stopped
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);

  // Default forward polarity
  digitalWrite(M1_DIR, M1_INVERT ? LOW : HIGH);
  digitalWrite(M2_DIR, M2_INVERT ? LOW : HIGH);
  digitalWrite(M3_DIR, M3_INVERT ? LOW : HIGH);
  digitalWrite(M4_DIR, M4_INVERT ? LOW : HIGH);
}

void setMotorSpeed(int index, int pwm) {
  switch (index) {
    case 0: applyMotor(M1_DIR, M1_PWM, M1_INVERT, pwm); break;
    case 1: applyMotor(M2_DIR, M2_PWM, M2_INVERT, pwm); break;
    case 2: applyMotor(M3_DIR, M3_PWM, M3_INVERT, pwm); break;
    case 3: applyMotor(M4_DIR, M4_PWM, M4_INVERT, pwm); break;
    default: /* ignore bad index */ break;
  }
}

void setMotorSpeeds(int m1, int m2, int m3, int m4) {
  setMotorSpeed(0, m1);
  setMotorSpeed(1, m2);
  setMotorSpeed(2, m3);
  setMotorSpeed(3, m4);
}

// Legacy wrapper (keeps older code compiling). Uses M1/M2 only.
void setMotorSpeeds(int left, int right) {
  setMotorSpeed(0, left);
  setMotorSpeed(1, right);
}

void brakeAll() {
  // Simple active brake: drive opposite polarity at full duty
  digitalWrite(M1_DIR, M1_INVERT ? HIGH : LOW);
  digitalWrite(M2_DIR, M2_INVERT ? HIGH : LOW);
  digitalWrite(M3_DIR, M3_INVERT ? HIGH : LOW);
  digitalWrite(M4_DIR, M4_INVERT ? HIGH : LOW);

  analogWrite(M1_PWM, MOTOR_MAX_PWM);
  analogWrite(M2_PWM, MOTOR_MAX_PWM);
  analogWrite(M3_PWM, MOTOR_MAX_PWM);
  analogWrite(M4_PWM, MOTOR_MAX_PWM);
}

void coastAll() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
} //motor_driver.ino
