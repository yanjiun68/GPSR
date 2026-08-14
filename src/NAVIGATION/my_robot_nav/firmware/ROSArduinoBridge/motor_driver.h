/***************************************************************
   Motor driver function definitions – 4 DC motors (DIR+PWM)
   Tested on Arduino MEGA2560

   Pin map (edit to suit your wiring):
   M1: DIR=50, PWM=7
   M2: DIR=52, PWM=6
   M3: DIR=22, PWM=5
   M4: DIR=23, PWM=4
 **********************************************************因为我们的odom不准，我在想可以使用激光定位的方法  叫 teach&repeat 方法。 但是这个方法有缺点，就是如果原路被堵死，需要重新规划全局路径的时候是做不到的，所以这个方法只能局部避障，没办法全局规划。 如果不需要改变路线，只需要原路返回  这个方法足够了。 但是我不清楚你们跟人时候的距离，会不会激光扫到人腿  会有干扰   我觉得需要实验。*****/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// ---------------- Pin assignment ----------------
#define M1_DIR 24
#define M1_PWM 4 // green board not efficienctly in contact with the arduino board. gt video of when press the green board, the motor 1 only moves. if not wont move  (pin 3 -> 4)

#define M2_DIR 25
#define M2_PWM 8 //pin 5 on the current arduino board cannot function; (pin 5 -> 8)

#define M3_DIR 26
#define M3_PWM 6

#define M4_DIR 27
#define M4_PWM 7

// ---------------- Behavior options ----------------
// Flip any of these to true if a motor spins the wrong direction
#define M1_INVERT true
#define M2_INVERT false
#define M3_INVERT false
#define M4_INVERT true

// Clamp for Arduino analogWrite
#define MOTOR_MAX_PWM 255

// API
void initMotorController();
void setMotorSpeed(int index, int pwm);                          // index: 0..3
void setMotorSpeeds(int m1, int m2, int m3, int m4);             // 4-motor
void setMotorSpeeds(int left, int right);                        // legacy 2-motor wrapper (M1/M2)
void brakeAll();                                                 // active brake (full PWM with opposite DIR)
void coastAll();                                                 // stop PWM, let motors coast

#endif  // MOTOR_DRIVER_H
