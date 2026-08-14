  #ifndef COMMANDS_H
  #define COMMANDS_H
  
  // ----- Basic I/O commands -----
  #define ANALOG_READ        'a'
  #define GET_BAUDRATE       'b'
  #define PIN_MODE           'c'
  #define DIGITAL_READ       'd'
  #define DIGITAL_WRITE      'w'
  #define ANALOG_WRITE       'x'
  
  // ----- Robot base / sensors -----
  #define READ_ENCODERS      'e'
  #define RESET_ENCODERS     'r'
  #define PING_CMD           'p'
  
  // ----- Motor control -----
  #define MOTOR_SPEEDS_TPS   'm'   // m  <t1>:<t2>:<t3>:<t4>   (ticks/sec)
  #define MOTOR_SPEEDS_FRAME 'f'   // f  <t1>:<t2>:<t3>:<t4>   (ticks/frame)
  #define STOP_ALL           's'   // stop all motors immediately
  #define READ_PID_OUT       'o'   // dump PID targets/measured/output
  #define READ_SPEED_RPM     'z'   // print Target/Measured speed (RPM) + PWM

  
  // ----- PID tuning -----
  #define UPDATE_PID         'u'   // u Kp:Kd:Ki:Ko  (global PID)
  // NOTE: U1..U4 are handled directly in code (case 'U') — no defines needed
  
  // ----- Motor indices -----
  #define M1 0   // Front Left
  #define M2 1   // Front Right
  #define M3 2   // Rear Right
  #define M4 3   // Rear Left
  
  #endif // COMMANDS_H

  
