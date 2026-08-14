/***************************************************************
  4-Encoder driver (x4) for Arduino MEGA2560
  A channels on INT pins 18..21, B channels on PCINT pins 50..53

  Wiring used:
    M1: A=18, B=50
    M2: A=19, B=51
    M3: A=20, B=52
    M4: A=21, B=53

  Public API:
    void initEncoders();
    long readEncoder(int index);        // index 0..3 (M1..M4)
    void resetEncoder(int index);       // zero one
    void resetEncoders();               // zero all
    void setEncoderDirection(int idx, int8_t sign); // +1 or -1 per channel
***************************************************************/
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

// ----- Channel pin map (edit if you wire differently) -----
#define ENC1_A 18   // INT3  (PD3)
#define ENC1_B 50   // PCINT3 (PB3)

#define ENC2_A 19   // INT2  (PD2)
#define ENC2_B 51   // PCINT2 (PB2)

#define ENC3_A 20   // INT1  (PD1)
#define ENC3_B 52   // PCINT1 (PB1)

#define ENC4_A 21   // INT0  (PD0)
#define ENC4_B 53   // PCINT0 (PB0)

// Optional symbolic indices
#define ENC_M1 0
#define ENC_M2 1
#define ENC_M3 2
#define ENC_M4 3

// API
void initEncoders();
long readEncoder(int index);
void resetEncoder(int index);
void resetEncoders();
// Flip sign at runtime (CW positive by default). sign must be +1 or -1.
void setEncoderDirection(int index, int8_t sign);

#endif  // ENCODER_DRIVER_H
