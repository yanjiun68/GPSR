/***************************************************************
  4-Encoder driver (x4) for Arduino MEGA2560
  - External interrupts on A channels (pins 18,19,20,21 -> INT3..INT0)
  - Pin-change interrupt on B channels (pins 50,51,52,53 -> PCINT0..3)
  - Gray-code LUT x4 decoding, CW = +, CCW = − (per-channel invertable)
***************************************************************/
#include "encoder_driver.h"

// ---------- Fast port access (Mega2560) ----------
// A pins 21..18 map to PIND bits 0..3 (PD0..PD3) → we mask lower nibble
// B pins 53..50 map to PINB bits 0..3 (PB0..PB3) → we mask lower nibble
#define READ_A_NIB()  (PIND & 0x0F)   // A4..A1 in bits [0..3] (21..18)
#define READ_B_NIB()  (PINB & 0x0F)   // B4..B1 in bits [0..3] (53..50)

// ---------- State ----------
static volatile long    enc_count[4] = {0,0,0,0};   // accumulated counts
static volatile uint8_t lastAB[4]   = {0,0,0,0};   // last 2-bit AB per encoder
static volatile uint8_t prevPINB    = 0;           // for PCINT change detect

// Per-encoder direction: +1 = CW positive (default), -1 = invert
static volatile int8_t dirSign[4] = { +1, -1, -1, +1 };

// Quadrature LUT: index = (old<<2)|new → {-1,0,+1}
static const int8_t QLUT[16] = {
/* old\new: 00  01  10  11 */
   /* 00 */  0, +1, -1,  0,
   /* 01 */ -1,  0,  0, +1,
   /* 10 */ +1,  0,  0, -1,
   /* 11 */  0, -1, +1,  0
};

// ---- Helpers: read AB (A=MSB, B=LSB) for each encoder ----
// Mapping: M1 uses PD3/PB3, M2 PD2/PB2, M3 PD1/PB1, M4 PD0/PB0
static inline uint8_t rdAB1(uint8_t pd, uint8_t pb){ return ((pd>>3)&1)<<1 | ((pb>>3)&1); } // M1
static inline uint8_t rdAB2(uint8_t pd, uint8_t pb){ return ((pd>>2)&1)<<1 | ((pb>>2)&1); } // M2
static inline uint8_t rdAB3(uint8_t pd, uint8_t pb){ return ((pd>>1)&1)<<1 | ((pb>>1)&1); } // M3
static inline uint8_t rdAB4(uint8_t pd, uint8_t pb){ return ((pd>>0)&1)<<1 | ((pb>>0)&1); } // M4

static inline void decodeStep(uint8_t i, uint8_t oldS, uint8_t newS){
  int8_t step = QLUT[(oldS<<2) | newS];
  if (step) enc_count[i] += (long)(step * dirSign[i]);
  lastAB[i] = newS;
}

// ------------------- ISRs -------------------
// A-channel ISRs: read ports and decode one encoder each
ISR(INT3_vect){ uint8_t pd=PIND, pb=PINB; decodeStep(0, lastAB[0], rdAB1(pd,pb)); } // pin 18 (M1 A)
ISR(INT2_vect){ uint8_t pd=PIND, pb=PINB; decodeStep(1, lastAB[1], rdAB2(pd,pb)); } // pin 19 (M2 A)
ISR(INT1_vect){ uint8_t pd=PIND, pb=PINB; decodeStep(2, lastAB[2], rdAB3(pd,pb)); } // pin 20 (M3 A)
ISR(INT0_vect){ uint8_t pd=PIND, pb=PINB; decodeStep(3, lastAB[3], rdAB4(pd,pb)); } // pin 21 (M4 A)

// B-channel PCINT ISR: only decode encoders whose B bit changed
ISR(PCINT0_vect){
  uint8_t pb = PINB;
  uint8_t changed = (pb ^ prevPINB) & 0x0F; // PB0..PB3
  prevPINB = pb;
  if (!changed) return;
  uint8_t pd = PIND;
  if (changed & (1<<3)) decodeStep(0, lastAB[0], rdAB1(pd,pb)); // M1 (PB3)
  if (changed & (1<<2)) decodeStep(1, lastAB[1], rdAB2(pd,pb)); // M2 (PB2)
  if (changed & (1<<1)) decodeStep(2, lastAB[2], rdAB3(pd,pb)); // M3 (PB1)
  if (changed & (1<<0)) decodeStep(3, lastAB[3], rdAB4(pd,pb)); // M4 (PB0)
}

// ------------------- init & API -------------------
void initEncoders() {
  // Inputs with pullups
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);

  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  // Prime initial AB states
  uint8_t pd = READ_A_NIB();
  uint8_t pb = READ_B_NIB();
  prevPINB   = PINB; // full register (but we only care PB0..3)
  lastAB[0]  = rdAB1(pd, pb);
  lastAB[1]  = rdAB2(pd, pb);
  lastAB[2]  = rdAB3(pd, pb);
  lastAB[3]  = rdAB4(pd, pb);

  // Enable external interrupts INT0..INT3 on ANY CHANGE (01)
  // EICRA bits: [ISC31:ISC30][ISC21:ISC20][ISC11:ISC10][ISC01:ISC00]
  EICRA = (1<<ISC30) | (1<<ISC20) | (1<<ISC10) | (1<<ISC00);
  EIFR  = (1<<INTF0) | (1<<INTF1) | (1<<INTF2) | (1<<INTF3); // clear pending
  EIMSK = (1<<INT0)  | (1<<INT1)  | (1<<INT2)  | (1<<INT3);  // enable

  // Enable pin-change interrupts for PB0..PB3 (pins 53..50)
  PCICR  |= (1 << PCIE0);                              // enable PCINT[7:0] group
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1)
          | (1 << PCINT2) | (1 << PCINT3);            // watch PB0..PB3
  PCIFR  |= (1 << PCIF0);                              // clear pending

  // Zero counts
  enc_count[0] = enc_count[1] = enc_count[2] = enc_count[3] = 0;
}

long readEncoder(int index) {
  index &= 3;
  uint8_t s = SREG; noInterrupts();
  long v = enc_count[index];
  SREG = s;
  return v;
}

void resetEncoder(int index) {
  index &= 3;
  uint8_t s = SREG; noInterrupts();
  enc_count[index] = 0;
  SREG = s;
}

void resetEncoders() {
  uint8_t s = SREG; noInterrupts();
  enc_count[0] = enc_count[1] = enc_count[2] = enc_count[3] = 0;
  SREG = s;
}

void setEncoderDirection(int index, int8_t sign) {
  index &= 3;
  if (sign != 0) {
    uint8_t s = SREG; noInterrupts();
    dirSign[index] = (sign > 0) ? +1 : -1;
    SREG = s;
  }
} //encoder_driver.ino
