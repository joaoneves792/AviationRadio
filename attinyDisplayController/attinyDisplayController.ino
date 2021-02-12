#include "TinyWireM.h"
/**
 *   Attiny85 PINS
 *             ____
 *   RESET   -|_|  |- Vcc 
 * SH_CP (3) -|    |- (2) SCL
 * ST_CP (4) -|    |- (1) DS
 *   GND     -|____|- (0) SDA
 * 
 */

 
#define SLAVE_ACTIVE_ADDRESS 0x12
#define SLAVE_STNDBY_ADDRESS 0x13
#define DISPLAY_COUNT 6
#define SEGMENT_COUNT 8
#define I2C_REGISTER_COUNT 6
#define LATCH_PIN 4
#define CLOCK_PIN 3
#define DATA_PIN 1
#define SDA_PIN 0
#define SCL_PIN 2
#define CMD_LEN 2
#define SHIFT_REGISTER_BITS 16

#define ASM_SET_PIN(PIN) {asm volatile("sbi 0x18, %0"::"M" (PIN));}
#define ASM_CLEAR_PIN(PIN) {asm volatile("cbi 0x18, %0"::"M" (PIN));}

#define latchHIGH ASM_SET_PIN(LATCH_PIN)
#define latchLOW ASM_CLEAR_PIN(LATCH_PIN)
#define clockPulse {asm volatile("sbi 0x18, %0\n\t"\
                                 "cbi 0x18, %0\n\t" ::"M" (CLOCK_PIN));}
#define shiftBit(data, b) {PORTB = (data & (1<<b))?(PORTB | B00000010):(PORTB & B11111101); clockPulse;} //Alternative method
//#define shiftBit(data, b) {if(data & (1<<b)){ASM_SET_PIN(DATA_PIN)}else{ASM_CLEAR_PIN(DATA_PIN)}; clockPulse;}
#define shiftShort(data) {\
      shiftBit(data, 0);\
      shiftBit(data, 1);\
      shiftBit(data, 2);\
      shiftBit(data, 3);\
      shiftBit(data, 4);\
      shiftBit(data, 5);\
      shiftBit(data, 6);\
      shiftBit(data, 7);\
      shiftBit(data, 8);\
      shiftBit(data, 9);\
      shiftBit(data, 10);\
      shiftBit(data, 11);\
      shiftBit(data, 12);\
      shiftBit(data, 13);\
      shiftBit(data, 14);\
      shiftBit(data, 15);\
      };

uint8_t i2cRegisters[I2C_REGISTER_COUNT];

void i2cReceive(uint8_t bytes){
  if(bytes < CMD_LEN)
    return;
  do{
    uint8_t registerPos = TinyWireS.receive();
    i2cRegisters[registerPos] = TinyWireS.receive(); //Not checking bounds on purpose, because hacking is fun ;)

  }while((bytes -= CMD_LEN) > 0);
}

void setup() {
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  for(int i=0;i<I2C_REGISTER_COUNT; i++){
    i2cRegisters[i] = 0;
  }

  //Start with 0's on all displays
  for(int i=0; i<DISPLAY_COUNT; i++){
    i2cRegisters[i] = 0b11111100;
  }


  TinyWireS.begin(SLAVE_STNDBY_ADDRESS);
  TinyWireS.onReceive(i2cReceive);
}

void loop() {
     /*
     * Segment remaping
     * 7|6|5|4|3|2|1|0  <-bits
     * A|B|C|D|E|F|G|DP
     * 7|6|4|3|5|0|1|2  <-pins
     * 0|1|3|4|2|7|6|5  <-bits
     */

  for(int i=0; i<DISPLAY_COUNT; i++){
      uint8_t remapped = 0x0;
      remapped |= (1 << 0) & (i2cRegisters[i] >> 7);
      remapped |= (1 << 1) & (i2cRegisters[i] >> 5);
      remapped |= (1 << 2) & (i2cRegisters[i] >> 1);
      remapped |= (1 << 3) & (i2cRegisters[i] >> 2);
      remapped |= (1 << 4) & (i2cRegisters[i] >> 0);
      remapped |= (1 << 5) & (i2cRegisters[i] << 5);
      remapped |= (1 << 6) & (i2cRegisters[i] << 5);
      remapped |= (1 << 7) & (i2cRegisters[i] << 5);
      
    for(int j=0; j<SEGMENT_COUNT; j++){
      latchLOW;
      shiftShort(((remapped & (1 << j)) | 0x2000 >> i));
      latchHIGH;
      delayMicroseconds(200);
      }
  }
  
  latchLOW;
  for(int j=0; j<SHIFT_REGISTER_BITS; j++){
    shiftBit(0, 0);
    clockPulse;
  }
  latchHIGH;
  
  TinyWireS_stop_check();
}
