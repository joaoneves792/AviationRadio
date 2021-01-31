#include "TinyWireS.h"
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

uint8_t i2cRegisters[I2C_REGISTER_COUNT];

void i2cReceive(uint8_t bytes){
  if(bytes < CMD_LEN)
    return;
  do{
    uint8_t registerPos = (TinyWireS.receive() + 3 )% 6; //Swap the order of the displays
    i2cRegisters[registerPos] = TinyWireS.receive();

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


  TinyWireS.begin(SLAVE_ACTIVE_ADDRESS);
  TinyWireS.onReceive(i2cReceive);
}

void loop() {

  for(int i=0; i<DISPLAY_COUNT; i++){
      /*
       * Segment remaping
       * 7|6|5|4|3|2|1|0  <-bits
       * A|B|C|D|E|F|G|DP
       * 7|6|4|3|5|0|1|2  <-pins
       * 0|1|3|4|2|7|6|5  <-bits
       */
      //Compiler does this better than me, so let it stay in C
      uint8_t remapped = 0x0;
      remapped |= (1 << 0) & (i2cRegisters[i] >> 7);
      remapped |= (1 << 1) & (i2cRegisters[i] >> 5);
      remapped |= (1 << 2) & (i2cRegisters[i] >> 1);
      remapped |= (1 << 3) & (i2cRegisters[i] >> 2);
      remapped |= (1 << 4) & (i2cRegisters[i] >> 0);
      remapped |= (1 << 5) & (i2cRegisters[i] << 5);
      remapped |= (1 << 6) & (i2cRegisters[i] << 5);
      remapped |= (1 << 7) & (i2cRegisters[i] << 5);
/* Old way (9-10 Clock cycles)
      //Bit 0
      "sbi 0x18, %[data_pin]\n\t"  
      "sbrs %[lsb], 0\n\t"
      "cbi 0x18, %[data_pin]\n\t"
      "sbi 0x18, %[clock_pin]\n\t"
      "cbi 0x18, %[clock_pin]\n\t"
  New way (5 clock cycles)
      //Bit 0
      "bst %[segment], 0\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
*/    

    register uint8_t segment;
    register uint8_t portb;
    register uint8_t us;
    asm volatile(
      
      "cbi 0x18, %[latch_pin]\n\t" //latchLOW
      
      "mov __tmp_reg__, __zero_reg__\n\t"
      "inc __tmp_reg__\n\t" //Set tmp_ret to 1, so we can lsl to select the segment

      "in %[portb], 0x18\n\t"  //Read portb and set clock_pin to 0
      //"cbr %[portb], %[clock_pin]\n\t"
      "cbr %[portb], 0x8\n\t"
      
      "SEGMENT_LOOP_%=:"

      "mov %[segment], %[segments]\n\t"
      "and %[segment], __tmp_reg__\n\t"//Isolate the single segment we want to light up
      //LSByte
      //Bit 0
      "bst %[segment], 0\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"

      //Bit 1
      "bst %[segment], 1\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 2
      "bst %[segment], 2\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"

      //Bit 3
      "bst %[segment], 3\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"

      //Bit 4
      "bst %[segment], 4\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 5
      "bst %[segment], 5\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 6
      "bst %[segment], 6\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 7
      "bst %[segment], 7\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"

      //MSByte
      //Bit 0
      "bst %[disp], 0\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
            
      //Bit 1
      "bst %[disp], 1\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 2
      "bst %[disp], 2\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 3
      "bst %[disp], 3\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 4
      "bst %[disp], 4\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 5
      "bst %[disp], 5\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 6
      "bst %[disp], 6\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      //Bit 7
      "bst %[disp], 7\n\t" 
      "bld %[portb], %[data_pin]\n\t"
      "out 0x18, %[portb]\n\t" 
      "sbi 0x18, %[clock_pin]\n\t"
      
      "sbi 0x18, %[latch_pin]\n\t" //latchHIGH

      //Loop to keep higher brightness and to smooth out difference in last segment
      //us = 0xc8 = 200
      //each loop takes 4 cycles, @16Mhz 4 cycles ~= 1/4us so execute 4*us
      //
      "ldi %[us], 0xc8\n\t"
      "busy1_%=: subi %[us],1\n\t" // 1 cycle
      "nop\n\t" //1 cycle
      "brne busy1_%=\n\t" // 2 cycles
      
      "ldi %[us], 0xc8\n\t"
      "busy2_%=: subi %[us],1\n\t" // 1 cycle
      "nop\n\t" //1 cycle
      "brne busy2_%=\n\t" // 2 cycles
      
      "ldi %[us], 0xc8\n\t"
      "busy3_%=: subi %[us],1\n\t" // 1 cycle
      "nop\n\t" //1 cycle
      "brne busy3_%=\n\t" // 2 cycles
      
      "ldi %[us], 0xc8\n\t"
      "busy4_%=: subi %[us],1\n\t" // 1 cycle
      "nop\n\t" //1 cycle
      "brne busy4_%=\n\t" // 2 cycles
      


      "lsl __tmp_reg__\n\t" // << until all 0s (sets carry to bit 7)
      "brcs END_%=\n\t" //Branch if carry
      "rjmp SEGMENT_LOOP_%=\n\t"
      "END_%=:nop\n\t"
    :
    [us] "=&a" (us),
    [portb] "=&a" (portb), 
    [segment] "=&a" (segment)
    :
    [segments] "a" (remapped),
    [disp] "a" ((0x20 >> i)),
    [data_pin] "M" (DATA_PIN),
    [clock_pin] "M" (CLOCK_PIN),
    [latch_pin] "M" (LATCH_PIN)
    );

  }
  
  //Clear all displays
  register uint8_t clock_hi;
  register uint8_t clock_lo;
  asm volatile(
    //This shifts one 0 bit
    "cbi 0x18, %[latch_pin]\n\t" //latchLOW
    "cbi 0x18, %[clock_pin]\n\t"
    "cbi 0x18, %[data_pin]\n\t"
    "sbi 0x18, %[clock_pin]\n\t"
    "nop\n\t"
    "in %[clock_hi], 0x18\n\t"
    "mov %[clock_lo], %[clock_hi]\n\t"
    "bst __zero_reg__, 0\n\t"
    "bld %[clock_lo], %[clock_pin]\n\t"

    //LSByte
    //Bit 1
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 2
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 3
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 4
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 5
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 6
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 7
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    
    //MSByte
    //Bit 8
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 9
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 10
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 11
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 12
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 13
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 14
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"
    //Bit 15
    "out 0x18, %[clock_lo]\n\t"
    "out 0x18, %[clock_hi]\n\t"

    "sbi 0x18, %[latch_pin]\n\t" //latchHIGH
  :
  [clock_hi] "=&a" (clock_hi),
  [clock_lo] "=&a" (clock_lo)
  :
  [data_pin] "M" (DATA_PIN),
  [clock_pin] "M" (CLOCK_PIN),
  [latch_pin] "M" (LATCH_PIN)
  );
  TinyWireS_stop_check();
}
