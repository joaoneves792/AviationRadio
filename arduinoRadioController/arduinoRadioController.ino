#include <Arduino.h>
#include <Wire.h>
#include "SI4844_NANO_EVERY.h"
//#include "./Encoder.h"
#include <avr/io.h>

#define latchPin 8 //Pin connected to ST_CP of 74HC595
#define clockPin 12 //Pin connected to SH_CP of 74HC595
#define dataPin 11 //Pin connected to DS of 74HC595
#define buttonPin A3 //Pin connected to active/standby button

#define ATDD_RESET_PIN 3
#define ATDD_IRQ_PIN 2

#define SLAVE_ACTIVE_ADDRESS 0x12
#define SLAVE_STNDBY_ADDRESS 0x13
#define ATDD_ADDRESS 0x11

#define LETTER_H 0b01101110
#define LETTER_E 0b10011110
#define LETTER_L 0b00011100
#define LETTER_R 0b00001010
#define LETTER_A 0b11101110



const unsigned char digitSegments[16]  = {
                 0b11111100,//0
                 0b01100000,//1
                 0b11011010,//2
                 0b11110010,//3
                 0b01100110,//4
                 0b10110110,//5
                 0b00111110,//6
                 0b11100000,//7
                 0b11111110,//8
                 0b11100110,//9
                 0b11101110,//A
                 0b00111110,//B
                 0b10011100,//C 
                 0b01111010,//D 
                 0b10011110,//E 
                 0b10001110,//F
                };


/*-----------------------*/
//Global variables (shared memory between all threads)
uint32_t shm_activeFreq = 1;
uint32_t shm_standbyFreq = 0;
volatile uint8_t shm_ATDDIRQ = 0;
/*-----------------------*/

/*------------------------*/
//Global *peripherals* 
//Encoder outerEncoder(2, 3);
//Encoder innerEncoder(4, 5);
SI4844 si4844; 
/*------------------------*/

/*-----------------------*/
//Thread handling
struct thread;
struct thread {
  void (*function)();
  short target;
  unsigned long lastExecuted;
  struct thread* next;
};

struct thread* threads;

struct thread* createNewThread(void (*function)(), short target){
  struct thread* newThread = (thread*)malloc(sizeof(thread));

  newThread->function = function;
  newThread->target = target;
  newThread->lastExecuted = 0;
  newThread->next =NULL;

  return newThread;  
}

void insertThread(struct thread* t){
  t->next = threads;
  threads = t;
}
/*-----------------------*/



void updateDisplays(){
  static long lastActiveFreq = 0;
  static long lastStandbyFreq = 0;

  if(lastActiveFreq != shm_activeFreq){
    Wire.beginTransmission(SLAVE_STNDBY_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[5]);
    Wire.write(1);
    Wire.write(digitSegments[4]);
    Wire.write(2);
    Wire.write(digitSegments[3]);
    Wire.write(3);
    Wire.write(digitSegments[2]);
    Wire.write(4);
    Wire.write(digitSegments[1]);  
    Wire.write(5);
    Wire.write(digitSegments[0]);  
    Wire.endTransmission();/*
    Wire.beginTransmission(SLAVE_STNDBY_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[(shm_standbyFreq / 100000) % 10]);
    Wire.write(1);
    Wire.write(digitSegments[(shm_standbyFreq / 10000) % 10]);
    Wire.write(2);
    Wire.write(digitSegments[(shm_standbyFreq / 1000) % 10] | 0x1);
    Wire.write(3);
    Wire.write(digitSegments[(shm_standbyFreq / 100) % 10]);
    Wire.write(4);
    Wire.write(digitSegments[(shm_standbyFreq / 10) % 10]);
    Wire.write(5);
    Wire.write(digitSegments[shm_standbyFreq % 10]);
    Wire.endTransmission();*/
    //lastActiveFreq = shm_activeFreq;
  }
     
}
/*
void smoothPotToDigit(){
#define potToDigit() ((short)(analogRead(signalPin)))
 static short average = potToDigit();
 
 average -= average/10;
 average += potToDigit()/10;

 shm_activeFreq = average;
}
*/
void readEncoder(){
  /*shm_activeFreq = outerEncoder.read();
  shm_standbyFreq = innerEncoder.read();*/
}

void buttonDebounce(){

  static bool M80 = false;
#define DEBOUNCE_DELAY 50
  static char lastFlickerableState = HIGH;
  static char lastStableState = HIGH;
  static unsigned long lastDebounceTime = 0;
  char currentState = digitalRead(buttonPin);

  if(currentState != lastFlickerableState){
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }

  if((millis()-lastDebounceTime) > DEBOUNCE_DELAY){
    if(lastStableState == LOW && currentState == HIGH){
      if(M80){
        si4844.setCustomBand(3, 10540-10, 10540, 10);
        M80 = false;
      }else{
        si4844.setCustomBand(3, 10430-10, 10430, 10);
        M80 = true;
      } 
      short tmpFreq = shm_activeFreq;
      shm_activeFreq = shm_standbyFreq;
      shm_standbyFreq = tmpFreq;
      lastStableState = HIGH;
    }else if (lastStableState == HIGH && currentState == LOW){
      lastStableState = LOW;
    }
  }
}

void checkSerial(){
  if(Serial.available()){
    char c = Serial.read();
    if(c == '+'){
      si4844.volumeUp();
      si4844.setStatusInterruptFromDevice(true);
    }else if(c == '-'){
      si4844.volumeDown();
      si4844.setStatusInterruptFromDevice(true);
    }
  }
}

void ATDD_RESET(){
  digitalWrite(ATDD_RESET_PIN, LOW);
  delay(200);
  digitalWrite(ATDD_RESET_PIN, HIGH);
}

void ATDD_POWERUP(){
#define ATDD_POWER_UP_CMD 0xE1
    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(ATDD_POWER_UP_CMD);
    Wire.write(0x0);
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(ATDD_ADDRESS, 1);
    uint8_t status = Wire.read();
    if(0x80 == status){
      digitalWrite(LED_BUILTIN, HIGH);
    }

}
/*
void ATDD_GET_STATUS(){
#define ATDD_GET_STATUS_CMD 0xE0
#define CTS_BIT_MASK (1 << 7) 
#define HOSTRST_BIT_MASK (1 << 6)
#define HOSTPWRUP_BIT_MASK (1 << 5)
#define INFORDY_BIT_MASK (1 << 4)
#define STATION_BIT_MASK (1 << 3)

    digitalWrite(LED_BUILTIN, HIGH);

    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(0xE0);
    Wire.endTransmission();
    delay(2);
    Wire.requestFrom(ATDD_ADDRESS, 4);
    uint8_t status = Wire.read();
    uint8_t resp1 = Wire.read();
    uint8_t resp2 = Wire.read();
    uint8_t resp3 = Wire.read();

    if(status & HOSTRST_BIT_MASK){
      Serial.println("Requested reset");
      ATDD_RESET();
    }

    if(status & HOSTPWRUP_BIT_MASK){
      Serial.println("Requested powerup");
      ATDD_POWERUP();
    }

    if(status & INFORDY_BIT_MASK){
      Serial.println("infoReady");
      shm_standbyFreq = ((uint16_t)resp2 << 8) | resp3;
    }
    Serial.println(status, HEX);
    //shm_standbyFreq = status;
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    
}
*/
void ATDDIHR(){
  shm_ATDDIRQ=1;
  //Serial.println("IRQ");
} 

void checkATTD(){
  //si4844.getStatus();
  if (si4844.hasStatusChanged())
  {
    Serial.print("Band Index: ");
    Serial.print(si4844.getStatusBandIndex());
    Serial.print(" - ");
    Serial.print(si4844.getBandMode());
    Serial.print(" - Frequency: ");    
    Serial.print(si4844.getFrequency(),0);
    //shm_standbyFreq = (uint32_t)si4844.getFrequency();
    Serial.print(" KHz");
    Serial.print(" - Volume: ");
    Serial.print(si4844.getVolume());
    Serial.print(" - Stereo ");
    Serial.println(si4844.getStereoIndicator());
 }
/*
  if(shm_ATDDIRQ){
    delay(5);
    ATDD_GET_STATUS();
    shm_ATDDIRQ = 0;
  }   
  */
}

void setup() {
  //Clock output
  //Set clock output on PA7
  //Unfortunatelly the Nano Every does not expose that pin, so I have shorted it to the adjacent PB0 (and PB1 accidentally as well)
  //This means the 16Mhz clock is exposed on the digital 9 and 10 of the arduino 
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKOUT_bm;


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  //pinMode(signalPin, INPUT);
  //ATDD
  //pinMode(ATDD_RESET_PIN, OUTPUT);
  //digitalWrite(ATDD_RESET_PIN, LOW);


  Wire.begin();
  struct thread* firstThread;

  //Create a cyclic list
  threads = NULL;
  
  insertThread(createNewThread(updateDisplays, 100));
  firstThread = threads;
  //insertThread(createNewThread(smoothPotToDigit, 5));
  //insertThread(createNewThread(readEncoder, 1));
  insertThread(createNewThread(checkATTD, 50));
  insertThread(createNewThread(buttonDebounce, 50));
  insertThread(createNewThread(checkSerial, 10));

  firstThread->next = threads;

  //outerEncoder.write(100);
  //innerEncoder.write(500);

  Serial.begin(9600);
  //Init ATDD IC 
  #define DEFAULT_BAND 3
  si4844.setup(ATDD_RESET_PIN, ATDD_IRQ_PIN, DEFAULT_BAND);
  si4844.setStatusInterruptFromDevice(true);
  //si4844.setVolume(63);
  si4844.setCustomBand(DEFAULT_BAND, 10540-10, 10540, 10);
  si4844.setBassTreble(1);
  digitalWrite(LED_BUILTIN, HIGH);
  //si4844.setBand(0);
  //shm_standbyFreq = (uint32_t)si4844.getFrequency();
  //attachInterrupt(digitalPinToInterrupt(ATDD_IRQ_PIN), ATDDIHR, RISING);
  //Wire.setClock(10000); 
  //For Nano Every
  //uint32_t frequency = 10000;
  // Formula is: BAUD = ((F_CLKPER/frequency) - F_CLKPER*T_RISE - 10)/2;
  // Where T_RISE varies depending on operating frequency...
  // From 1617 DS: 1000ns @ 100kHz / 300ns @ 400kHz / 120ns @ 1MHz
  //int16_t t_rise = 10000;
  //uint32_t baud = ((F_CPU_CORRECTED/frequency) - (((F_CPU_CORRECTED*t_rise)/1000)/1000)/1000 - 10)/2;
  //TWI0.MBAUD = (uint8_t)baud; 
  //ATDD_RESET();
}

void loop() {
  unsigned long time;
  struct thread* thread = threads;

  while(1){
    time = millis();
    if(time - thread->lastExecuted > thread->target){
      thread->function();
      thread->lastExecuted = time;
    }
    thread = thread->next;
  }
}
