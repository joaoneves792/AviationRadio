#include <Arduino.h>
#include <Wire.h>
//#include "SI4844_NANO_EVERY.h"
//#include "./Encoder.h"
#include <avr/io.h>

#define latchPin 8 //Pin connected to ST_CP of 74HC595
#define clockPin 12 //Pin connected to SH_CP of 74HC595
#define dataPin 11 //Pin connected to DS of 74HC595
#define buttonPin A3 //Pin connected to active/standby button
#define volumePotPin A2

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
//SI4844 si4844; 
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


/*-----------------------*/
//ATDD commands

uint16_t ATDD_FREQ_TO_INT(uint16_t ATDD_freq){
  return ((ATDD_freq & 0x0f)+((ATDD_freq & 0xf0)>>4)*10+((ATDD_freq & 0xf00)>>8)*100+((ATDD_freq & 0xf000)>>12)*1000)*10;
}



void ATDD_SET_PROPERTY(uint16_t propertyNumber, uint16_t parameter)
{
#define SET_PROPERTY 0x12
    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);
    Wire.write(propertyNumber >> 8); // Send property - High byte - most significant first
    Wire.write(propertyNumber & 0xff);  // Send property - Low byte - less significant after
    Wire.write(parameter >> 8);    // Send the argments. High Byte - Most significant first
    Wire.write(parameter & 0xff);     // Send the argments. Low Byte - Less significant after
    Wire.endTransmission();
}


void ATDD_RESET(){
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ATDD_RESET_PIN, LOW);
  delay(200);
  digitalWrite(ATDD_RESET_PIN, HIGH);
}

void ATDD_POWERUP(){
#define ATDD_POWER_UP_CMD 0xE1
#define EU_FM_BAND 2
#define FM_CHSPC 10
    

    uint8_t args[7];

    args[0] = 0x80;
    args[0] |= EU_FM_BAND;

    args[1] = (shm_activeFreq-FM_CHSPC) >> 8;
    args[2] = (shm_activeFreq-FM_CHSPC) & 0xff;

    args[3] = (shm_activeFreq) >> 8;
    args[4] = (shm_activeFreq) & 0xff;

    args[5] = FM_CHSPC;

    args[6] = 0x0;

    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(ATDD_POWER_UP_CMD);
    Wire.write(args[0]);
    Wire.write(args[1]);
    Wire.write(args[2]);
    Wire.write(args[3]);
    Wire.write(args[4]);
    Wire.write(args[5]);
    Wire.write(args[6]);
    Wire.endTransmission();
}

void ATDD_GET_STATUS(){
#define ATDD_GET_STATUS_CMD (0xE0)
#define CTS_BIT_MASK (1 << 7) 
#define HOSTRST_BIT_MASK (1 << 6)
#define HOSTPWRUP_BIT_MASK (1 << 5)
#define INFORDY_BIT_MASK (1 << 4)
#define STATION_BIT_MASK (1 << 3)
#define STEREO_BIT_MASK (1 << 2)
#define BCFG1_BIT_MASK (1 << 1)
#define BCFG0_BIT_MASK (1 << 0)
#define GET_BAND_MODE(resp) ((resp & 0xC0) >> 6)
#define GET_BANDIDX(resp) (resp & 0x3f)


    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(ATDD_GET_STATUS_CMD);
    Wire.endTransmission();
    delay(2);
    if(!Wire.requestFrom(ATDD_ADDRESS, 4))
      return;
    uint8_t status = Wire.read();
    uint8_t resp1 = Wire.read();
    uint8_t resp2 = Wire.read();
    uint8_t resp3 = Wire.read();

    if(status & HOSTRST_BIT_MASK){
      Serial.println("Requested reset");
      ATDD_RESET();
      return;
    }

    if(status & HOSTPWRUP_BIT_MASK){
      Serial.println("Requested powerup");
      ATDD_POWERUP();
      return;
    }

    if(status & INFORDY_BIT_MASK){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Frequency: ");
      //shm_activeFreq = ((uint16_t)resp2 << 8) | resp3;
      Serial.print(ATDD_FREQ_TO_INT(((uint16_t)resp2 << 8) | resp3));
      Serial.print("KHz");
      Serial.print(" Station ");
      (status & STATION_BIT_MASK)?Serial.print("valid"):Serial.print("invalid");
      Serial.print(" Stereo ");
      (status & STEREO_BIT_MASK)?Serial.print("on"):Serial.print("off");
      Serial.print(" Band: ");
      if(GET_BAND_MODE(resp1) == 0)
        Serial.print("FM");
      if(GET_BAND_MODE(resp1) == 1)
        Serial.print("AM");
      if(GET_BAND_MODE(resp1) == 2)
        Serial.print("SW");
    }
    Serial.println("");
    
}

void ATDDIHR(){
  shm_ATDDIRQ=1;
  //Serial.println("IRQ");
} 

/*-----------------------------*/

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

void checkVolumePot(){
#define potToDigit() ((short)(analogRead(volumePotPin)))
  static uint16_t prevVolume = 0;
  static short average = potToDigit();
 
  average -= average/10;
  average += potToDigit()/10;

  uint16_t volume = map(average, 0, 1023, 5, 63);
  if(volume != prevVolume){
 #define RX_VOLUME 0x4000
 //#define FM_DEEMPHASIS 0x1100
   ATDD_SET_PROPERTY(RX_VOLUME, volume);
   //ATDD_SET_PROPERTY(FM_DEEMPHASIS, 0x01);
   prevVolume = volume;
  }

}

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
        shm_activeFreq = 10540;
        ATDD_POWERUP();
        //si4844.setCustomBand(3, 10540-10, 10540, 10);
        M80 = false;
      }else{
        shm_activeFreq = 10430;
        ATDD_POWERUP();
        //si4844.setCustomBand(3, 10430-10, 10430, 10);
        M80 = true;
      } 
      /*short tmpFreq = shm_activeFreq;
      shm_activeFreq = shm_standbyFreq;
      shm_standbyFreq = tmpFreq;*/
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
      /*si4844.volumeUp();
      si4844.setStatusInterruptFromDevice(true);*/
    }else if(c == '-'){
      /*si4844.volumeDown();
      si4844.setStatusInterruptFromDevice(true);*/
    }
  }
}


void checkATTD(){
  //si4844.getStatus();
  /*
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
 }*/
  if(shm_ATDDIRQ){
    ATDD_GET_STATUS();
    shm_ATDDIRQ = 0;
  }   
  
}

void setup() {
  //Clock output
  //Set clock output on PA7
  //Unfortunatelly the Nano Every does not expose that pin, so I have shorted it to the adjacent PB0 (and PB1 accidentally as well)
  //This means the 16Mhz clock is exposed on the digital 9 and 10 of the arduino 
  //CPU_CCP = CCP_IOREG_gc;
  //CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKOUT_bm;
  //(No longer needed, got the crystal to work)


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  //pinMode(signalPin, INPUT);
  //ATDD
  pinMode(ATDD_RESET_PIN, OUTPUT);
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
  insertThread(createNewThread(checkVolumePot, 10));
  //insertThread(createNewThread(checkSerial, 10));

  firstThread->next = threads;

  //outerEncoder.write(100);
  //innerEncoder.write(500);

  Serial.begin(9600);

  //Init ATDD IC 

/*
  #define DEFAULT_BAND 3
  si4844.setup(ATDD_RESET_PIN, ATDD_IRQ_PIN, DEFAULT_BAND);
  si4844.setStatusInterruptFromDevice(true);
  //si4844.setVolume(63);
  si4844.setCustomBand(DEFAULT_BAND, 10540-10, 10540, 10);
  si4844.setBassTreble(1);
  digitalWrite(LED_BUILTIN, HIGH);
*/
  attachInterrupt(digitalPinToInterrupt(ATDD_IRQ_PIN), ATDDIHR, RISING);
  //Wire.setClock(10000); 
  //For Nano Every
  // Formula is: BAUD = ((F_CLKPER/frequency) - F_CLKPER*T_RISE - 10)/2;
  // Where T_RISE varies depending on operating frequency...
  // From 1617 DS: 1000ns @ 100kHz / 300ns @ 400kHz / 120ns @ 1MHz
  uint32_t frequency = 10000;
  int16_t t_rise = 10000;
  uint32_t baud = ((F_CPU_CORRECTED/frequency) - (((F_CPU_CORRECTED*t_rise)/1000)/1000)/1000 - 10)/2;
  TWI0.MBAUD = (uint8_t)baud;

  shm_activeFreq = 10540; 
  ATDD_RESET();
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
