#include <Arduino.h>
#include <Wire.h>
//#include "SI4844_NANO_EVERY.h"
#include "./Encoder.h"
#include <avr/io.h>

#define buttonPin 2 //Pin connected to active/standby button
#define volumePotPin A6
#define FMselectPin 6
#define COMselectPin 7
#define NAVselectPin 8
#define ATDD_RESET_PIN 5
#define ATDD_IRQ_PIN 4

#define SLAVE_ACTIVE_ADDRESS 0x12
#define SLAVE_STNDBY_ADDRESS 0x13
#define ATDD_ADDRESS 0x11
#define EPROM_ADDRESS 0x50

#define ACTIVE_MEMSLOT 0
#define STANDBY_MEMSLOT 1

#define FM_MODE 0
#define AM_MODE 1
#define SW_MODE 2
#define COM_MODE 3
#define NAV_MODE 4


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

typedef struct{
  uint16_t MHz;
  uint16_t kHz;
} freq;
#define SET_FREQ(FREQ, STRUCT) {STRUCT.MHz = FREQ/1000; STRUCT.kHz = FREQ%1000;}
#define GET_FREQ(STRUCT) ((uint32_t)(STRUCT.MHz)*1000+STRUCT.kHz)



/*-----------------------*/
//Global variables (shared memory between all threads)
freq shm_activeFreq;
freq shm_standbyFreq;
uint8_t shm_bandmode = 0;
volatile uint8_t shm_ATDDIRQ = 0;
uint8_t shm_ATDDOperational = 0;
/*-----------------------*/

/*------------------------*/
Encoder outerEncoder(15, 14, 2, 1);
Encoder innerEncoder(16, 17, 2, 5);
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
//EEPROM commands

void EEPROMwrite(uint32_t freq, uint8_t mode, uint8_t active){
  uint8_t address = mode*8+active*4;
  do{
    Wire.beginTransmission(EPROM_ADDRESS);
    Wire.write(address);
    Wire.write((freq >> 24) & 0xff);
    Wire.write((freq >> 16) & 0xff);
    Wire.write((freq >> 8) & 0xff);
    Wire.write(freq & 0xff);
  }while(Wire.endTransmission());
}

uint32_t EEPROMread(uint8_t mode, uint8_t active){
  uint8_t address = mode*8+active*4;
  do{
    Wire.beginTransmission(EPROM_ADDRESS);
    Wire.write(address);
  }while(Wire.endTransmission(false));
  Wire.requestFrom(EPROM_ADDRESS, 4);
  while(Wire.available() < 4){delay(5);}
  uint32_t f =   ((uint32_t)Wire.read() << 24) 
          |((uint32_t)Wire.read() << 16) 
          |((uint32_t)Wire.read() << 8) 
          | Wire.read();
  return f;
}

/*-----------------------*/


/*-----------------------*/
//ATDD commands

uint16_t ATDD_FREQ_TO_INT_FM(uint16_t ATDD_freq){
  return ((ATDD_freq & 0x0f)+((ATDD_freq & 0xf0)>>4)*10+((ATDD_freq & 0xf00)>>8)*100+((ATDD_freq & 0xf000)>>12)*1000)*10;
}
uint16_t ATDD_FREQ_TO_INT_AM(uint16_t ATDD_freq){
  return ((ATDD_freq & 0x0f)+((ATDD_freq & 0xf0)>>4)*10+((ATDD_freq & 0xf00)>>8)*100+((ATDD_freq & 0xf000)>>12)*1000);
}


void ATDD_SET_PROPERTY(uint16_t propertyNumber, uint16_t parameter)
{
  if(!shm_ATDDOperational)
    return;
#define SET_PROPERTY 0x12
    Wire.beginTransmission(ATDD_ADDRESS);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);
    Wire.write(propertyNumber >> 8); // Send property - High byte - most significant first
    Wire.write(propertyNumber & 0xff);  // Send property - Low byte - less significant after
    Wire.write(parameter >> 8);    // Send the argments. High Byte - Most significant first
    Wire.write(parameter & 0xff);     // Send the argments. Low Byte - Less significant after
    Wire.endTransmission();
    Serial.println("property set");
}


void ATDD_RESET(){
  shm_ATDDOperational = 0;
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ATDD_RESET_PIN, LOW);
  delay(200);
  digitalWrite(ATDD_RESET_PIN, HIGH);
}

void ATDD_POWERUP(){
#define ATDD_POWER_UP_CMD 0xE1
#define EU_FM_BAND 2
#define AM_BAND 20
#define BAND EU_FM_BAND
//#define BAND AM_BAND
#define CHSPC 1

    if(shm_bandmode != FM_MODE && shm_bandmode != AM_MODE && shm_bandmode != SW_MODE)
      return; 

    uint16_t freq = (GET_FREQ(shm_activeFreq)/((BAND==EU_FM_BAND)?10:1));

    uint8_t args[7];

    //args[0] = 0x80; //Enable crystal
    args[0] = 0x0; //Disable crystal
    args[0] |= BAND;
    if(BAND == EU_FM_BAND){
      args[1] = (freq-1) >> 8;
      args[2] = (freq-1) & 0xff;

      args[3] = (freq) >> 8;
      args[4] = (freq) & 0xff;
    }else if(BAND == AM_BAND){
      args[1] = (freq) >> 8;
      args[2] = (freq) & 0xff;

      args[3] = (freq+1) >> 8;
      args[4] = (freq+1) & 0xff;

    }

    args[5] = CHSPC;

    args[6] = 0x1 << 6;

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
   
    shm_ATDDOperational = 1;
    delayMicroseconds(2500);
    ATDD_GET_STATUS();
    delayMicroseconds(2500);
    Serial.println("Setting clock");
    ATDD_SET_PROPERTY(0x0201, 0x7c9c); //Set clock to 31900
    delayMicroseconds(2500);
    ATDD_GET_STATUS();
    delayMicroseconds(2500);
    ATDD_SET_PROPERTY(0x0202, 500); //Set divider to 500
    delayMicroseconds(2500);
    ATDD_GET_STATUS();
    delayMicroseconds(2500);
    ATDD_SET_PROPERTY(0x4002, 1);
    shm_ATDDOperational = 0;
    
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

    Serial.println(status, HEX);
    if(status & HOSTRST_BIT_MASK){
      Serial.println("ATDD: Requested reset");
      ATDD_RESET();
      return;
    }

    if(status & HOSTPWRUP_BIT_MASK){
      Serial.println("ATDD: Requested powerup");
      ATDD_POWERUP();
      return;
    }
    
    if(GET_BAND_MODE(resp1) != shm_bandmode){
      Serial.println("ATDD: Band mismatch, resetting");
      ATDD_RESET();
      return;
    }

    if(status & INFORDY_BIT_MASK){
      digitalWrite(LED_BUILTIN, HIGH);
      shm_ATDDOperational = 1;
      Serial.print("ATDD: Frequency: ");
      Serial.print(ATDD_FREQ_TO_INT_FM(((uint16_t)resp2 << 8) | resp3));
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
      Serial.println("");
    }
    
}

void ATDDIHR(){
  shm_ATDDIRQ=1;
  //Serial.println("IRQ");
} 

/*-----------------------------*/
void printChange(){
  Serial.print("AVRADIO ");
  Serial.print("A:");
  Serial.print(GET_FREQ(shm_activeFreq));
  Serial.print(" S:");
  Serial.print(GET_FREQ(shm_standbyFreq));
  Serial.print(" M:");
  Serial.print(shm_bandmode);
  Serial.println("");
}


void updateDisplays(){
  static uint32_t lastActiveFreq = 0;
  static uint32_t lastStandbyFreq = 0;

  if(lastActiveFreq != GET_FREQ(shm_activeFreq)){
    Wire.beginTransmission(SLAVE_ACTIVE_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq) / 100000) % 10]);
    Wire.write(1);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq) / 10000) % 10]);
    Wire.write(2);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq) / 1000) % 10] | 0x1);
    Wire.write(3);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq) / 100) % 10]);
    Wire.write(4);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq)/ 10) % 10]);
    Wire.write(5);
    Wire.write(digitSegments[(GET_FREQ(shm_activeFreq)/ 1) % 10]);
    Wire.endTransmission();
    lastActiveFreq = GET_FREQ(shm_activeFreq);
  }
  if(lastStandbyFreq != GET_FREQ(shm_standbyFreq)){
    Wire.beginTransmission(SLAVE_STNDBY_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 100000) % 10]);
    Wire.write(1);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 10000) % 10]);
    Wire.write(2);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 1000) % 10] | 0x1);
    Wire.write(3);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 100) % 10]);
    Wire.write(4);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 10) % 10]);
    Wire.write(5);
    Wire.write(digitSegments[(GET_FREQ(shm_standbyFreq) / 1) % 10]);
    Wire.endTransmission();
    lastStandbyFreq = GET_FREQ(shm_standbyFreq);
  }
     
}

void checkVolumePot(){
#define potToDigit() ((uint16_t)(analogRead(volumePotPin)))
  static uint16_t prevVolume = 0;
  static uint16_t average = potToDigit();
 
  if(!shm_ATDDOperational)
    return;

  average -= average/10;
  average += potToDigit()/10;

  uint16_t volume = map(average, 1023, 0, 20, 63);
  //uint16_t volume = map(average, 0, 1023, 0, 30);
  if(volume != prevVolume){
 #define RX_VOLUME 0x4000
 //#define FM_DEEMPHASIS 0x1100
   ATDD_SET_PROPERTY(RX_VOLUME, volume);
   //ATDD_SET_PROPERTY(FM_DEEMPHASIS, 0x01);
   prevVolume = volume;
  }

}

void readEncoder(){
  uint32_t innerEncoderRead = innerEncoder.read()%1000;
  if(innerEncoderRead != shm_activeFreq.kHz){
    shm_activeFreq.kHz = innerEncoderRead;
    printChange();
    ATDD_POWERUP();
  }
  uint32_t outerEncoderRead = outerEncoder.read()%1000;
  if(outerEncoderRead != shm_activeFreq.MHz){
    shm_activeFreq.MHz = outerEncoderRead;
    printChange();
    ATDD_POWERUP();
  }
}

void buttonDebounce(){
#define DEBOUNCE_DELAY 50
  static uint8_t lastFlickerableState = HIGH;
  static uint8_t lastStableState = HIGH;
  static unsigned long lastDebounceTime = 0;
  uint8_t currentState = digitalRead(buttonPin);

  if(currentState != lastFlickerableState){
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }

  if((millis()-lastDebounceTime) > DEBOUNCE_DELAY){
    if(lastStableState == LOW && currentState == HIGH){
      uint16_t tmpFreqMHz = shm_activeFreq.MHz;
      uint16_t tmpFreqkHz = shm_activeFreq.kHz;
      SET_FREQ(GET_FREQ(shm_standbyFreq), shm_activeFreq);
      shm_standbyFreq.MHz = tmpFreqMHz;
      shm_standbyFreq.kHz = tmpFreqkHz;

      innerEncoder.write(shm_activeFreq.kHz);
      outerEncoder.write(shm_activeFreq.MHz);
      printChange();
      ATDD_POWERUP();


      lastStableState = HIGH;
    }else if (lastStableState == HIGH && currentState == LOW){
      lastStableState = LOW;
    }
  }
}


void checkATTD(){
  if(shm_ATDDIRQ && shm_bandmode == FM_MODE){
    ATDD_GET_STATUS();
    shm_ATDDIRQ = 0;
  }   
  
}

void checkEEPROM(){
  static uint32_t activeFreq = -1;
  if(activeFreq != GET_FREQ(shm_activeFreq)){
    activeFreq = GET_FREQ(shm_activeFreq);
    EEPROMwrite(GET_FREQ(shm_standbyFreq), shm_bandmode, STANDBY_MEMSLOT);
    EEPROMwrite(GET_FREQ(shm_activeFreq), shm_bandmode, ACTIVE_MEMSLOT);
  }

}

void checkMode(){
  static uint8_t currentBandMode = -1;
  if(!digitalRead(FMselectPin)){
    shm_bandmode = FM_MODE;
  }else if(!digitalRead(COMselectPin)){
    shm_bandmode = COM_MODE;
  }else if(!digitalRead(NAVselectPin)){
    shm_bandmode = NAV_MODE;  
  }
  if(currentBandMode != shm_bandmode){  
    currentBandMode = shm_bandmode;
    SET_FREQ(EEPROMread(shm_bandmode, STANDBY_MEMSLOT), shm_standbyFreq);
    SET_FREQ(EEPROMread(shm_bandmode, ACTIVE_MEMSLOT), shm_activeFreq);
    innerEncoder.write(shm_activeFreq.kHz);
    outerEncoder.write(shm_activeFreq.MHz);
    printChange();
    ATDD_RESET();
  }
}

void setup() {
  //Clock output
  //Set clock output on PA7
  //Unfortunatelly the Nano Every does not expose that pin, so I have shorted it to the adjacent PB0 (and PB1 accidentally as well)
  //This means the 16Mhz clock is exposed on the digital 9 and 10 of the arduino 
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKOUT_bm;
  //(No longer needed, got the crystal to work)
  /*pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);*/
  pinMode(FMselectPin, INPUT_PULLUP);
  pinMode(COMselectPin, INPUT_PULLUP);
  pinMode(NAVselectPin, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  shm_ATDDOperational = 0;
  pinMode(buttonPin, INPUT_PULLUP);
  
  pinMode(ATDD_RESET_PIN, OUTPUT);


  Wire.begin();
  struct thread* firstThread;

  //Create a cyclic list
  threads = NULL;
  
  insertThread(createNewThread(updateDisplays, 100));
  firstThread = threads;
  insertThread(createNewThread(readEncoder, 10));
  insertThread(createNewThread(checkATTD, 50));
  insertThread(createNewThread(buttonDebounce, 50));
  insertThread(createNewThread(checkVolumePot, 10));
  insertThread(createNewThread(checkMode, 20));
  insertThread(createNewThread(checkEEPROM, 500));

  firstThread->next = threads;

  Serial.begin(9600);

  //Init ATDD IC 

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

  checkMode();  
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
