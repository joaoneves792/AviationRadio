#include <Arduino.h>
#include "si5351.h"
#include <Wire.h>
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
#define LO_ADDRESS 0x60

#define ACTIVE_MEMSLOT 0
#define STANDBY_MEMSLOT 1

#define FM_MODE 0
#define AM_MODE 1
#define SW_MODE 2
#define COM_MODE 3
#define NAV_MODE 4
#define COM2_MODE 5

#define ENCODER_CONTROLLS_STANDBY
#undef ENCODER_CONTROLLS_STANDBY


// IF in kHz
#define IF 1205
#define LO_PPM 146391

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

struct setPropertyQueueElement;
struct setPropertyQueueElement{
  uint16_t property;
  uint16_t value;
  struct setPropertyQueueElement* next;
};


/*-----------------------*/
//Global variables (shared memory between all threads)
freq shm_activeFreq;
freq shm_standbyFreq;
struct setPropertyQueueElement* shm_propertiesQueue = NULL;
uint8_t shm_bandmode = 0;
volatile uint8_t shm_ATDDIRQ = 0;
uint8_t shm_ATDDOperational = 0;
uint8_t shm_ATDDReady = 0;
uint8_t shm_scanInProgress = 0;
Si5351 shm_si5351(LO_ADDRESS);
/*-----------------------*/

/*------------------------*/
Encoder* outerEncoder = NULL;
Encoder* innerEncoder = NULL;
/*------------------------*/

/*------------------------*/
/* property queue*/
void queueProperty(uint16_t property, uint16_t value) {
  struct setPropertyQueueElement* prev = shm_propertiesQueue;
  while(prev && prev->next){
    prev = prev->next;
  }
  struct setPropertyQueueElement* prop = (struct setPropertyQueueElement*) malloc(sizeof(struct setPropertyQueueElement));
  prop->property = property;
  prop->value = value;
  prop->next = NULL;
  if(NULL == prev){
    shm_propertiesQueue = prop;
  }else{
    prev->next = prop;
  }
 }
/*-----------------------*/

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

  shm_ATDDReady = 0;

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
  shm_ATDDOperational = 0;
  shm_ATDDReady = 0;
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ATDD_RESET_PIN, LOW);
  delay(200);
  digitalWrite(ATDD_RESET_PIN, HIGH);
}

void ATDD_POWERUP(){
#define ATDD_POWER_UP_CMD 0xE1
    uint8_t band;
    uint8_t chspc;

    if(shm_bandmode != FM_MODE && shm_bandmode != AM_MODE && shm_bandmode != SW_MODE)
      return; 

    if(shm_bandmode == FM_MODE){
      band = 2;
      chspc = 1;
    }
    else if (shm_bandmode == AM_MODE){
      band = 25; //SW bandmode
      chspc = 5;
    }

    shm_ATDDReady = 0;

    uint16_t freq = (GET_FREQ(shm_activeFreq)/((band==EU_FM_BAND)?10:1));

    uint8_t args[7];

    args[0] = 0x80; //Enable crystal
    //args[0] = 0x0; //Disable crystal
    args[0] |= band;
    if(shm_bandmode == FM_MODE){
      args[1] = (freq-chspc) >> 8;
      args[2] = (freq-chspc) & 0xff;

      args[3] = (freq) >> 8;
      args[4] = (freq) & 0xff;
    }else if(shm_bandmode == AM_MODE){
      args[1] = (IF-(50*chspc)) >> 8;
      args[2] = (IF-(50*chspc)) & 0xff;

      args[3] = (IF) >> 8;
      args[4] = (IF) & 0xff;

    }

    args[5] = chspc;

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
   
    //ATDD_SET_PROPERTY(0x0201, 0x7c9c); //Set clock to 31900
    //ATDD_SET_PROPERTY(0x0202, 500); //Set divider to 500
    
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
      Serial.println("ATDD: Requested reset");
      ATDD_RESET();
      return;
    }

    if(status & HOSTPWRUP_BIT_MASK){
      Serial.println("ATDD: Requested powerup");
      ATDD_POWERUP();
      return;
    }
    
    if(status & INFORDY_BIT_MASK){
      if(!shm_ATDDOperational){
#define RX_BASS_TREBLE 0x4002
#define AM_SOFT_MUTE_SLOPE 0x3301
#define AM_SOFT_MUTE_MAX_ATTENUATION 0x3302
#define AM_SOFT_MUTE_SNR_THRESHOLD 0x3303
#define AM_SOFT_MUTE_RATE 0x3300
        if(shm_bandmode == FM_MODE){
          queueProperty(RX_BASS_TREBLE, 1);
        }else{
          queueProperty(RX_BASS_TREBLE, 1);
          queueProperty(AM_SOFT_MUTE_SNR_THRESHOLD, 5);//40
          queueProperty(AM_SOFT_MUTE_SLOPE, 5);
          queueProperty(AM_SOFT_MUTE_MAX_ATTENUATION, 30); //63
          queueProperty(AM_SOFT_MUTE_RATE, 255);
        }

       //Mute: queueProperty(0x4001, 0x3);
      }
      shm_ATDDOperational = 1;
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("ATDD: Frequency: ");
      if(shm_bandmode == FM_MODE)
        Serial.print(ATDD_FREQ_TO_INT_FM(((uint16_t)resp2 << 8) | resp3));
      else
        Serial.print(ATDD_FREQ_TO_INT_AM((uint16_t)resp2 << 8 | resp3));
      
      Serial.print("KHz");
      Serial.print(" Station ");
      if(shm_scanInProgress && (status & STATION_BIT_MASK)){
        shm_scanInProgress = 0;
      }
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
    shm_ATDDReady = (status & CTS_BIT_MASK);
    
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

  uint16_t volume = map(average, 200, 1023, 20, 63);
  //uint16_t volume = map(average, 0, 1023, 0, 30);
  if(prevVolume != volume){
#define RX_VOLUME 0x4000
   ATDD_SET_PROPERTY(RX_VOLUME, volume);
   shm_ATDDReady = 0;
   prevVolume = volume;
  }

}

void readEncoder(){
  uint32_t innerEncoderRead = innerEncoder->read()%1000;
#ifdef ENCODER_CONTROLLS_STANDBY 
  if(innerEncoderRead != shm_standbyFreq.kHz){
    shm_standbyFreq.kHz = innerEncoderRead;
#else
  if(innerEncoderRead != shm_activeFreq.kHz){
    shm_activeFreq.kHz = innerEncoderRead;
#endif
    printChange();
    //We are only changing standby freq, so no need for powerup
#ifndef ENCODER_CONTROLLS_STANDBY
    ATDD_POWERUP();
#endif
  }
  uint32_t outerEncoderRead = outerEncoder->read()%1000;
#ifdef ENCODER_CONTROLLS_STANDBY
  if(outerEncoderRead != shm_standbyFreq.MHz){
    shm_standbyFreq.MHz = outerEncoderRead;
#else
  if(outerEncoderRead != shm_activeFreq.MHz){
    shm_activeFreq.MHz = outerEncoderRead;
#endif
    printChange();
#ifndef ENCODER_CONTROLLS_STANDBY
    if(shm_bandmode == FM_MODE)
      ATDD_POWERUP();
    else{
      uint64_t LO = GET_FREQ(shm_activeFreq)*SI5351_FREQ_MULT - IF;
      shm_si5351.set_freq(LO, SI5351_CLK0);
    }
#endif
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
#ifdef ENCODER_CONTROLLS_STANDBY
      innerEncoder->write(shm_standbyFreq.kHz);
      outerEncoder->write(shm_standbyFreq.MHz);
#else
      innerEncoder->write(shm_activeFreq.kHz);
      outerEncoder->write(shm_activeFreq.MHz);
#endif
      printChange();
      if(shm_bandmode == FM_MODE){
        ATDD_POWERUP();
      }else if(shm_bandmode == AM_MODE){
        uint64_t LO = GET_FREQ(shm_activeFreq)*SI5351_FREQ_MULT - IF;
        shm_si5351.set_freq(LO, SI5351_CLK0);
      }


      lastStableState = HIGH;
    }else if (lastStableState == HIGH && currentState == LOW){
      lastStableState = LOW;
    }
  }
}


void checkATTD(){
  if(shm_ATDDIRQ && (shm_bandmode == FM_MODE || shm_bandmode == AM_MODE)){
    ATDD_GET_STATUS();
    shm_ATDDIRQ = 0;
  }   
  
}

void checkEEPROM(){
  static uint32_t activeFreq = -1;
  static uint32_t standbyFreq = -1;
  if(
    (activeFreq != GET_FREQ(shm_activeFreq)) ||
    (standbyFreq != GET_FREQ(shm_standbyFreq))    
    ){
    activeFreq = GET_FREQ(shm_activeFreq);
    standbyFreq = GET_FREQ(shm_standbyFreq);
    EEPROMwrite(GET_FREQ(shm_standbyFreq), shm_bandmode, STANDBY_MEMSLOT);
    EEPROMwrite(GET_FREQ(shm_activeFreq), shm_bandmode, ACTIVE_MEMSLOT);
  }

}

void checkMode(){
  static uint8_t currentBandMode = -1;
  uint8_t encoderSteps = 1;
  if(shm_bandmode == AM_MODE){
    encoderSteps = 1;
  }

  if(!digitalRead(FMselectPin)){
    shm_bandmode = FM_MODE;
    encoderSteps = 100;
  }else if(!digitalRead(COMselectPin)){
    shm_bandmode = COM_MODE;
    encoderSteps = 5;
  }else if(!digitalRead(NAVselectPin)){
    shm_bandmode = NAV_MODE;  
    encoderSteps = 100;
  }else{
    shm_bandmode = AM_MODE;
    encoderSteps = 1;
  }
  if(currentBandMode != shm_bandmode){  
    currentBandMode = shm_bandmode;
    SET_FREQ(EEPROMread(shm_bandmode, STANDBY_MEMSLOT), shm_standbyFreq);
    SET_FREQ(EEPROMread(shm_bandmode, ACTIVE_MEMSLOT), shm_activeFreq);
    if(outerEncoder != NULL)
      delete outerEncoder;
    outerEncoder = new Encoder(14, 17, 2, 1);
    if(innerEncoder != NULL)
      delete innerEncoder;
    innerEncoder = new Encoder(16, 15, 2, encoderSteps);
#ifdef ENCODER_CONTROLLS_STANDBY
    innerEncoder->write(shm_standbyFreq.kHz);
    outerEncoder->write(shm_standbyFreq.MHz);
#else
    innerEncoder->write(shm_activeFreq.kHz);
    outerEncoder->write(shm_activeFreq.MHz);
#endif
    printChange();
    if(shm_bandmode == AM_MODE){
      uint64_t LO = GET_FREQ(shm_activeFreq)*SI5351_FREQ_MULT - IF;
      shm_si5351.set_freq(LO, SI5351_CLK0);
    }
    ATDD_RESET();
  }
}

void applyQueuedProperties(){
  if(!shm_ATDDOperational || !shm_ATDDReady || NULL == shm_propertiesQueue)
    return;

  struct setPropertyQueueElement* property = shm_propertiesQueue;
  shm_propertiesQueue = property->next;

  ATDD_SET_PROPERTY(property->property, property->value);
  shm_ATDDIRQ = 1;
  shm_ATDDReady = 0;

  free(property);
 
}

void handleSerial(){
  char query[3+6+1];
  int8_t i = 0;
  uint8_t mode;
  uint8_t active;
  if(Serial.available() > 0){
    while(Serial.available() && i < 3){
      char c = Serial.read();
      if(c != '\n' || c != '\r'){
        query[i] = c;
        i++;
      }
      if(query[0] == 'a'){
        shm_bandmode = AM_MODE;
        return;
      }
      if(query[0] == 'x'){
        shm_scanInProgress = 1;
        return;
      }
    }
    if(i==3){
        mode = (query[1] == 'c')?COM_MODE:-1;
        mode = (query[1] == 'n')?NAV_MODE:mode;
        mode = (query[1] == 'f')?FM_MODE:mode;
        mode = (query[1] == 'a')?AM_MODE:mode;
        if(mode == -1) return;

        active = (query[2] == 'a')?ACTIVE_MEMSLOT:-1;
        active = (query[2] == 's')?STANDBY_MEMSLOT:active;
        if(active == -1) return;
    }
    if(i==3 && query[0] == 'q'){
        query[3] = '\0';
        Serial.print(query);
        freq f;
        SET_FREQ(EEPROMread(mode, ACTIVE_MEMSLOT), f);
        Serial.println(GET_FREQ(f));
    }else if (i==3 && query[0] == 's'){
      while(Serial.available() && (i < 3+6)){
        char c = Serial.read();
        if(c != '\n' || c != '\r'){
          query[i] = c;
          i++;
        }
      }
      if(i==3+6){
        query[3+6]='\0';
        uint32_t serialFreq;
        serialFreq = atol(query+3);
        EEPROMwrite(serialFreq, mode, active);
        if(shm_bandmode == mode){
          if(active == ACTIVE_MEMSLOT){
            SET_FREQ(serialFreq, shm_activeFreq);
#ifndef ENCODER_CONTROLLS_STANDBY
            innerEncoder->write(serialFreq%1000);
            outerEncoder->write(serialFreq/1000);
#endif
            ATDD_POWERUP();
          }else if(active == STANDBY_MEMSLOT){
            SET_FREQ(serialFreq, shm_standbyFreq);
#ifdef ENCODER_CONTROLLS_STANDBY
            innerEncoder->write(serialFreq%1000);
            outerEncoder->write(serialFreq/1000);
#endif
          }
        }
      }

    }
  }
}

void handleScan(){
  if(!shm_scanInProgress || !shm_ATDDOperational)
    return;

  uint8_t chspc = (shm_bandmode == FM_MODE)?100:10;
  shm_activeFreq.kHz += chspc;

  if(shm_activeFreq.kHz >= 990){
    shm_activeFreq.kHz = 0;
  }

  if(shm_activeFreq.kHz >= 1000){
    shm_activeFreq.kHz = 0;
    //shm_activeFreq.MHz += 1;
  }

#ifndef ENCODER_CONTROLLS_STANDBY
  innerEncoder->write(shm_activeFreq.kHz);
  outerEncoder->write(shm_activeFreq.MHz);
#endif
  printChange();
  ATDD_POWERUP();
}

void setup() {
  //Clock output
  //Set clock output on PA7
  //Unfortunatelly the Nano Every does not expose that pin, so I have shorted it to the adjacent PB0 (and PB1 accidentally as well)
  //This means the 16Mhz clock is exposed on the digital 9 and 10 of the arduino 
  //CPU_CCP = CCP_IOREG_gc;
  //CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKOUT_bm;
  //(No longer needed, got the crystal to work)
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
  insertThread(createNewThread(checkMode, 100));
  insertThread(createNewThread(checkEEPROM, 500));
  insertThread(createNewThread(applyQueuedProperties, 10));
  insertThread(createNewThread(handleSerial, 10));
  insertThread(createNewThread(handleScan, 200));

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


  //Init LO IC
  while(!shm_si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, LO_PPM))
  {
    Serial.println("Clock generator not found on I2C bus!");
    delay(1000);
  }
  shm_si5351.update_status();
  
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
