#include <Wire.h>
#include "./Encoder.h"
Encoder outerEncoder(2, 3);
Encoder innerEncoder(4, 5);

#define latchPin 8 //Pin connected to ST_CP of 74HC595
#define clockPin 12 //Pin connected to SH_CP of 74HC595
#define dataPin 11 //Pin connected to DS of 74HC595
#define buttonPin A3 //Pin connected to active/standby button
#define signalPin A6

#define SLAVE_ACTIVE_ADDRESS 0x12
#define SLAVE_STNDBY_ADDRESS 0x13

#define LETTER_H 0b01101110
#define LETTER_E 0b10011110
#define LETTER_L 0b00011100
#define LETTER_R 0b00001010
#define LETTER_A 0b11101110



const unsigned char digitSegments[10]  = {
                 0b11111100,//0
                 0b01100000,//1
                 0b11011010,//2
                 0b11110010,//3
                 0b01100110,//4
                 0b10110110,//5
                 0b00111110,//6
                 0b11100000,//7
                 0b11111110,//8
                 0b11100110 //9
                };


/*-----------------------*/
//Global variables (shared memory between all threads)
long shm_activeFreq = 0;
short shm_standbyFreq = 0;
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



void updateDisplays(){
  static long lastActiveFreq = 0;
  static long lastStandbyFreq = 0;

  if(lastActiveFreq != shm_activeFreq){
    Wire.beginTransmission(SLAVE_ACTIVE_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[(int)((shm_activeFreq / 100) % 10)]);
    Wire.write(1);
    Wire.write(digitSegments[(int)((shm_activeFreq /10) % 10)]);
    Wire.write(2);
    Wire.write(digitSegments[(int)(shm_activeFreq % 10)]);  
    Wire.write(3);
    Wire.write(digitSegments[3]);
    Wire.write(4);
    Wire.write(digitSegments[4]);  
    Wire.write(5);
    Wire.write(digitSegments[8]);  
    Wire.endTransmission();
    Wire.beginTransmission(SLAVE_STNDBY_ADDRESS);
    Wire.write(0);
    Wire.write(digitSegments[0]);
    Wire.write(1);
    Wire.write(digitSegments[1]);
    Wire.write(2);
    Wire.write(digitSegments[2]);  
    Wire.write(3);
    Wire.write(digitSegments[3]);
    Wire.write(4);
    Wire.write(digitSegments[4]);
    Wire.write(5);
    Wire.write(digitSegments[5]);
    Wire.endTransmission();
    lastActiveFreq = shm_activeFreq;
  }
     
}

void smoothPotToDigit(){
#define potToDigit() ((short)(analogRead(signalPin)))
 static short average = potToDigit();
 
 average -= average/10;
 average += potToDigit()/10;

 shm_activeFreq = average;
}

void readEncoder(){
  shm_activeFreq = outerEncoder.read();
  shm_standbyFreq = innerEncoder.read();
}

void buttonDebounce(){
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
      short tmpFreq = shm_activeFreq;
      shm_activeFreq = shm_standbyFreq;
      shm_standbyFreq = tmpFreq;
      lastStableState = HIGH;
    }else if (lastStableState == HIGH && currentState == LOW){
      lastStableState = LOW;
    }
  }
}

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(signalPin, INPUT);

  Wire.begin();

  struct thread* firstThread;

  //Create a cyclic list
  threads = NULL;
  
  insertThread(createNewThread(updateDisplays, 100));
  firstThread = threads;
  insertThread(createNewThread(smoothPotToDigit, 5));
  //insertThread(createNewThread(readEncoder, 1));
  insertThread(createNewThread(buttonDebounce, 50));

  firstThread->next = threads;

  outerEncoder.write(100);
  innerEncoder.write(500);
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
