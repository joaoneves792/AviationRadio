#define latchPin 8 //Pin connected to ST_CP of 74HC595
#define clockPin 12 //Pin connected to SH_CP of 74HC595
#define dataPin 11 //Pin connected to DS of 74HC595
#define signalPin A3 //Pin connected to potenciometer
#define buttonPin A4 //Pin connected to active/standby button

struct thread;

struct thread {
  void (*function)();
  short target;
  unsigned long lastExecuted;
  struct thread* next;
};


struct thread* threads;

short shm_activeFreq = 0;
short shm_standbyFreq = 0;


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


void refreshDisplays(){

  char dig1 = (shm_activeFreq / 100) % 10;
  char dig2 = (shm_activeFreq / 10) % 10;
  char dig3 = shm_activeFreq % 10;

  char dig4 = (shm_standbyFreq / 100) % 10;
  char dig5 = (shm_standbyFreq / 10) % 10;
  char dig6 = shm_standbyFreq % 10;
  
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 1 << 3 | 1 << 2 | (dig1 << 4));
  digitalWrite(latchPin, HIGH);
  
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 1 << 3 | 1 << 1 | (dig2 << 4));
  digitalWrite(latchPin, HIGH);

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 1 << 3 | 1 << 0 | (dig3 << 4));
  digitalWrite(latchPin, HIGH);

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 0 << 3 | 1 << 2 | (dig4 << 4));
  digitalWrite(latchPin, HIGH);
  
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 0 << 3 | 1 << 1 | (dig5 << 4));
  digitalWrite(latchPin, HIGH);
  
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 0 << 3 | 1 << 0 | (dig6 << 4));
  digitalWrite(latchPin, HIGH);


  //Blank all displays
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, 0 << 0 | (0 << 4));
  digitalWrite(latchPin, HIGH);
      
}

void smoothPotToDigit(){
#define potToDigit() ((short)(analogRead(signalPin)/850.0*999))
 static short average = potToDigit();
 
 average -= average/10;
 average += potToDigit()/10;

 shm_activeFreq = average;
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

  struct thread* firstThread;

  //Create a cyclic list
  threads = NULL;
  
  insertThread(createNewThread(refreshDisplays, 5));
  firstThread = threads;
  insertThread(createNewThread(smoothPotToDigit, 20));
  insertThread(createNewThread(buttonDebounce, 50));

  firstThread->next = threads;
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
