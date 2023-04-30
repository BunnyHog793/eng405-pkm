// PKM-Motor-Controller for 6 Axis Paralel Kinomatics Machine
// Christpher Zuehlke
// ENG405 Senior Project

// Commands:
//    H - Go home
//    G x y z a b c - move PKM
//    A 
//    P - printout the current location of the PKM

// original Stepper Moter move function code from https://www.youtube.com/watch?v=fHAO7SW-SZI || http://www.iforce2d.net/sketches/


// Pin assignments will need to change when the shield is used
#define X_STEP_PIN         5
#define X_DIR_PIN          6 // PORTB pin 3
#define X_ENABLE_PIN       7
#define LIMIT_SWITCH_M1    63

#define Y_STEP_PIN         8
#define Y_DIR_PIN          9
#define Y_ENABLE_PIN       10
#define LIMIT_SWITCH_M2    64

#define Z_STEP_PIN         11
#define Z_DIR_PIN          12
#define Z_ENABLE_PIN       13
#define LIMIT_SWITCH_M3    65

#define A_STEP_PIN         54
#define A_DIR_PIN          55
#define A_ENABLE_PIN       56
#define LIMIT_SWITCH_M4    66

#define B_STEP_PIN         57
#define B_DIR_PIN          58
#define B_ENABLE_PIN       59
#define LIMIT_SWITCH_M5    67

#define C_STEP_PIN         60
#define C_DIR_PIN          61
#define C_ENABLE_PIN       62
#define LIMIT_SWITCH_M6    68

#define EstopPin           69

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

#define stepLimit       10000

// Hold the current locaiton of the steppers
int x_loc = 0;
int y_loc = 0;
int z_loc = 0;
int a_loc = 0;
int b_loc = 0;
int c_loc = 0;

//steps per mm
float XstepRate = 50;
float YstepRate = 50;
float ZstepRate = 50;
float AstepRate = 50;
float BstepRate = 50;
float CstepRate = 50;

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned long minStepInterval; // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  volatile unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
  volatile unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;               // used to slow down this motor to make coordinated movement with other motors

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xStep() {
  // X_STEP_HIGH
  // X_STEP_LOW
  digitalWrite(X_STEP_PIN, HIGH);
  digitalWrite(X_STEP_PIN, LOW);
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  // Y_STEP_HIGH
  // Y_STEP_LOW
  digitalWrite(Y_STEP_PIN, HIGH);
  digitalWrite(Y_STEP_PIN, LOW);
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

void zStep() {
  // Z_STEP_HIGH
  // Z_STEP_LOW
  digitalWrite(Z_STEP_PIN, HIGH);
  digitalWrite(Z_STEP_PIN, LOW);
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void aStep() {
  // A_STEP_HIGH
  // A_STEP_LOW
  digitalWrite(A_STEP_PIN, HIGH);
  digitalWrite(A_STEP_PIN, LOW);
}
void aDir(int dir) {
  digitalWrite(A_DIR_PIN, dir);
}

void bStep() {
  // B_STEP_HIGH
  // B_STEP_LOW
  digitalWrite(B_STEP_PIN, HIGH);
  digitalWrite(B_STEP_PIN, LOW);
}
void bDir(int dir) {
  digitalWrite(B_DIR_PIN, dir);
}

void cStep() {
  // C_STEP_HIGH
  // C_STEP_LOW
  digitalWrite(C_STEP_PIN, HIGH);
  digitalWrite(C_STEP_PIN, LOW);
}
void cDir(int dir) {
  digitalWrite(C_DIR_PIN, dir);
}

void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 6

volatile stepperInfo steppers[NUM_STEPPERS];

void setup() {
  Serial.begin(9600);
  while(!Serial.available());

  int accel = 250; // low is faster
  

  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  pinMode(A_STEP_PIN,   OUTPUT);
  pinMode(A_DIR_PIN,    OUTPUT);
  pinMode(A_ENABLE_PIN, OUTPUT);

  pinMode(B_STEP_PIN,   OUTPUT);
  pinMode(B_DIR_PIN,    OUTPUT);
  pinMode(B_ENABLE_PIN, OUTPUT);

  pinMode(C_STEP_PIN,   OUTPUT);
  pinMode(C_DIR_PIN,    OUTPUT);
  pinMode(C_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(A_ENABLE_PIN, LOW);
  digitalWrite(B_ENABLE_PIN, LOW);
  digitalWrite(C_ENABLE_PIN, LOW);

  pinMode(LIMIT_SWITCH_M1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_M2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_M3, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_M4, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_M5, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_M6, INPUT_PULLUP);
  pinMode(EstopPin, INPUT_PULLUP);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = accel;
  steppers[0].minStepInterval = 50;

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = accel;
  steppers[1].minStepInterval = 50;

  steppers[2].dirFunc = zDir;
  steppers[2].stepFunc = zStep;
  steppers[2].acceleration = accel;
  steppers[2].minStepInterval = 50;

  steppers[3].dirFunc = aDir;
  steppers[3].stepFunc = aStep;
  steppers[3].acceleration = accel;
  steppers[3].minStepInterval = 50;

  steppers[4].dirFunc = bDir;
  steppers[4].stepFunc = bStep;
  steppers[4].acceleration = accel;
  steppers[4].minStepInterval = 50;

  steppers[5].dirFunc = cDir;
  steppers[5].stepFunc = cStep;
  steppers[5].acceleration = accel;
  steppers[5].minStepInterval = 50;
  

  Serial.println("PKM-Started: Awaiting command");
  Serial.println("Enter: G XX YY ZZ AA BB CC (mm->0.02) to command a movement");
  Serial.println("P - print current position in steps.");
  Serial.println("H - Move to home position");
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;

  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;

  si.estStepsToSpeed = n;
}

volatile byte remainingSteppersFlag = 0;

float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
  float d = s.c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  
  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = abs(steps);

  if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
    // there will be a period of time at full speed
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  }
  else {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

volatile byte nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d * s.speedScale; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}


void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingSteppersFlag );
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}

void moveStepper(int motor_step_pin, int steps, int interval) {
    
  for (int i = 0; i < steps; i++) {
    digitalWrite(motor_step_pin, HIGH);
    delay(interval);
    digitalWrite(motor_step_pin, LOW);
    
    delay(interval);
  }
}

void setAccel(int acc) {
  for (int i = 0; i < 6; ++i) {
    steppers[i].acceleration = acc;
  }
}

void goHome() {

  movePKM(-abs(x_loc), -abs(y_loc), -abs(z_loc), -abs(a_loc), -abs(b_loc), -abs(c_loc));


  setAccel(5000);
  
  
  while (digitalRead(LIMIT_SWITCH_M1) == HIGH || digitalRead(LIMIT_SWITCH_M2) == HIGH || 
         digitalRead(LIMIT_SWITCH_M3) == HIGH || digitalRead(LIMIT_SWITCH_M4) == HIGH || 
         digitalRead(LIMIT_SWITCH_M5) == HIGH || digitalRead(LIMIT_SWITCH_M6) == HIGH) {
    int ls1 = digitalRead(LIMIT_SWITCH_M1);
    int ls2 = digitalRead(LIMIT_SWITCH_M2);
    int ls3 = digitalRead(LIMIT_SWITCH_M3);
    int ls4 = digitalRead(LIMIT_SWITCH_M4);
    int ls5 = digitalRead(LIMIT_SWITCH_M5);
    int ls6 = digitalRead(LIMIT_SWITCH_M6);

    if (ls1 == HIGH) {
      prepareMovement( 0, 10);
    } 
    if (ls2 == HIGH) {
      prepareMovement( 1, 10);
    }
    if (ls3 == HIGH) {
      prepareMovement( 2, 10);
    }
    if (ls4 == HIGH) {
      // Serial.println("ls4");
      prepareMovement( 3, 10);
    }
    if (ls5 == HIGH) {
      // Serial.println("ls5");
      prepareMovement( 4, 10);
    }
    if (ls6 == HIGH) {
      // Serial.println("ls6");
      prepareMovement( 5, 10);
    }
    runAndWait();
  }
  

  
  x_loc = 0;
  y_loc = 0;
  z_loc = 0;
  a_loc = 0;
  b_loc = 0;
  c_loc = 0;
  
  setAccel(250);
  
  Serial.println("Homing Complete");
}


//Move the PKM by distance
void moveDPKM(double x, double y, double z, double a, double b, double c, double v) {
  // steps = dist * steprate
  int minStep = 50;
  int xSteps = round(x * XstepRate);
  int ySteps = round(y * YstepRate);
  int zSteps = round(z * ZstepRate);
  int aSteps = round(a * AstepRate);
  int bSteps = round(b * BstepRate);
  int cSteps = round(c * CstepRate);

  if (v != 0) {
    minStep = calculateMinStep(v);//find min number////
  } 
    

  for (int i = 0; i < 6; ++i) {
    steppers[i].minStepInterval = minStep;
  }
  // if ((xSteps + x_loc) > stepLimit) {
  //   xSteps = stepLimit - x_loc;
  // }
  
  movePKM(xSteps, ySteps, zSteps, a * AstepRate, b * BstepRate, c * CstepRate);
}


int calculateMinStep(double speed) {
  if (speed < 1) {
    speed = 1;
  }
  // calculate the min step int off of the max speed
  // 62500
  // int minStep = 31250 / ( XstepRate * speed); //This eqaution needs to be tuned!
  int minStep = (14.1*speed*speed - 2946*speed + 132000) / 1000;
  // int minStep = -1621*speed+101860
  if (minStep < 1) {
    minStep = 2;
  }
  // Serial.println(minStep);
  return minStep;
}


void movePKM(int x, int y, int z, int a, int b, int c) {
  /*
  move the steppers x, y, z, a, b, c setps
  
  */
  x_loc += x;
  y_loc += y;
  z_loc += z;
  a_loc += a;
  b_loc += b;
  c_loc += c;

  if (x != 0) {
    prepareMovement( 0, -x);
  }
  if (y != 0) {
    prepareMovement( 1, -y);
  }
  if (z != 0) {
    prepareMovement( 2, -z);
  }
  if (a != 0) {
    prepareMovement( 3, -a);
  }
  if (b != 0) {
    prepareMovement( 4, -b);
  }
  if (c != 0) {
    prepareMovement( 5, -c);
  }
  
  runAndWait();
}

void loop() {
  // while(checkEstop()) {
  //   delay(10);
  // }
  parseCommand();
  
  // else {
  //   Serial.println("Currently Estoped!");
  //   delay(50)
  // }
}


void parseCommand() {
  static byte index = 0;
  char endMarker = '\n';
  char command;
  float commands[6];

  
  static char num[64];
  static unsigned int msg_loc = 0;

  while (Serial.available() == 0) {
      //wait for command;
  }

  char buf = Serial.read();
  
  if (buf == 'H') {
    goHome();
    // smoothHome();
  }
  
  else if (buf == 'G') {  

    char com[64];
    int loc = 0;

    delay(100);
    while (Serial.available() > 0) {

      char buf2 = Serial.read();

      if (buf2 == '\n') {
        com[loc] = '\0';

        loc = 0;
      }
      else {
        com[loc] = buf2;
        ++loc;          
      }
              
    }
  
    char *tknInd;
    char *buffer;
    char **rem;
    tknInd = strtok(com, " ");
    commands[0] = atof(tknInd);
    
    for (int i = 1; i < 6; ++i) {
        tknInd = strtok(NULL, " ");
        commands[i] = atof(tknInd);
         
    }
    float speed = atof(strtok(NULL, " "));
    
    moveDPKM(commands[0], commands[1], commands[2], commands[3], commands[4], commands[5], speed);
    
  } 
  
  else if (buf == 'A') {  

    char com[64];
    int loc = 0;

    delay(100); //maybe this is why it's so slow?
    while (Serial.available() > 0) {

      char buf2 = Serial.read();

      if (buf2 == '\n') {
        com[loc] = '\0';

        loc = 0;
      }
      else {
        com[loc] = buf2;
        ++loc;          
      }
              
    }
    int locs[6];
    locs[0] = x_loc;
    locs[1] = y_loc;
    locs[2] = z_loc;
    locs[3] = a_loc;
    locs[4] = b_loc;
    locs[5] = c_loc;
    char *tknInd;
    char *buffer;
    char **rem;
    tknInd = strtok(com, " ");
    commands[0] = atof(tknInd) - locs[0];
    
    for (int i = 1; i < 6; ++i) {
        tknInd = strtok(NULL, " ");
        commands[i] = atof(tknInd) - locs[i];
         
    }
    float speed = atof(strtok(NULL, " "));
    
      moveDPKM(commands[0], commands[1], commands[2], commands[3], commands[4], commands[5], speed);
    Serial.println("Move Complete");


  } else if (buf == 'P') {
    printLoc();
  } else if (buf == 'R') {
    for (int i = 0; i < 6; ++i) {
      x_loc = 0;
      y_loc = 0;
      z_loc = 0;
      a_loc = 0;
      b_loc = 0;
      c_loc = 0;
    }

    
  } else if (buf == 'D') {  

    char com[64];
    int loc = 0;

    delay(100);
    while (Serial.available() > 0) {

      char buf2 = Serial.read();

      if (buf2 == '\n') {
        com[loc] = '\0';

        loc = 0;
      }
      else {
        com[loc] = buf2;
        ++loc;          
      }
              
    }
  
    char *tknInd;
    char *buffer;
    char **rem;
    tknInd = strtok(com, " ");
    int mode = atoi(tknInd);
    demo(mode);
  }
}

void printLoc() {
  // Serial.print("");
  Serial.print(x_loc);
  Serial.print(", ");
  Serial.print(y_loc);
  Serial.print(", ");
  Serial.print(z_loc);
  Serial.print(", ");
  Serial.print(a_loc);
  Serial.print(", ");
  Serial.print(b_loc);
  Serial.print(", ");
  Serial.println(c_loc);
  // printout the current step counts of each motor
  return 0;
}


bool checkEstop() {
  int Estop = digitalRead(EstopPin);
  // Read from e stop pin and return the result
  return Estop;
}

void demo(int mode) {
  goHome();
  if (mode == 1) {
    moveDPKM(100, 100, 100, 100, 100, 100, 40);
    moveDPKM(30, 30, 0, 0, -30, -30, 40);
    moveDPKM(-60, -60, 0, 0, 60, 60, 40);
    moveDPKM(30, 30, 0, 0, -30, -30, 40);
    moveDPKM(40, 40, -30, -30, 40, 40, 40);
    moveDPKM(-40, -40, 30, 30, -40, -40, 40);
  }
  goHome();
  

}



