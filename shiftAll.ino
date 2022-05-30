//#include <Stepper.h>
#include <A4988.h>
#include <avr/sleep.h>
#include <OneButton.h>

#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

//system constants
const unsigned long INACTIVITY_LIM = 30 * 1000; // in ms


//motor constants
const int STEPS = 48; //number of steps per rev (motor specific)
const int RPM = 500; //RPM
const int DISTANCE_STEP_UM = 13; //linear distance in um (DISTANCE_REV * 1000)

//bike constants
const int NUMGEARS = 9; //number of gears in rear cassette
const int DOWNHOLDTIME = 200; //how long to hold the button for a downshift
const int DEBOUNCETIME = 20; //software debounce length (in ms)
const int CLICKTIME = 120;
const int TRIMHOLDTIME = 3000; //how long to hold a button for trim settings (ms)
const int TRIMDELAY = 2000; // how long to hold a button for more trim (ms)
const int MINTRIM = 1000; //in um
const int MAXTRIM = 3500; //in um
const int MAXTRAVEL = 30000;//in um stepper specific
const int TRIMTRAVEL = 250; //in um

//Potentionmeter constants
const word POT_MAX = 980; //max value of potentiometer, empirical or set in calibration
const word POT_MIN = 40; //min value of potentiometer, empirical or set in calibration
const word POT_ERROR = 10; //typical error of potentiometer readings
const word POT_k = 34; //linear conversion factor between pot readings and gear location (in mm)

//global variables
word gearDistances_um[NUMGEARS - 1] = {3200, 2900, 2900, 3100, 2200, 2200, 2600, 2400}; //in um (gearDistances * 1000)
word gearLocation[NUMGEARS]; //linear distance of each gear from x=0
word gearPotLocation[NUMGEARS]; //value of potentiometer at each location
word gearPot_min = 60;//pot gets a bit wonky below this
word gearPot_max; //numerically determined from MAXTRAVEL and gearPot_min
word currentGear = 0; //what gear is the system currently in, initialized to 0 (not possible in program)
int shiftDirection = 0;
bool buttonFlag = false; 

//Pin definitions
const int BUTTON_PIN = 2;//button pin
const word BUTTON_GROUND_PIN = 3;//
const word DIR_PIN = 4;//stepper direction pin
const word STEP_PIN = 5;//stepper step pin
const word SLEEP_PIN = 6;//stepper sleep pin, inverse logic
const int POT_PIN = A0; //potentiometer pin
const int POT_HIGH_PIN = A1;//set to vcc, swap pot limits
const int POT_GROUND_PIN = A2;//set to ground, swap pot limits

//function prototypes
void runStepper(int); //run the stepper motor a linear distance
void changeGears(); //decide which gear we're moving to
int buttonHoldCheck(); //is the user holding the button down? (used for differentiating
//up and down shifts on a single button)
void trimGear();//adjust the gear change distances
void calibratePot();
bool setGearLocations(int change = 0);//set locations from gear offsets (the preffered metric). overloaded for trim functionality
void shiftUp();
void shiftDown();


void wakeUp_ISR();



// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
A4988 stepper(STEPS, DIR_PIN, STEP_PIN, SLEEP_PIN);//, MODE0, MODE1, MODE2);
OneButton shiftButton = OneButton(BUTTON_PIN, true, true);

void setup() {
  word potPosition;   //variable to track the potentiometer reading

  //pinMode(BUTTON_PIN, INPUT_PULLUP);//shift button
  pinMode(BUTTON_GROUND_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT); //potentiometer
  pinMode(POT_GROUND_PIN, OUTPUT); //potentiometer
  pinMode(POT_HIGH_PIN, OUTPUT); //potentiometer

  digitalWrite(BUTTON_GROUND_PIN, LOW);
  digitalWrite(POT_GROUND_PIN, LOW);
  digitalWrite(POT_HIGH_PIN, HIGH);

  shiftButton.attachClick(shiftUp);
  shiftButton.attachDoubleClick(shiftDown);
  shiftButton.attachLongPressStop(trimGear);

  shiftButton.setDebounceTicks(DEBOUNCETIME);
  shiftButton.setClickTicks(CLICKTIME);
  shiftButton.setPressTicks(TRIMHOLDTIME);


  Serial.begin(9600);//begin serial comms. 9600 is pretty slow

  stepper.begin(RPM);
  stepper.setSpeedProfile(stepper.CONSTANT_SPEED);//
  stepper.disable();
  //  digitalRead(BUTTON_PIN);
  //  delay(500);
  //  if (!digitalRead(BUTTON_PIN)) {
  //    calibratePot();
  //  }
  setGearLocations();
  analogRead(POT_PIN);//read off capacitance
  potPosition = analogRead(POT_PIN);
  potPosition = constrain(potPosition, gearPot_min, gearPot_max);//constraing pin to these empirical (testing) values
  DEBUG_PRINT("Current Pot Position: ");
  DEBUG_PRINTLN(analogRead(POT_PIN));
  DEBUG_PRINT("CONSTRAINED TO: ");
  DEBUG_PRINT(gearPot_min);
  DEBUG_PRINT(", ");
  DEBUG_PRINTLN(gearPot_max);
  currentGear = map(potPosition, gearPot_min, gearPot_max, 1, NUMGEARS);
  getGearInfo();
}

void loop() {
  //unsigned long secondCounter;= millis()
  unsigned long inactivity= millis();
  word secs = 0;

  //wait for the flag to change from a button press/click (in attached functions)
  while (!buttonFlag) {
    shiftButton.tick();
    if (millis() - inactivity > INACTIVITY_LIM) {
      DEBUG_PRINTLN("\nSLEEP TIME!");
      delay(100);
      sleepTime();
      secs = 0;
      inactivity = millis();
    }
    //    else if (millis() - secondCounter >= 1000) {
    //      secs++;
    //      DEBUG_PRINT(INACTIVITY_LIM / 1000 - secs);
    //      DEBUG_PRINT("...");
    //      secondCounter = millis();
    //    }
  }
  changeGears();
  getGearInfo();
  buttonFlag = false; //reset button flag
}
/*FUNCTIONS*/

void shiftUp() {
  buttonFlag = true;
  shiftDirection = 1;
}
void shiftDown() {
  buttonFlag = true;
  shiftDirection = -1;
}

bool setGearLocations(int change) {
  bool success = true;
  word gearPos = 0;
  if (change != 0) {
    DEBUG_PRINT("CURRENT GEAR: ");
    DEBUG_PRINTLN(currentGear);
    int newLocation = (int)gearLocation[currentGear - 1] + change;
    int newPotLocation = ((long)newLocation * POT_k / 1000) + gearPot_min;
    DEBUG_PRINT("NEW LOCATION: ");
    DEBUG_PRINTLN(newLocation);
    DEBUG_PRINT("NEW POT LOCATION: ");
    DEBUG_PRINTLN(newPotLocation);
    if (newLocation < 0 || newLocation > MAXTRAVEL || newPotLocation < POT_MIN || newPotLocation > POT_MAX) {
      success = false;
    }
    else {
      int dir = (change > 0) - (change < 0);
      DEBUG_PRINT("CHANGING GEAR: ");
      DEBUG_PRINT(currentGear);
      DEBUG_PRINT(" TO LOCATION: ");
      DEBUG_PRINTLN(newLocation);
      gearLocation[currentGear - 1] = newLocation;
      gearPotLocation[currentGear - 1] = newPotLocation;
      if (currentGear != 1) {
        gearDistances_um[currentGear - 2] += change;
        DEBUG_PRINT("CHANGING DISTANCE: ");
        DEBUG_PRINTLN(gearDistances_um[currentGear - 2]);
      }
      else gearPos = newLocation;
    }
  }
  if (success) {
    for (int i = 0; i < NUMGEARS - 1; i++) {
      gearLocation[i] = gearPos;
      gearPos += gearDistances_um[i];
    }
    gearLocation[NUMGEARS - 1] = gearPos;
    for (int i = 0; i < NUMGEARS; i++) {
      gearPotLocation[i] = ((long)gearLocation[i] * POT_k / 1000) + gearPot_min;
    }
    gearPot_max = gearPotLocation[NUMGEARS - 1];
  }
  return success;
}

void getGearInfo() {
  DEBUG_PRINTLN("Gear Distances (um): ");
  for (int i = 0; i < NUMGEARS - 1; i++)
  {
    DEBUG_PRINT('\t');
    DEBUG_PRINT(gearDistances_um[i]);
  }
  DEBUG_PRINT("\n");
  DEBUG_PRINTLN("Gear Locations (um): ");
  for (int i = 0; i < NUMGEARS; i++)
  {
    DEBUG_PRINT('\t');
    DEBUG_PRINT(gearLocation[i]);
  }
  DEBUG_PRINT("\n");
  DEBUG_PRINTLN("Gear Pot Locations: ");
  for (int i = 0; i < NUMGEARS; i++)
  {
    DEBUG_PRINT('\t');
    DEBUG_PRINT(gearPotLocation[i]);
  }
  DEBUG_PRINT("\n");
  DEBUG_PRINT("CURRENT GEAR: ");
  DEBUG_PRINTLN(currentGear);

}

void calibratePot() {
  unsigned long wait_time_micros;
  int dir;
  if (analogRead(POT_PIN) <= gearPot_min) {
    dir = 1;
  }
  else dir = -1;
  DEBUG_PRINTLN("RUNNING TO MIN");
  stepper.enable();
  stepper.startMove(dir * MAXTRAVEL / DISTANCE_STEP_UM);
  while (1) {
    wait_time_micros = stepper.nextAction();
    if (wait_time_micros <= 0) {
      stepper.stop();
    }
    else if (wait_time_micros > 100) {
      if (dir < 0 && analogRead(POT_PIN) <= gearPot_min || dir > 0 && analogRead(POT_PIN) >= gearPot_min) {
        stepper.stop();
        break;
      }
    }
  }
  stepper.disable();
}


void wakeUp_ISR() {
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  sleep_disable();//AVR function
}


void sleepTime() {
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeUp_ISR, LOW);//low side button interrupt
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  DEBUG_PRINTLN("WAKE UP!");
  buttonFlag = false;
}

//trimGear will adjust the location of the current gear in small increments
void trimGear() {
  int gearDistanceIndex;
  word newGap;//the new shift distance
  int trimDirection;
  int gearArrayIndex;
  bool activeTrimFlag = true;
  bool allowTrim;
  unsigned long secondCounter;
  unsigned long inactivity;
  word secs = 0;
  buttonFlag = false;
  shiftButton.reset();
  DEBUG_PRINTLN("TRIM MODE");
  inactivity = millis();
  secondCounter = millis();
  DEBUG_PRINT("USE BUTTON TO TRIM GEAR: ");
  DEBUG_PRINTLN(currentGear);
  while (activeTrimFlag) {
    shiftButton.tick();
    if (buttonFlag) {
      trimDirection = shiftDirection;
      DEBUG_PRINT("ATTEMPTING TRIM BY: ");
      DEBUG_PRINTLN(TRIMTRAVEL * trimDirection);
      allowTrim = setGearLocations(TRIMTRAVEL * trimDirection);
      if (allowTrim) {
        runStepperTarget(gearPotLocation[currentGear - 1]);
        DEBUG_PRINT("GEAR TRIMMED BY: ");
        DEBUG_PRINT(TRIMTRAVEL);
        DEBUG_PRINTLN(" um");
        delay(1000);
      }
      else {
        DEBUG_PRINTLN("MAX TRIM or MAX TRAVEL EXCEEDED. TRIM FAILED");
        delay(1000);
      }
      DEBUG_PRINT("USE BUTTON TO TRIM GEAR: ");
      DEBUG_PRINTLN(currentGear);
      inactivity = millis();
      secondCounter = millis();
      secs = 0;
      buttonFlag = false;
    }
    else if (millis() - inactivity > TRIMHOLDTIME * 2) {
      DEBUG_PRINTLN("EXITING TRIM");
      delay(100);
      activeTrimFlag = false;
    }
    else if (millis() - secondCounter >= 1000) {
      secs++;
      DEBUG_PRINT(TRIMHOLDTIME * 2 / 1000 - secs);
      DEBUG_PRINT("...");
      secondCounter = millis();
    }
  }
}

void changeGears() {
  word newGear;//what gear does user want to move to?
  int gearDistanceIndex;//what gap do we have to cross to get there (what index of gear distances array)
  bool success = true;

  newGear = currentGear + shiftDirection;
  gearDistanceIndex = currentGear + (shiftDirection - 3) / 2;
  DEBUG_PRINT(currentGear);
  DEBUG_PRINT('\t');
  DEBUG_PRINT(newGear);
  DEBUG_PRINT('\t');
  DEBUG_PRINTLN(gearDistanceIndex);

  if (newGear > 0 && newGear <= NUMGEARS) {
    DEBUG_PRINT("Moving to Gear: ");
    DEBUG_PRINTLN(newGear);
    DEBUG_PRINT("By: ");
    DEBUG_PRINTLN(gearDistances_um[gearDistanceIndex]);
    DEBUG_PRINT("TO LOCATION: ");
    DEBUG_PRINTLN(gearLocation[newGear - 1]);
    DEBUG_PRINT("TO POT LOCATION: ");
    DEBUG_PRINTLN(gearPotLocation[newGear - 1]);
    success = runStepperTarget(gearPotLocation[newGear - 1]);
    if (success) {
      currentGear = newGear;
    }
    else {
      DEBUG_PRINTLN("GEAR CHANGE FAILED");
      DEBUG_PRINT("MOVING BACK TO: ");
      DEBUG_PRINTLN(gearPotLocation[currentGear - 1]);
      success = runStepperTarget(gearPotLocation[currentGear - 1]);
    }
  }
  else {
    DEBUG_PRINTLN("End of cassette");
    delay(DEBOUNCETIME);
  }
  return;

}

//given a distance in micrometers
//determine the numbers of steps to turn the stepper motor
void runStepper(int distance) {
  int numSteps; //can be positive or negative
  int divRemainder;
  numSteps = distance / DISTANCE_STEP_UM;
  divRemainder = distance % DISTANCE_STEP_UM;
  if (abs(divRemainder) >= DISTANCE_STEP_UM / 2) {
    int dir = (distance > 0) - (distance < 0);
    numSteps += 1 * dir;
  }
  stepper.enable();
  stepper.move(numSteps);
  stepper.disable();
  DEBUG_PRINT("Steps Taken: ");
  DEBUG_PRINTLN(numSteps);

  return;
}

bool runStepperTarget(int targetPot) {
  int missedSteps;
  int distance;
  int dir = 1;
  int moved;
  int gearDistanceIndex;
  unsigned long moveTime;
  unsigned wait_time_micros;
  bool success = true;
  bool movingFlag = true;
  int initialPot = analogRead(POT_PIN);
  if (initialPot > targetPot) {
    dir = -1;
  }
  distance = initialPot - targetPot;
  distance = abs(distance);
  gearDistanceIndex = currentGear + (dir - 3) / 2;
  DEBUG_PRINT("MOVING FROM: ");
  DEBUG_PRINTLN(initialPot);
  DEBUG_PRINT("TO TARGET: ");
  DEBUG_PRINTLN(targetPot);
  stepper.enable();
  stepper.startMove(dir * MAXTRIM * 2 / 13);
  moveTime = millis();
  while (movingFlag) {
    wait_time_micros = stepper.nextAction();
    if (wait_time_micros <= 0) {
      stepper.stop();
      movingFlag = false;
      success = false;
    }
    if (wait_time_micros > 100) {
      moved = analogRead(POT_PIN) - initialPot;
      moved = abs(moved);
      if (moved >= distance) {
        stepper.stop();
        movingFlag = false;
      }
    }
  }
  stepper.disable();
  distance = gearDistances_um[gearDistanceIndex];
  missedSteps = stepper.getStepsCompleted() - (distance / 13);
  DEBUG_PRINTLN("TARGET ACQUIRED");
  DEBUG_PRINT("POT READING: ");
  DEBUG_PRINTLN(analogRead(POT_PIN));
  DEBUG_PRINT("MISSED STEPS: ");
  DEBUG_PRINTLN(missedSteps);
  moveTime = millis() - moveTime;
  DEBUG_PRINT("Move took: ");
  DEBUG_PRINT(moveTime);
  DEBUG_PRINTLN("ms");
  return success;
}
