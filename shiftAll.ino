#include <A4988.h>
#include <avr/sleep.h>
#include <EEPROM.h>
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
const unsigned long INACTIVITY_LIM = 5 * 1000; // in ms

//motor constants
const int STEPS = 48; //number of steps per rev (motor specific)
const int RPM = 600; //RPM
const int DISTANCE_STEP_UM = 13; //linear distance in um (DISTANCE_REV * 1000)
const int STEPPER_NATIVE_DIR = -1; //which direction does stepper move for positive numbers? depends on wiring

//bike constants
const int NUMGEARS = 9; //number of gears in rear cassette
const int DOWNHOLDTIME = 200; //how long to hold the button for a downshift
const int DEBOUNCETIME = 20; //software debounce length (in ms)
const int CLICKTIME = 120;
const int TRIMHOLDTIME = 3000; //how long to hold a button for trim settings (ms)
const int TRIMDELAY = 2000; // how long to hold a button for more trim (ms)
const int MINTRIM = 1500; //in um
const int MAXTRIM = 3750; //in um
const int MAXTRAVEL = 30000;//in um stepper specific
const int TRIMTRAVEL = 250; //in um

//Potentionmeter constants
const word POT_MAX = 980; //max value of potentiometer, empirical or set in calibration
const word POT_MIN = 40; //min value of potentiometer, empirical or set in calibration
const word POT_ERROR = 10; //typical error of potentiometer readings
const word POT_k = 34; //linear conversion factor between pot readings and gear location (in mm)

//global variables
//word gearDistances_um[NUMGEARS - 1] = {3200, 2900, 2900, 3100, 2200, 2200, 2600, 2400}; //in um (gearDistances * 1000)
word gearDistances_um[NUMGEARS - 1];
word gearLocation[NUMGEARS]; //linear distance of each gear from x=0
word gearPotLocation[NUMGEARS]; //value of potentiometer at each location
word gearPot_min = 60;//pot gets a bit wonky below this
word gearPot_max; //numerically determined from MAXTRAVEL and gearPot_min
word currentGear = 0; //what gear is the system currently in, initialized to 0 (not possible in program)
word gearChanges = 0;
int shiftDirection = 0;
bool buttonFlag = false;


//Pin definitions
const int UPBUTTON_PIN = 2;//button pin
const int DOWNBUTTON_PIN = 3;//button pin
const word BUTTON_GROUND_PIN = 4;//
const word THREEVOLTPOWER_PIN = 5;
const word TWELVEVOLTPOWER_PIN = 6;
const word SLEEP_PIN = 7;//stepper sleep pin, inverse logic
const word STEP_PIN = 8;//stepper step pin
const word DIR_PIN = 9;//stepper direction pin
const int POT_PIN = A1; //potentiometer pin
const int POT_HIGH_PIN = A2;//set to vcc, swap pot limits
const int POT_GROUND_PIN = A3;//set to ground, swap pot limits

//function prototypes
void runStepper(int); //run the stepper motor a linear distance
bool changeGears(); //decide which gear we're moving to
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
OneButton shiftUpButton = OneButton(UPBUTTON_PIN, true, true);
OneButton shiftDownButton = OneButton(DOWNBUTTON_PIN, true, true);

void setup() {
  word potPosition;   //variable to track the potentiometer reading

  //pinMode(BUTTON_PIN, INPUT_PULLUP);//shift button
  pinMode(BUTTON_GROUND_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT); //potentiometer
  pinMode(POT_GROUND_PIN, OUTPUT); //potentiometer
  pinMode(POT_HIGH_PIN, OUTPUT); //potentiometer
  pinMode(THREEVOLTPOWER_PIN, OUTPUT);
  pinMode(TWELVEVOLTPOWER_PIN, OUTPUT);

  digitalWrite(THREEVOLTPOWER_PIN, HIGH);
  digitalWrite(TWELVEVOLTPOWER_PIN, HIGH);

  digitalWrite(BUTTON_GROUND_PIN, LOW);
  digitalWrite(POT_GROUND_PIN, LOW);
  digitalWrite(POT_HIGH_PIN, HIGH);

  shiftUpButton.attachClick(shiftUp);
  shiftUpButton.attachDoubleClick(shiftUp);
  shiftUpButton.attachMultiClick(shiftUp);
  shiftUpButton.attachLongPressStop(trimGear);
  shiftDownButton.attachClick(shiftDown);
  shiftDownButton.attachDoubleClick(shiftDown);
  shiftDownButton.attachMultiClick(shiftDown);
  shiftDownButton.attachLongPressStop(trimGear);

  shiftUpButton.setDebounceTicks(DEBOUNCETIME);
  shiftUpButton.setClickTicks(CLICKTIME);
  shiftUpButton.setPressTicks(TRIMHOLDTIME);
  shiftDownButton.setDebounceTicks(DEBOUNCETIME);
  shiftDownButton.setClickTicks(CLICKTIME);
  shiftDownButton.setPressTicks(TRIMHOLDTIME);


  Serial.begin(9600);//begin serial comms. 9600 is pretty slow

  stepper.begin(RPM);
  stepper.setSpeedProfile(stepper.CONSTANT_SPEED);//
  stepper.setEnableActiveState(LOW);
  disableMotor();
  //  digitalRead(BUTTON_PIN);
  //  delay(500);
  //  if (!digitalRead(BUTTON_PIN)) {
  //    calibratePot();
  //  }
  readMemory();
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
  unsigned long secondCounter = millis();
  unsigned long inactivity = millis();
  word secs = 0;
  //wait for the flag to change from a button press/click (in attached functions)
  while (!buttonFlag) {
    shiftUpButton.tick();
    shiftDownButton.tick();
    if (millis() - inactivity > INACTIVITY_LIM) {
      DEBUG_PRINTLN("\nSLEEP TIME!");
      sleepTime();
      secs = 0;
      inactivity = millis();
    }
    else if (millis() - secondCounter >= 1000) {
      secs++;
      DEBUG_PRINT(INACTIVITY_LIM / 1000 - secs);
      DEBUG_PRINT("...");
      secondCounter = millis();
    }
    else if (!shiftUpButton.isIdle() || !shiftDownButton.isIdle()) {
      secs = 0;
      secondCounter = millis();
      inactivity = millis();
    }
  }
  DEBUG_PRINT("\nBUTTON CLICKED: ");
  DEBUG_PRINT(gearChanges);
  DEBUG_PRINTLN(" TIMES");
  for (int i = gearChanges; i > 0; i--) {
    if (!changeGears()) break;
  }
  getGearInfo();
  buttonFlag = false; //reset button flag
}
/*FUNCTIONS*/

void shiftUp() {
  gearChanges = shiftUpButton.getNumberClicks();
  for (int i = gearChanges; i > 0; i--) {
    if (gearChanges + currentGear > NUMGEARS) {
      gearChanges--;
    }
    else break;
  }
  buttonFlag = true;
  shiftDirection = 1;
}
void shiftDown() {
  gearChanges = shiftDownButton.getNumberClicks();
  for (int i = gearChanges; i > 0; i--) {
    if ((int)(currentGear - gearChanges) < 1) {
      gearChanges--;
    }
    else break;
  }
  buttonFlag = true;
  shiftDirection = -1;
}

void enableMotor() {
  digitalWrite(TWELVEVOLTPOWER_PIN, HIGH);
  delay(5);
  stepper.enable();
}

void disableMotor() {
  stepper.disable();
  digitalWrite(TWELVEVOLTPOWER_PIN, LOW);
  delay(5);
}

void readMemory() {
  word eepromValue;
  for (int i = 0; i < NUMGEARS - 1; i++) {
    eepromValue = EEPROM.read(i) * 100 / 2;
    if (eepromValue >= MINTRIM && eepromValue <= MAXTRIM) {
      DEBUG_PRINT("VALID VALUE, LOADING FROM EEPROM: ");
      gearDistances_um[i] = eepromValue;
    }
    else {
      DEBUG_PRINT("ERROR WITH EEPROM -- ADDRESS: ");
      DEBUG_PRINT(i);
      DEBUG_PRINT(" VALUE: ");
      DEBUG_PRINTLN(eepromValue);
      DEBUG_PRINT("RESTORING DEFAULT VALUE ");
      EEPROM.update(i, 2500 * 2 / 100);
      gearDistances_um[i] = 2500;
    }
    DEBUG_PRINTLN(gearDistances_um[i]);
  }
}

void updateEEPROM(word address) {
  word eepromValue = EEPROM.read(address) * 100 / 2;
  DEBUG_PRINT("EEPROM ADDRESS: ");
  DEBUG_PRINT(address);
  DEBUG_PRINT(" WITH VALUE: ");
  DEBUG_PRINTLN(eepromValue);
  if (gearDistances_um[address] == eepromValue) {
    DEBUG_PRINT("IS THE SAME AS ");
    DEBUG_PRINTLN(gearDistances_um[address]);
    DEBUG_PRINTLN("NO CHANGE MADE");
  }
  else {
    DEBUG_PRINT("IS DIFFERENT THAN ");
    DEBUG_PRINTLN(gearDistances_um[address]);
    DEBUG_PRINTLN("UPDATING EEPROM...");
    EEPROM.update(address, gearDistances_um[address] * 2 / 100);
  }
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
  enableMotor();
  stepper.startMove(STEPPER_NATIVE_DIR * dir * MAXTRAVEL / DISTANCE_STEP_UM);
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
  disableMotor();

}


void wakeUp_ISR() {
  detachInterrupt(digitalPinToInterrupt(UPBUTTON_PIN));
  detachInterrupt(digitalPinToInterrupt(DOWNBUTTON_PIN));
  sleep_disable();//AVR function
}


void sleepTime() {
  // wait for transmit buffer to empty
  Serial.flush ();
  while ((UCSR0A & _BV (TXC0)) == 0) {};
  attachInterrupt(digitalPinToInterrupt(UPBUTTON_PIN), wakeUp_ISR, LOW);//low side button interrupt
  attachInterrupt(digitalPinToInterrupt(DOWNBUTTON_PIN), wakeUp_ISR, LOW);
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
  shiftUpButton.reset();
  shiftDownButton.reset();
  DEBUG_PRINTLN("TRIM MODE");
  inactivity = millis();
  secondCounter = millis();
  DEBUG_PRINT("USE BUTTON TO TRIM GEAR: ");
  DEBUG_PRINTLN(currentGear);
  while (activeTrimFlag) {
    shiftUpButton.tick();
    shiftDownButton.tick();
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
    else if (millis() - inactivity > 10000) {
      DEBUG_PRINTLN("EXITING TRIM");
      delay(1000);
      activeTrimFlag = false;
    }
    else if (millis() - secondCounter >= 1000) {
      secs++;
      DEBUG_PRINT(10 - secs);
      DEBUG_PRINT("...");
      secondCounter = millis();
    }
  }
  updateEEPROM(currentGear - 2);
  getGearInfo();
}

bool changeGears() {
  word newGear;//what gear does user want to move to?
  int gearDistanceIndex;//what gap do we have to cross to get there (what index of gear distances array)
  bool success = true;

  newGear = currentGear + shiftDirection;
  gearDistanceIndex = currentGear + (shiftDirection - 3) / 2;
  DEBUG_PRINT("CURRENT GEAR: ");
  DEBUG_PRINT(currentGear);
  DEBUG_PRINT(", NEW GEAR: ");
  DEBUG_PRINT(newGear);
  DEBUG_PRINT(", DISTANCE INDEX: ");
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
      runStepperTarget(gearPotLocation[currentGear - 1]);//i don't take any action if this fails. Probably need to...
    }
  }
  else {
    DEBUG_PRINTLN("End of cassette");
    delay(DEBOUNCETIME);
  }
  return success;

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
  enableMotor();
  stepper.move(numSteps);
  disableMotor();
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
  enableMotor();
  stepper.startMove(STEPPER_NATIVE_DIR * dir * MAXTRIM * 2 / 13);
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
  disableMotor();
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
