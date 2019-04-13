#include "Arduino.h"
#include "NewPing.h"
#include <Servo.h>

#define DUBUG_MODE

// -- MARK: PIN Definitions

// Motors
const int motorRightPWM = 11;
const int motorRightA = 9;
const int motorRightB = 10;

const int motorLeftPWM = 6;
const int motorLeftA = 7;
const int motorLeftB = 8;

// Slapper b
const int slapperSwitch = 25;
const int leftServoPWM = 3;
const int rightServoPWM = 5;
Servo leftServo;
Servo rightServo;

// Ultrasonic
const int leftTrig = 34;
const int leftEcho = 30;

const int frontTrig = 12;
const int frontEcho = 2;
 
const int rightTrig = 13;
const int rightEcho = 4;

// IR
const int IRA1_PIN = A15;
const int IRA2_PIN = A14;
const int IRA3_PIN = A13;
const int IRA4_PIN = A12;
const int IRA5_PIN = A11;

// Switch pins
const int TURN_DIRECTION_PIN = 29;

// -- MARK: PROGRAM VARIABLES
const int MAX_DISTANCE = 100; // cm
const int WALL_DETECT_THRESHOLD = 20;
const int FRONT_WALL_DETECT_THRESHOLD = 18; 
const int WALL_CENTERING_DISTANCE = 15; // 7cm - only used when only one wall is detected
const int CANDLE_WALL_OVERRIDE_DISTANCE = 7;
const int TURN_TIME = 700; // ms
const int speed =  160;
bool defaultTurnIsLeft = true;

enum ProgramMode {maze, candleHunting, extinguish};
ProgramMode programMode = maze;
enum TurnDirection {counterclockwise, clockwise, aroundCW, aroundCCW};

// Ultrasonic
NewPing frontSonar(frontTrig, frontEcho, MAX_DISTANCE);
NewPing leftSonar(leftTrig, leftEcho, MAX_DISTANCE);
NewPing rightSonar(rightTrig, rightEcho, MAX_DISTANCE);
long frontDistance = 10000;
long leftDistance = 10000;
long rightDistance = 10000;

long leftOldDistance = 10000;
long rightOldDistance = 10000;

// IR
enum CandleDirection { candle_none, candle_left, candle_center, candle_right };
CandleDirection candleDirection = candle_none;
int IR1 = 0;
int IR2 = 0;
int IR3 = 0;
int IR4 = 0;
int IR5 = 0;
// For any pin, when should we detect that a candle is within our sight?
const int CANDLE_DETECT_THRESHOLD = 500;

// Slapper
int slapUpTime = 270;

// Program start delay
long startDelay = 2000;
long startTime = 0;
long lastCandleDetection = 0;
long candleTimeoutTime = 4000;

long candleNotCenteredTime = 0;
long candleCenteredExtinguishStart = 1500;
// Timer to start candle hunting
long candleNotDetectedTime = 0;
long candleDetectionDelay = 500;

int extinguishCount = 0;

bool firstStart = true;


// -- MARK: FUNCTION DEFINITIONS
// Motors
void setLeft(int s);
void setRight(int s);

// Slapper
void slapDown();
void slapUp();

// Turning
void turn(TurnDirection direction);
void crawl(bool hard, TurnDirection direction);

// Moving
void moveForwards();

// Ultrasonic
void scanDistances();
void verifyDistances();
void printDistances();

// IR
void scanIR();
void scanCandleDirection();

// -- MARK: FUNCTION IMPLEMENTATION

void setup() {
  Serial.begin(9600);
  while (!Serial) {} // Wait for serial to connect
  Serial.println("Awake!");

  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);

  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);

  pinMode(slapperSwitch, INPUT);

  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, HIGH);

  leftServo.attach(leftServoPWM);
  rightServo.attach(rightServoPWM);

  leftServo.write(90);
  rightServo.write(88);

  // Setup default turn direction from switch
  defaultTurnIsLeft = true;

  delay(2000);
}

void loop() {
  // Motor Test
  
  /*
  setRight(255);
  setLeft(255);
  delay(2000);
  setRight(-255);
  setLeft(-255);
  delay(2000);
  */

  // Slapper Test
  
  /*
  slapDown();
  slapUp();
  */
  

  // Candle Test
  

  /*
  scanCandleDirection();
  String ir1 = String(IR1);
  String ir2 = String(IR2);
  String ir3 = String(IR3);
  String ir4 = String(IR4);
  String ir5 = String(IR5);
  Serial.println(" " + ir1 + " " + ir2 + " " + ir3 + " " + ir4 + " " + ir5);
  switch (candleDirection) {
    case candle_none:
      Serial.println("No Candle Detected");
      break;
    case candle_left:
      Serial.println("Candle on Left");
      break;
    case candle_center:
      Serial.println("Candle in Center");
      break;
    case candle_right:
      Serial.println("Candle on Right");
      break;
  }

  delay(100);
  */
 
 /*
 // Test ultrasonic
 scanDistances();
 Serial.println(" " + String(leftDistance) + " " + String(frontDistance) + " " + String(rightDistance));
 delay(100);
 */

  /*
 
  // Test Speed
  setLeft(speed);
  setRight(speed);
  delay(1000);
  turn(clockwise);
  setLeft(speed);
  setRight(speed);
  delay(1000);
  turn(clockwise);
  setLeft(speed);
  setRight(speed);
  delay(1000);
  turn(clockwise);
  setLeft(speed);
  setRight(speed);
  delay(1000);
  turn(clockwise);

  */

  /*
  // Test Turning
  turn(clockwise);
  setRight(0);
  setLeft(0);
  delay(1000);
  */

 
  int currentTime = millis();

  scanCandleDirection();
  scanDistances();

  if (firstStart) {
    firstStart = false;
    setLeft(speed);
    setRight(speed);
    delay(1000);
    return;
  }

  if (programMode == extinguish) {

    if (extinguishCount == 0) {
      extinguishCount += 1;
    } else {
      slapDown();
      slapUp();
      delay(250);
    }
    
    while(true) {
      scanCandleDirection();
      Serial.println("Scan Can D");
      if (candleDirection == candle_left) {
        // Spin counterclockwise
        setLeft(-140);
        setRight(140);
      } else if (candleDirection == candle_right) {
        // Spin clockwise
        setLeft(140);
        setRight(-140);
      } else if (candleDirection == candle_none) {
        setLeft(-140);
        setRight(140);
      } else {
        break;
      }
      delay(50);
    }
    setLeft(150);
    setRight(150);
    delay(250);
    setLeft(0);
    setRight(0);
    return;
  }

  if (programMode == candleHunting) {
    // Spin slowly in a circle if we don't detect a candle
    if (candleDirection == candle_none) {
      // If we pass the candleTimeoutTime, reset the program/
      if (currentTime - lastCandleDetection > candleTimeoutTime) {
        programMode = maze;
        return;
      }

      // Spin counterclockwise slowly
      setLeft(-145); 
      setRight(145);
    } else {
      // Reset candle timout
      lastCandleDetection = currentTime;

      // Wall Override
      if (leftDistance < CANDLE_WALL_OVERRIDE_DISTANCE) {
        crawl(true, clockwise);
        return;
      } else if (rightDistance < CANDLE_WALL_OVERRIDE_DISTANCE) {
        crawl(true, counterclockwise);
        return;
      }

      // Move towards candle
      if (candleDirection == candle_left) {
        // Spin counterclockwise
        setLeft(-140);
        setRight(140);
        candleNotCenteredTime = currentTime;
      } else if (candleDirection == candle_right) {
        // Spin clockwise
        setLeft(140);
        setRight(-140);
        candleNotCenteredTime = currentTime;
      } else {

        programMode = extinguish;
        return;
      }
    }

    delay(30);

    return;
  }

  if (candleDirection != candle_none) {
    delay(800);
    programMode = candleHunting;
    return;
  }

  // Turn if there's a wall in front of us
  if (frontDistance < FRONT_WALL_DETECT_THRESHOLD) {
    if (rightDistance < WALL_DETECT_THRESHOLD && leftDistance < WALL_DETECT_THRESHOLD) {
      Serial.println("Turning Around");
      turn(aroundCCW);
    } else if (rightDistance < WALL_DETECT_THRESHOLD) {
      Serial.println("Turning Left");
      turn(counterclockwise);
    } else if (leftDistance < WALL_DETECT_THRESHOLD) {
      Serial.println("Turning Right");
      turn(clockwise);
    } else {
      Serial.println("Turning Left");
      turn(counterclockwise);
    }
  }

  // Right hand rule
  if (rightDistance > 35) {
    crawl(true, clockwise);
    // Serial.println("Turning Right with Delay");
    // setRight(speed);
    // setLeft(speed);
    // delay(600);
    // turn(clockwise);
    // setRight(speed);
    // setLeft(speed);
    // delay(500);
  } else if (rightDistance < 27 && rightDistance < WALL_CENTERING_DISTANCE) {
    crawl(false, counterclockwise);
  } else if (rightDistance < 27 && rightDistance > WALL_CENTERING_DISTANCE) {
    crawl(false, clockwise);
  } else {
    Serial.println("Going Straight");
    setRight(speed);
    setLeft(speed);
  }

  delay(30);
  scanDistances();
}

void setLeft(int s) {
  if (s > 0) {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, HIGH);
  } else if (s < 0) {
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
  } else {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, LOW);
  }

  s = abs(s);
  analogWrite(motorLeftPWM, s);
}

void setRight(int s) {
  if (s > 0) {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
  } else if (s < 0)  {
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
  } else {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, LOW);
  }

  s = abs(s);
  analogWrite(motorRightPWM, s);
}

void slapDown() {
    leftServo.write(0);
    rightServo.write(180);
    while(true) {
      Serial.println("Looking for switch");
      if (digitalRead(slapperSwitch) == LOW) {
        Serial.println("Detected Switch");
        break;
      }
    }
    leftServo.write(90);
    rightServo.write(88);
    Serial.println("Slapped Down");
}

void slapUp() {
    leftServo.write(180);
    rightServo.write(0);
    delay(slapUpTime);
    leftServo.write(90);
    rightServo.write(88);
    Serial.println("Slapped Up");
}

// - MARK: TURNING

void turn(TurnDirection direction) {
  switch (direction) {
  case counterclockwise:
    setLeft(-speed);
    setRight(speed);
    delay(TURN_TIME);
    break;
  case clockwise:
    setLeft(speed);
    setRight(-speed);
    delay(TURN_TIME);
    break;
  case aroundCW:
    setLeft(speed);
    setRight(-speed);
    delay(TURN_TIME * 2);
    break;
  case aroundCCW:
    setLeft(-speed);
    setRight(speed);
    delay(TURN_TIME * 2);
    break;
  }
}

void crawl(bool hard, TurnDirection direction) {
  int fastSpeed = speed;
  int slowSpeed;
  if (hard) {
    slowSpeed = fastSpeed - 70;
    fastSpeed = fastSpeed + 20;
  } else {
    slowSpeed = fastSpeed - 40;
  }

  switch (direction) {
  case clockwise:
      setLeft(fastSpeed);
      setRight(slowSpeed);
      Serial.println("Crawling Right");
      break;
  case counterclockwise:
      setLeft(slowSpeed);
      setRight(fastSpeed);
      Serial.println("Crawling Left");
      break;
  default:
    break;
  }
}

// Moves forward with a basic centering algorithm
void moveForwards() {
  if (rightDistance <= WALL_DETECT_THRESHOLD && leftDistance <= WALL_DETECT_THRESHOLD) {
    if (rightDistance > leftDistance) {
      crawl(false, clockwise);
    } else {
      crawl(false, counterclockwise);
    }
  } else if (rightDistance <= WALL_DETECT_THRESHOLD) {
    if (rightDistance < WALL_CENTERING_DISTANCE) {
      crawl(false, counterclockwise);
    } else {
      crawl(false, clockwise);
    }
  } else if (leftDistance <= WALL_DETECT_THRESHOLD) {
    if (leftDistance < WALL_CENTERING_DISTANCE) {
      crawl(false, clockwise);
    } else {
      crawl(false, counterclockwise);
    }
  } else {
    Serial.println("Crawling Forward");
    setLeft(speed); setRight(speed);
  }
}

void scanDistances() {
  leftOldDistance = leftDistance;
  rightOldDistance = rightDistance;

   // Needs to be a minimum of 29ms between pings to prevent cross-sensor echo according to NewPing documentation.
  frontDistance = frontSonar.ping_cm();
  delay(30);
  leftDistance = leftSonar.ping_cm();
  delay(30);
  rightDistance = rightSonar.ping_cm();

  verifyDistances();
}

void verifyDistances() {
  if (frontDistance == 0) {
    frontDistance = 10000;
  }

  if (leftDistance == 0) {
    leftDistance = 10000;
  }

  if (rightDistance == 0) {
    rightDistance = 10000;
  }
}

void printDistances() {
  Serial.println("Left: " + String(int(leftDistance)));
  Serial.println("Center: " + String(int(frontDistance)));
  Serial.println("Right: " + String(int(rightDistance)));
}



void scanIR() {
    IR1 = analogRead(IRA1_PIN);
    IR2 = analogRead(IRA2_PIN);
    IR3 = analogRead(IRA3_PIN);
    IR4 = analogRead(IRA4_PIN);
    IR5 = analogRead(IRA5_PIN);
}

/*
       IR3
   IR2     IR4
IR1           IR5
*/

void scanCandleDirection() {
  scanIR();
  int leftSum = IR1 + IR2 + IR3;
  int centerSum = IR2 + IR3 + IR4;
  int rightSum = IR3 + IR4 + IR5;

  if (leftSum < CANDLE_DETECT_THRESHOLD && centerSum < CANDLE_DETECT_THRESHOLD && rightSum < CANDLE_DETECT_THRESHOLD) {
    candleDirection = candle_none;
    return;
  }

  if (IR1 < CANDLE_DETECT_THRESHOLD &&
      IR2 < CANDLE_DETECT_THRESHOLD &&
      IR3 < CANDLE_DETECT_THRESHOLD && 
      IR4 < CANDLE_DETECT_THRESHOLD &&
      IR5 < CANDLE_DETECT_THRESHOLD) {
        candleDirection = candle_none;
        return;
      }

  // IR1 is the greatest so left
  if (IR1 > IR2 && IR1 > IR3 && IR1 > IR4 && IR1 > IR5) {
    candleDirection = candle_left;
  } else if (IR2 > IR1 && IR2 > IR3 && IR2 > IR4 && IR2 > IR5) {
    // IR2 is the greatest so left
    candleDirection = candle_left;
  } else if (IR3 > IR1 && IR3 > IR2 && IR3 > IR4 && IR3 > IR5) {
    // IR3 is the greatest so center
    candleDirection = candle_center;
  } else if (IR4 > IR1 && IR4 > IR2 && IR4 > IR3 && IR4 > IR5) {
    // IR4 is the greatest so right
    candleDirection = candle_right;
  } else if (IR5 > IR1 && IR5 > IR2 && IR5 > IR3 && IR5 > IR4) {
    // IR5 is the greatest so right
    candleDirection = candle_right;
  }
}
