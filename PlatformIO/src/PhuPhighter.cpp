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

// Slapper
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

// -- MARK: PROGRAM VARIABLES
const int MAX_DISTANCE = 100; // cm
const int WALL_DETECT_THRESHOLD = 18;
const int FRONT_WALL_DETECT_THRESHOLD = 15; 
const int WALL_CENTERING_DISTANCE = 10; // 7cm - only used when only one wall is detected
const int TURN_TIME = 640; // ms
const int WALL_AVOIDANCE_TIME = 800; // ms
const int speed =  140;
bool defaultTurnIsLeft = true;

enum TurnDirection {counterclockwise, clockwise, aroundCW, aroundCCW};
enum Facing {left, right, forwards};
Facing robotDirection = forwards;

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
int IR1 = 0;
int IR2 = 0;
int IR3 = 0;
int IR4 = 0;
int IR5 = 0;
// For any pin, when should we detect that a candle is within our sight?
const int CANDLE_DETECT_THRESHOLD = 250;

// Program start delay
long startDelay = 0;
// Slapper
int slapUpTime = 450;

// -- MARK: FUNCTION DEFINITIONS
// Motors
void setLeft(int s);
void setRight(int s);

// Slapper
void slapDown();
void slapUp();

// Turning
void rotateRobotDirection(TurnDirection direction);
void turn(TurnDirection direction);
void crawl(TurnDirection direction);

// Moving
void moveForwards();

// Ultrasonic
void verifyDistances();
void printDistances();

// IR
void scanIR();
CandleDirection getCandleDirection();

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

  startDelay = millis();
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
  scanIR();
  String ir1 = String(IR1);
  String ir2 = String(IR2);
  String ir3 = String(IR3);
  String ir4 = String(IR4);
  String ir5 = String(IR5);
  Serial.println(" " + ir1 + " " + ir2 + " " + ir3 + " " + ir4 + " " + ir5);
  CandleDirection direction = getCandleDirection();
  switch (direction) {
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

  delay(1000);
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

 
  Serial.print("Left: " + String(s) + "\n");
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

  
  Serial.print("Right: " + String(s) + "\n");
  s = abs(s);
  analogWrite(motorRightPWM, s);
}

void slapDown() {
    leftServo.write(0);
    rightServo.write(180);
    while(true) {
      if (digitalRead(slapperSwitch) == HIGH) {
        break;
      }
    }
    delay(50);
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

void rotateRobotDirection(TurnDirection direction) {
  switch (robotDirection) {
  case left:
    if (direction == clockwise) {
      robotDirection = forwards;
    }

    if (direction == aroundCCW || direction == aroundCW) {
      robotDirection = right;
    }

    break;
  case right:
    if (direction == counterclockwise) {
      robotDirection = forwards;
    }

    if (direction == aroundCCW || direction == aroundCW) {
      robotDirection = left;
    }

    break;
  case forwards:
    if (direction == counterclockwise) {
      robotDirection = left;
    }

    if (direction == clockwise) {
      robotDirection = right;
    }

    break;
  }
 
  if (robotDirection == forwards) {
    Serial.println("DIR: F");
  } else if (robotDirection == left) {
    Serial.println("DIR: L");
  } else {
    Serial.println("DIR: R");
  }

}

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

  rotateRobotDirection(direction);
}

void crawl(TurnDirection direction) {
  int fastSpeed = speed;
  int slowSpeed = speed - 40;
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
      crawl(clockwise);
    } else {
      crawl(counterclockwise);
    }
  } else if (rightDistance <= WALL_DETECT_THRESHOLD) {
    if (rightDistance < WALL_CENTERING_DISTANCE) {
      crawl(counterclockwise);
    } else {
      crawl(clockwise);
    }
  } else if (leftDistance <= WALL_DETECT_THRESHOLD) {
    if (leftDistance < WALL_CENTERING_DISTANCE) {
      crawl(clockwise);
    } else {
      crawl(counterclockwise);
    }
  } else {
    Serial.println("Crawling Forward");
    setLeft(speed); setRight(speed);
  }
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

CandleDirection getCandleDirection() {
  int leftSum = IR1 + IR2 + IR3;
  int centerSum = IR2 + IR3 + IR4;
  int rightSum = IR3 + IR4 + IR5;

  if (leftSum < CANDLE_DETECT_THRESHOLD && centerSum < CANDLE_DETECT_THRESHOLD && rightSum < CANDLE_DETECT_THRESHOLD) {
    return candle_none;
  }

  if (IR1 < CANDLE_DETECT_THRESHOLD &&
      IR2 < CANDLE_DETECT_THRESHOLD &&
      IR3 < CANDLE_DETECT_THRESHOLD && 
      IR4 < CANDLE_DETECT_THRESHOLD &&
      IR5 < CANDLE_DETECT_THRESHOLD) {
        return candle_none;
      }

  // IR1 is the greatest so left
  if (IR1 > IR2 && IR1 > IR3 && IR1 > IR4 && IR1 > IR5) {
    return candle_left;
  } else if (IR2 > IR1 && IR2 > IR3 && IR2 > IR4 && IR2 > IR5) {
    // IR2 is the greatest so left
    return candle_left;
  } else if (IR3 > IR1 && IR3 > IR2 && IR3 > IR4 && IR3 > IR5) {
    // IR3 is the greatest so center
    return candle_center;
  } else if (IR4 > IR1 && IR4 > IR2 && IR4 > IR3 && IR4 > IR5) {
    // IR4 is the greatest so right
    return candle_right;
  } else if (IR5 > IR1 && IR5 > IR2 && IR5 > IR3 && IR5 > IR4) {
    // IR5 is the greatest so right
    return candle_right;
  }

  return candle_none;
}
