#include "Arduino.h"
#include "NewPing.h"
#include <Servo.h>

#define DUBUG_MODE

const int motorRightPWM = 11;
const int motorRightA = 9;
const int motorRightB = 10;

const int motorLeftPWM = 6;
const int motorLeftA = 7;
const int motorLeftB = 8;

const int leftServoPWM = 3;
const int rightServoPWM = 5;
Servo leftServo;
Servo rightServo;

const int slapperSwitch = 25;

const int frontTrig = 12;
const int frontEcho = 2;

const int leftTrig = 34;
const int leftEcho = 30;
 
const int rightTrig = 13;
const int rightEcho = 4; 

// -- MARK: PROGRAM CONFIGURATION
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

NewPing frontSonar(frontTrig, frontEcho, MAX_DISTANCE);
NewPing leftSonar(leftTrig, leftEcho, MAX_DISTANCE);
NewPing rightSonar(rightTrig, rightEcho, MAX_DISTANCE);
long frontDistance = 10000;
long leftDistance = 10000;
long rightDistance = 10000;

long leftOldDistance = 10000;
long rightOldDistance = 10000;

long startTime = 0;

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

  startTime = millis();
}

// - MARK: MOTOR SPEEDS

// -255 to 255
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

int slapUpTime = 450;

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

// - MARK: MAIN LOOP

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

 slapDown();
 slapUp();
} 
