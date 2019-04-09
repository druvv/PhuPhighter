#include "Arduino.h"
#include "NewPing.h"

#define DUBUG_MODE

const int motorRightPWM = 11;
const int motorRightA = 5;
const int motorRightB = 4;

const int motorLeftPWM = 10;
const int motorLeftA = 3;
const int motorLeftB = 2;

const int frontTrig = 7;
const int frontEcho = 6;

const int leftTrig = 9;
const int leftEcho = 8;
 
const int rightTrig = 13;
const int rightEcho = 12; 

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

  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, HIGH);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, HIGH);

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

  s = abs(s);
  //Serial.print("Left: " + String(s));

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
  //Serial.print("Right: " + String(s));

  analogWrite(motorRightPWM, s);
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
  leftOldDistance = leftDistance;
  rightOldDistance = rightDistance;

   // Needs to be a minimum of 29ms between pings to prevent cross-sensor echo according to NewPing documentation.
  frontDistance = frontSonar.ping_cm();
  delay(30);
  leftDistance = leftSonar.ping_cm();
  delay(30);
  rightDistance = rightSonar.ping_cm();

  verifyDistances();
  //printDistances();

  //delay(500);

  if(millis() - startTime < 5000) { return; }

  // If we are facing forwards, move forward until we detect a wall.
  if (robotDirection == forwards && frontDistance <= FRONT_WALL_DETECT_THRESHOLD) {
    int difference = leftDistance - rightDistance;
    // If we don't detect any walls, turn in the default direction.
    if (leftDistance > WALL_DETECT_THRESHOLD && rightDistance > WALL_DETECT_THRESHOLD) {
      Serial.println("T Default");
      if (defaultTurnIsLeft) {
        turn(counterclockwise);
      } else {
        turn(clockwise);
      }
    // If the left wall is closer than the right wall, move right.
    } else if (difference < 0) {
      Serial.println("T Right");
      turn(clockwise);
    // If the right wall is closer than the left wall, move left.
    } else {
      Serial.println("T Left");
      turn(counterclockwise);
    }
  // If we are facing left, move until we don't detect a right wall.
  } else if (robotDirection == left) {
    // If there are walls on all three sides, turn around.
    if (rightDistance <= WALL_DETECT_THRESHOLD && frontDistance <= FRONT_WALL_DETECT_THRESHOLD) {
      turn(aroundCCW);
    } else if (rightDistance > WALL_DETECT_THRESHOLD) {
      if (frontDistance <= 60) {
        while (frontDistance > 10) {
          delay(30);
          frontDistance = frontSonar.ping_cm();
          setRight(speed); setLeft(speed);
          Serial.println("e");
        }
        turn(clockwise);
      } else {
        Serial.println("WA");
        setLeft(speed); setRight(speed);
        delay(WALL_AVOIDANCE_TIME);
        turn(clockwise);
      }
    }
   // If we are facing right, move until we don't detect a left wall. 
  } else if (robotDirection == right) {
    // If there are walls on all three sides, turn around.
    if (leftDistance <= WALL_DETECT_THRESHOLD && frontDistance <= FRONT_WALL_DETECT_THRESHOLD) {
      turn(aroundCW);
    } else if (leftDistance > WALL_DETECT_THRESHOLD) {
      if (frontDistance <= 60) {
        while (frontDistance > 10) {
          delay(30);
          frontDistance = frontSonar.ping_cm();
          setRight(speed); setLeft(speed);
          Serial.println("e");
        }
        turn(counterclockwise);
      } else {
        Serial.println("WA");
        setLeft(speed); setRight(speed);
        delay(WALL_AVOIDANCE_TIME);
        turn(counterclockwise);
      }
    }
  }

  // Move according to centering program
  moveForwards();
  delay(30);
} 


