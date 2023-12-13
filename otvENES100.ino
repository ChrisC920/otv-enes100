#include "Enes100.h"

/* 
IMPORTANT VALUES:
starting x position: 0.35
starting y position: random
starting theta position: random
middle obstacle y position: ~1.00
top obstacle y position: ~1.65
bottom obstacle y position: ~0.40
minimum x position: 0.10
maximum x position: 3.90
minimum y position: 0.10
maximum y position: 1.90
open zone (for sim only) x start: ~0.65
open zone (for sim only) x end: ~1.25
goal zone x end: ~3.00
*/

// defines pins numbers
const int MARKER_ID = 69;
const int TX_PIN = 3;
const int RX_PIN = 17;
// const int LEFT_SENSOR_OUTPUT_PIN = 9;
// const int LEFT_SENSOR_INPUT_PIN = 10;
const int RIGHT_SENSOR_OUTPUT_PIN = 14;
const int RIGHT_SENSOR_INPUT_PIN = 15;
const int LIN_UP = 12;
const int LIN_DOWN = 13;
const int BUMP_SENSOR = A4;
const int PWM_READER = 11;
const int RIGHT_FORWARD = 5;
const int RIGHT_BACKWARD = 6;
const int LEFT_FORWARD = 9;
const int LEFT_BACKWARD = 10;

// defines variables
long duration;
float distance;

// TODO: implement left and right check
float otvX, startingX;
float otvY, startingY;
float otvTheta, startingTheta;
unsigned long startingTime;
unsigned long currentTime;
// To be adjusted
const double STARTING_X = 0.5;
const double BOTTOM_THRESHOLD_Y = 0.7;
const double TOP_THRESHOLD_Y = 1.60;
const double TOP_Y = 1.65;
const double MIDDLE_Y = 1.00;
const double BOTTOM_Y = 0.50;
const double GOAL_ZONE_X = 2.80;
const double OBSTACLE_ZONE_X = 1.25;
const float RANGE_THRESHOLD = 15;
const double END_ZONE = 3.70;
int leftPWM = 0;
int rightPWM = 0;
bool isTopStart = false;

// STUFF FOR PWM READING
int values[100];          // Array to store the last 100 integers
int index = 0;            // Current index in the array
bool matchFound = false;  // Flag to indicate if a match is found
int matchedNumber = 0;    // The number that matched more than 5 times


void goForward(int speed = 230) {
  setSpeed(speed, speed);
}

void setup() {
  Enes100.begin("TEAM PARROTS", DATA, 69, 2, 3);

  // pinMode(LEFT_SENSOR_OUTPUT_PIN, OUTPUT);  // Sets the trigPin as an Output
  // pinMode(LEFT_SENSOR_INPUT_PIN, INPUT);    // Sets the echoPin as an Input
  pinMode(RIGHT_SENSOR_OUTPUT_PIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(RIGHT_SENSOR_INPUT_PIN, INPUT);    // Sets the echoPin as an Input
  pinMode(LIN_UP, OUTPUT);                   // Linear actuator up
  pinMode(LIN_DOWN, OUTPUT);                 // Linear actuuator down
  pinMode(BUMP_SENSOR, INPUT);               // Bump sensor

  pinMode(PWM_READER, INPUT);  // PWM READER
  // pinMode(A2, INPUT); // 2nd arduino?
  // pinMode(A2, )
  pinMode(RIGHT_FORWARD, OUTPUT);   // right forward
  pinMode(RIGHT_BACKWARD, OUTPUT);  // right backwards
  pinMode(LEFT_FORWARD, OUTPUT);    // left forwards
  pinMode(LEFT_BACKWARD, OUTPUT);   // left backwards

  memset(values, 0, sizeof(values));  // Initialize the array with zeros


  delay(3000);

  updateLocation();
  startingX = otvX;
  startingY = otvY;
  startingTheta = otvTheta;
  delay(300);
  digitalWrite(LIN_UP, HIGH);
  delay(7000);
  digitalWrite(LIN_UP, LOW);
  // Navigation algorithm
  if (otvY < MIDDLE_Y) {
    isTopStart = false;
    turnToAngleDegrees(90);
    delay(200);
    // goForward();
    // delay(3000);
    // updateLocation();
    // double angleToTurnTo = atan2(y - otvY, x - otvX);
    // turnToAngleRadians(atan2(startingY - otvY, startingX - otvX));
    // delay(100);
    goUntilBump();
  } else {
    isTopStart = true;
    turnToAngleDegrees(270);
    delay(200);
    // delay(200);
    // goForward();
    // delay(3000);
    // updateLocation();
    // double angleToTurnTo = atan2(y - otvY, x - otvX);
    // turnToAngleRadians(atan2(startingY - otvY, startingX - otvX));
    // delay(100);
    goUntilBump();
  }
  mission();
  setSpeed(-130, -130);
  delay(2000);
  turnToAngleDegrees(0);

  while (!inGoalZone()) {
    goUntilObject(RANGE_THRESHOLD);
    if (inGoalZone()) {
      break;
    }
    if (inTopZone()) {
      turn90DegreesRight();
      updateLocation();
      navigateTo(otvX, MIDDLE_Y);
      turn90DegreesLeft();
      updateLocation();
    } else if (inBottomZone()) {
      turn90DegreesLeft();
      navigateTo(otvX, MIDDLE_Y);
      turn90DegreesRight();
      updateLocation();
    } else {
      int randomNumber = random(0, 2); // 0 means going bottom
      if (randomNumber == 0) {
        turn90DegreesRight();
        navigateTo(otvX, BOTTOM_Y);
        turn90DegreesLeft();
      } else {
        turn90DegreesLeft();
        navigateTo(otvX, TOP_Y);
        turn90DegreesRight();
      }

      updateLocation();
    }

    updateLocation();
  }
  navigateTo(otvX, BOTTOM_Y);
  turnToAngleDegrees(0);
  traverseLog();
  setSpeed(0, 0);
}

void loop() {
  updateLocation();
  Enes100.print("Distance: ");
  Enes100.println(readDistanceSensor());
}

void traverseLog() {
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(LEFT_FORWARD, HIGH);
  while (otvX < END_ZONE) {
    updateLocation();
  }
  goUntilObject(20);
  updateLocation();
}

void goUntilBump() {
  setSpeed(170, 170);
  while (analogRead(BUMP_SENSOR) > 1000) {
    updateLocation();
    if (otvY > 1.85 || otvY < 0.15) {
      break;
    } 
  }
  setSpeed(0, 0);
  updateLocation();
}

void mission() {

  // Enes100.print("MAGNETISM READING: ");
  // Enes100.println(readMagnetism());
  int pwmReadValue = readPWM();
  Enes100.print("PWM READING: ");
  Enes100.println(pwmReadValue);
  Enes100.mission(MAGNETISM, MAGNETIC);
  Enes100.mission(CYCLE, pwmReadValue);
  // if (readMagnetism()) {
  //   Enes100.mission(MAGNETISM, MAGNETIC);
  // } else {
  //   Enes100.mission(MAGNETISM, NOT_MAGNETIC);
  // }

  digitalWrite(LIN_UP, HIGH);
  delay(7000);
  digitalWrite(LIN_UP, LOW);
}

//This function computes the distance from the OSV to the coordinate passed in
float distanceTo(float x, float y) {
  //parameter: float x - the x value of the coordinate
  //parameter: float y - the y value of the coordinate
  //return value: this function should return a float representing how far the OSV is from the coordinate (x,y)
  //Note: after the OSV's location is updated, its location will be stored in the fields enes.location.x and enes.location.y
  //Hint: The Arduino Math functions will be helpful here
  float distance = sqrtf(sq(x - otvX) + sq(y - otvY));
  return distance;
}

float readDistanceSensor() {
  // Clears the trigPin
  digitalWrite(RIGHT_SENSOR_OUTPUT_PIN, LOW);
  delayMicroseconds(5);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(RIGHT_SENSOR_OUTPUT_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_SENSOR_OUTPUT_PIN, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(RIGHT_SENSOR_INPUT_PIN, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Prints the distance on the Serial Monitor
  return distance;
}

// Checks if an object is detected within a certain range in meters
bool isObjectDetected(float range) {
  return readDistanceSensor() < range;
}

// This function will turn the tank to a given angle.
void turnToAngleRadians(float angle, float moe = 0.08, int speed = 100) {  // mode 0 is default turn, mode 1 is drive and turn
  updateLocation();
  // Define a constant for the margin of error
  float MARGIN_OF_ERROR = moe * PI;

  // Calculate the angle difference
  float angleDifference = angle - otvTheta;

  // Ensure the angle difference is within the range of -PI to PI
  while (angleDifference > PI) {
    angleDifference -= 2 * PI;
  }
  while (angleDifference < -PI) {
    angleDifference += 2 * PI;
  }

  // Determine which way to turn (right or left)
  int leftPWM, rightPWM;
  if (angleDifference > MARGIN_OF_ERROR) {  // turn left
    leftPWM = -speed;
    rightPWM = speed;
  } else if (angleDifference < -MARGIN_OF_ERROR) {  // turn right
    leftPWM = speed;
    rightPWM = -speed;
  } else {
    // The angle is within the desired range
    leftPWM = 0;
    rightPWM = 0;
  }


  // Continue updating the tank's position while turning
  while (angleDifference > MARGIN_OF_ERROR || angleDifference < -MARGIN_OF_ERROR) {
    setSpeed(leftPWM, rightPWM);
    updateLocation();

    // Recalculate the angle difference
    angleDifference = angle - otvTheta;

    // Ensure it's within the range of -PI to PI
    while (angleDifference > PI) {
      angleDifference -= 2 * PI;
    }
    while (angleDifference < -PI) {
      angleDifference += 2 * PI;
    }
  }

  // Stop the tank when the desired angle is reached
  setSpeed(0, 0);
  updateLocation();
}

void turnToAngleDegrees(float angle) {
  turnToAngleRadians(radians(angle));
}

// This function will turn the OSV 90 degrees left
void turn90DegreesLeft() {
  //This function should turn the OSV 90 degrees left
  //Hint: Can you use the function turnToAngle() to make writing this function easier?
  turnToAngleRadians(otvTheta + radians(90));
  updateLocation();
}

// This function will turn the OSV 90 degrees right
void turn90DegreesRight() {
  //This function should turn the OSV 90 degrees right
  //Hint: Can you use the function turnToAngle() to make writing this function easier?
  turnToAngleRadians(otvTheta - radians(90));
  updateLocation();
}

// This function will move the OSV to a specific coordinate
void navigateTo(float x, float y) {
  const float MARGIN_OF_ERROR_X = 0.4;
  const float MARGIN_OF_ERROR_Y = 0.4;

  // Calculate the angle to turn to reach the target coordinate
  updateLocation();
  double angleToTurnTo = atan2(y - otvY, x - otvX);
  turnToAngleRadians(angleToTurnTo);

  // Set the motors to move forward
  goForward();

  // Continue moving until the OSV reaches the target coordinate
  while (fabs(otvX - x) > MARGIN_OF_ERROR_X || fabs(otvY - y) > MARGIN_OF_ERROR_Y) {
    // Update the OSV's position
    updateLocation();
    if (isObjectDetected(10)) {
      break;
    }
  }
  angleToTurnTo = atan2(y - otvY, x - otvX);
  turnToAngleRadians(angleToTurnTo, 0.1, 150);
  goForward(230);
  while (fabs(otvX - x) > MARGIN_OF_ERROR_X - 0.2 || fabs(otvY - y) > MARGIN_OF_ERROR_Y - 0.2) {
    // Update the OSV's position
    updateLocation();
    if (isObjectDetected(10)) {
      break;
    }
  }

  angleToTurnTo = atan2(y - otvY, x - otvX);
  turnToAngleRadians(angleToTurnTo, 0.07, 100);
  goForward(160);
  while (fabs(otvX - x) > MARGIN_OF_ERROR_X - 0.3 || fabs(otvY - y) > MARGIN_OF_ERROR_Y - 0.3) {
    // Update the OSV's position
    updateLocation();
    if (isObjectDetected(10)) {
      break;
    }
  }
  // Stop the motors when the target coordinate is reached
  setSpeed(0, 0);
}

void goUntilObject(float range) {
  goForward();
  while (!isObjectDetected(range)) {
    updateLocation();
    if (inGoalZone()) {
      break;
    }
  }
  setSpeed(0, 0);
  updateLocation();
}

bool inTopZone() {
  return otvY > TOP_THRESHOLD_Y;
}

bool inBottomZone() {
  return otvY < BOTTOM_THRESHOLD_Y;
}

bool inGoalZone() {
  return otvX > GOAL_ZONE_X;
}

// Updates global variables for OTV location
void updateLocation() {
  Enes100.updateLocation();
  otvX = Enes100.getX();
  otvY = Enes100.getY();
  otvTheta = Enes100.getTheta();
}





void setSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_BACKWARD, 0);
    analogWrite(RIGHT_FORWARD, rightSpeed);
  } else {
    analogWrite(RIGHT_FORWARD, 0);
    analogWrite(RIGHT_BACKWARD, rightSpeed);
  }

  if (leftSpeed >= 0) {
    analogWrite(LEFT_BACKWARD, 0);
    analogWrite(LEFT_FORWARD, leftSpeed);
  } else {
    analogWrite(LEFT_FORWARD, 0);
    analogWrite(LEFT_BACKWARD, leftSpeed);
  }
}

bool readMagnetism() {
  int sensorValue = analogRead(A5);
  if (sensorValue > 525) {
    return true;
  }
  return false;
}


int readPWM() {
  digitalWrite(LIN_DOWN, HIGH);
  long startingTime = millis();
  while (!matchFound) {
    if (millis() - startingTime > 10000) {
      matchFound = true;
      break;
    }
    int pwmReadValue = pulseIn(PWM_READER, HIGH);  // Measure the length of a HIGH pulse (this is blocking)

    // Check if the PWM value is within the specified range
    if (pwmReadValue >= 500 && pwmReadValue <= 9500) {
      // Calculate nearest thousand
      int nearestThousand = ((pwmReadValue + 500) / 1000) * 1000;

      // Check if the value is within 100 of the nearest thousand
      if (abs(pwmReadValue - nearestThousand) <= 100) {
        // Convert to an integer from 1 to 9
        int roundedValue = nearestThousand / 1000;
        roundedValue = constrain(roundedValue, 1, 9);  // Ensure it's within 1 to 9

        // Store the rounded value in the array
        values[index] = roundedValue;
        index = (index + 1) % 100;  // Move to the next index, wrapping around at 100
      }
    }

    // Check for matches more than 5 times
    for (int i = 1; i <= 9; i++) {

      int count = 0;
      for (int j = 0; j < 100; j++) {
        if (values[j] == i) {
          count++;
        }
      }
      if (count > 4) {
        matchFound = true;
        matchedNumber = i;
        break;
      }
    }
  }

  // This part should be outside of the if (pwmReadValue >= 500 && pwmReadValue <= 9500) condition
  if (matchFound) {
    // Read the analog voltage on pin A2 for magnet detection
    // int sensorValue = analogRead(A0);

    // Determine if a magnet is detected
    // bool magnetDetected = (sensorValue > 525 || sensorValue < 490);

    // Output the PWM signal based on magnet detection
    int pwmOutput = (matchedNumber * 10);
    digitalWrite(LIN_DOWN, LOW);
    return pwmOutput;
    // Wait for 0.1 seconds before the next read
  }
}