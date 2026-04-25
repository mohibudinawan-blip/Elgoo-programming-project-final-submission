/* 
============================================================
   Autonomous Robot
   Code by Mohsen Mirza and Mohib Awan
   April 24 2026
   Project Overview:
     - Detects and makes decisions on what direction it's turns are based on physical obstructions, (will make have a 50% chance of picking left or right if both directions are clear)
     - Switches to line mode on white detection
     - Counts black lines and performs dance on second one
     - Includes tape wall logic
============================================================
*/
#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// ====== PIN CONSTANT VALUES ======
// Motor, LED, sensor, and servo pin assignments
#define NUM_LEDS 2
#define PIN_RBGLED 4
#define PWR_R 5
#define PWR_L 6
#define MTR_R 8
#define MTR_L 7
#define SERVO 10
#define MTR_ENABLE 3
#define US_OUT 13
#define US_IN 12
#define LINE_L A2
#define LINE_C A1
#define LINE_R A0
#define BUTTON 2
#define GYRO 0x68
#define TURN_TOLERANCE 5   // gyro turning accuracy

// ====== PROGRAM CONSTANTS ======
// Speeds, distances, and servo scan angles
#define SPEED_NORMAL 55
#define SPEED_TURN 77
#define WALL_DISTANCE 3
#define SCAN_LEFT_ANGLE 0
#define SCAN_RIGHT_ANGLE 200
#define SCAN_CENTER_ANGLE 93
#define SPEED_BALANCE_RIGHT -1   // right motor trim

// ====== MODE SWITCHING ======
enum Mode { WALL_MODE, LINE_MODE };
Mode currentMode = WALL_MODE;    // robot starts in wall mode

#define WHITE_THRESHOLD 160       // white floor detection
#define BLACK_THRESHOLD 650       // black tape detection

// ====== PROGRAM VARIABLES ======
int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;
CRGB leds[NUM_LEDS];
Servo scanServo;

bool leftBlocked = false;
bool rightBlocked = false;

// ====== DANCE VARIABLES ======
int blackCount = 0;       // counts black lines crossed
bool danceDone = false;   // ensures dance only runs once

// ====== TAPE WALL LOGIC FLAGS ======
bool hasSeenWhite = false;            // robot saw white floor
bool hasSeenBlackAfterWhite = false;  // robot saw black after white
bool tapeWallUsed = false;            // prevents repeated tape-wall triggers

void turnByAngle(float targetAngle);
void dance();   // dance routine

// =========================================================
// SETUP
// =========================================================

void setup() {

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

  // motor and sensor pin setup
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);

  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  pinMode(LINE_L, INPUT);
  pinMode(LINE_C, INPUT);
  pinMode(LINE_R, INPUT);

  digitalWrite(MTR_ENABLE, HIGH); // enable motor driver

  Serial.begin(9600);

  scanServo.attach(SERVO);
  centerServo(); // center ultrasonic servo

  randomSeed(analogRead(A3)); // 50/50 for wall turns

  // wait for button press to start
  while (digitalRead(BUTTON) == HIGH);
  delay(500);

  // initialize gyro
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);
  }

  calibrateGyro(); // calibrate gyro drift
}

// =========================================================
// MODE SWITCHING AND MAIN LOOP
// =========================================================

void loop() {

  // read line sensors
  int L = analogRead(LINE_L);
  int C = analogRead(LINE_C);
  int R = analogRead(LINE_R);

  // debug output (for tuning).
  Serial.print("L: "); Serial.print(L);
  Serial.print("   C: "); Serial.print(C);
  Serial.print("   R: "); Serial.print(R);
  Serial.print("   MODE: ");
  Serial.println(currentMode == WALL_MODE ? "WALL" : "LINE");

  // enter line mode when all sensors see white
  if (L < WHITE_THRESHOLD && C < WHITE_THRESHOLD && R < WHITE_THRESHOLD) {
    currentMode = LINE_MODE;
    hasSeenWhite = true;
  }

  // return to wall mode when all sensors see black
  if (L > BLACK_THRESHOLD && C > BLACK_THRESHOLD && R > BLACK_THRESHOLD) {
    currentMode = WALL_MODE;
  }

  // run line following logic
  if (currentMode == LINE_MODE) {
    lineFollow();
    return;
  }

  // ===== WALL MODE =====
  updateGyroAngle();
  int front = getDistance(); // ultrasonic distance

  // tape wall detection (special case)
  if (hasSeenWhite && hasSeenBlackAfterWhite && !tapeWallUsed) {
    if ((L >= 680 && L <= 750) ||
        (C >= 680 && C <= 750) ||
        (R >= 680 && R <= 750)) {

        tapeWallUsed = true;
        avoidWall();
        return;
    }
  }

  // wall avoidance
  if (front > 0 && front <= WALL_DISTANCE) {
      avoidWall();
  } else {
      ledOn(CRGB::Green);
      driveForward(SPEED_NORMAL);
  }
}

// =========================================================
// LINE FOLLOWING MODE 
// =========================================================

void lineFollow() {

  int leftV   = analogRead(LINE_L);
  int middleV = analogRead(LINE_C);
  int rightV  = analogRead(LINE_R);

  // detect full black line
  if (leftV > BLACK_THRESHOLD && middleV > BLACK_THRESHOLD && rightV > BLACK_THRESHOLD) {

    blackCount++;                 // count black line
    hasSeenBlackAfterWhite = true;
    currentMode = WALL_MODE;      // exit line mode
    stopMotors();
    delay(150);

    // second black → dance
    if (blackCount >= 2 && !danceDone) {
      dance();
      danceDone = true;
      while (true);               // freeze robot after dance
    }

    return;
  }

  // line following logic
  if (middleV < 300 && leftV < 300 && rightV < 300) {
    driveForward(45);             // straight
  }
  else if (leftV > 300 && middleV < 300 && rightV < 300) {
    turnLeftRaw(45);              // left correction
  }
  else if (rightV > 300 && middleV < 300 && leftV < 300) {
    turnRightRaw(45);             // right correction
  }
  else if (leftV > 300 && middleV > 300 && rightV > 300) {
    stopMotors();                 // all sensors on line
  }
}

// =========================================================
// LED FUNCTIONS
// =========================================================

void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

// =========================================================
// SERVO FUNCTIONS
// =========================================================

void setServoAngle(int angle) {
  static int lastAngle = -1;
  angle = constrain(angle, 0, 180);
  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(15);
    lastAngle = angle;
  }
}

void centerServo() {
  setServoAngle(SCAN_CENTER_ANGLE);
}

// =========================================================
// GYRO FUNCTIONS 
// =========================================================

bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);   
  Wire.write(0);     
  byte error = Wire.endTransmission();
  if (error != 0) return false;

  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);   // gyro config
  Wire.write(0x00);   
  Wire.endTransmission();

  lastTime = millis();
  return true;
}

void calibrateGyro() {
  delay(500);
  long sum = 0;
  int samples = 100;

  // average gyro drift
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);

    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(10);
  }

  gyroZOffset = sum / samples;
  currentAngle = 0;
}

int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return (Wire.read() << 8 | Wire.read());
}

void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  gyroZ = readGyroZ();
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0); // convert to degrees
  currentAngle += gyroRate * dt;                     // integrate angle

  // wrap angle to -180..180
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void resetAngle() {
  currentAngle = 0;
}

float getAngle() {
  return currentAngle;
}

// =========================================================
// ULTRASONIC FUNCTIONS 
// =========================================================

int getDistance() {
  int validReading = 0;
  int attempts = 0;

  while (validReading == 0 && attempts < 3) {

    if (attempts > 0) delay(60);

    // send ultrasonic pulse
    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);

    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2; // convert to cm

    if (duration > 0 && distance <= 200) {
      validReading = distance;
    }

    attempts++;
  }

  return validReading;
}

// =========================================================
// MOTOR FUNCTIONS
// =========================================================

void driveForward(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);

  int leftSpeed  = speed;
  int rightSpeed = speed + SPEED_BALANCE_RIGHT; // trim

  analogWrite(PWR_L, leftSpeed);
  analogWrite(PWR_R, rightSpeed);
}

void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}

void turnLeftRaw(int speed) {
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

void turnRightRaw(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

// =========================================================
// WALL AVOIDANCE
// =========================================================

int scanDirection(int angle) {
  setServoAngle(angle); // rotate ultrasonic sensor
  delay(250);
  return getDistance();
}

void avoidWall() {
  stopMotors();
  ledOn(CRGB::Red);

  // scan left and right distances
  int leftDist = scanDirection(SCAN_LEFT_ANGLE);
  int rightDist = scanDirection(SCAN_RIGHT_ANGLE);

  centerServo();
  delay(200);

  leftBlocked = (leftDist < 25);
  rightBlocked = (rightDist < 25);

  // choose turn direction
  if (leftBlocked && !rightBlocked) {
    turnByAngle(-90);
  }
  else if (rightBlocked && !leftBlocked) {
    turnByAngle(90);
  }
  else {
    // random turn when both sides are open
    if (random(0, 2) == 0) {
        turnByAngle(90);
    } else {
        turnByAngle(-90);
    }
  }
}

// =========================================================
// TURN BY ANGLE
// =========================================================

void turnByAngle(float targetAngle) {
  ledOn(CRGB::Blue);
  resetAngle();
  delay(10);

  // positive = CCW, negative = CW
  if (targetAngle > 0) {
    turnLeftRaw(SPEED_TURN);
    while (abs(getAngle() - targetAngle) > TURN_TOLERANCE) {
      updateGyroAngle();
      delay(5);
    }
  } else {
    turnRightRaw(SPEED_TURN);
    while (abs(getAngle() - targetAngle) > TURN_TOLERANCE) {
      updateGyroAngle();
      delay(5);
    }
  }

  stopMotors();
  delay(200);
}

// =========================================================
// DANCE FUNCTION
// =========================================================

void dance() {

  // forward 
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, 60);
  analogWrite(PWR_R, 60);
  delay(2000);
  
  ledOn(CRGB::Red);

  // CCW spin
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_L, 255);
  analogWrite(PWR_R, 255);
  delay(1000);
  
  ledOn(CRGB::Purple);

  // CW spin
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, 255);
  analogWrite(PWR_R, 255);
  delay(1000);

  // back
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_L, 60);
  analogWrite(PWR_R, 60);
  delay(500);

  // forward
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, 60);
  analogWrite(PWR_R, 60);
  delay(500);

  // CW spin 
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_L, 155);
  analogWrite(PWR_R, 155);
  delay(1000);

  // CW spin 
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, 255);
  analogWrite(PWR_R, 255);
  delay(1000);

  // stop
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}
