#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// ====== PIN CONSTANT VALUES ======

#define NUM_LEDS 2            // Number of LEDs on your board
#define PIN_RBGLED 4          // LED Pin
#define PWR_R 5               // Right Motor Power
#define PWR_L 6               // Left Motor Power
#define MTR_R 8               // Right Motor Control
#define MTR_L 7               // Left Motor Control
#define SERVO 10              // Servo Motor
#define MTR_ENABLE 3          // Motor Enable Pin
#define US_OUT 13             // Ultrasonic Trigger
#define US_IN 12              // Ultrasonic Echo
#define LINE_L A2             // Left Line Tracker
#define LINE_C A1             // Center Line Tracker
#define LINE_R A0             // Right Line Tracker
#define BUTTON 2              // Push Button
#define GYRO 0x68             // Gyro Sensor Address
#define TURN_TOLERANCE 5      // allowed angle error

// ====== PROGRAM CONSTANTS ======
#define SPEED_NORMAL 110
#define SPEED_TURN 90
#define WALL_DISTANCE 20      
#define SCAN_LEFT_ANGLE 160
#define SCAN_RIGHT_ANGLE 70
#define SCAN_CENTER_ANGLE 93

// ====== TURN MEMORY STRUCT ======
//data type that stores the robots turns
struct TurnRecord {
  bool isLeft;   // true = left and false = right
};

TurnRecord turnHistory[4];  // store 4 turns
int turnCount = 0;

// ====== PROGRAM VARIABLES ======
int16_t gyroZ;                
float gyroZOffset = 0;        
float currentAngle = 0;       
unsigned long lastTime = 0;   
CRGB leds[NUM_LEDS];          
Servo scanServo;              

bool leftBlocked = false;
bool rightBlocked = false;

void setup() {

  // setup LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255
  
  // Motor pins
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);
  
  // Button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(MTR_ENABLE, HIGH);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  centerServo();   // Center position
  
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {}

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }

  calibrateGyro();
}

void loop() {
  // Keep gyro angle updated
  updateGyroAngle();

  // Stop after 4 turns and print memory
  if (turnCount >= 4) {
    stopMotors();
    printTurnHistory();
    while (true); // freeze robot
  }

  // Get distance ahead
  int front = getDistance();

  // If valid reading and wall is close, avoid it
  if (front > 0 && front <= WALL_DISTANCE) {
    avoidWall();
  } else {
    driveForward(SPEED_NORMAL);
  }
}

// ==== TURN MEMORY ====
void printTurnHistory() {
  Serial.println("Turns made in Maze:");
  for (int i = 0; i < turnCount; i++) {
    Serial.print("Turn ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(turnHistory[i].isLeft ? "Right " : "Left ");
  }
}

// ====== LED FUNCTIONS ======

void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

// ===== SERVO FUNCTIONS =====

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

// ====== GYRO FUNCTIONS ======

bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  byte error = Wire.endTransmission();
  
  if (error != 0) return false;
  
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  lastTime = millis();
  return true;
}

void calibrateGyro() {
  delay(500);
  
  long sum = 0;
  int samples = 100;
  
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
  
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  gyroZ = readGyroZ();
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);
  
  currentAngle += gyroRate * dt;
  
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void resetAngle() {
  currentAngle = 0;
}

float getAngle() {
  return currentAngle;
}

// ===== ULTRASONIC SENSOR =====

int getDistance() {
  int validReading = 0;
  int attempts = 0;
  
  while (validReading == 0 && attempts < 3) {
    if (attempts > 0) delay(60);
    
    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);
    
    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2;
    
    if (duration > 0 && distance <= 200) {
      validReading = distance;
    }
    
    attempts++;
  }
  
  return validReading;
}

// ===== MOTOR FUNCTIONS =====

void driveForward(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
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

// ===== TURN WITH MEMORY =====

void turnByAngle(float targetAngle) {
  resetAngle();
  delay(10);

  bool turningLeft = (targetAngle > 0);

  if (turningLeft) {
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

  // ===== RECORD TURN =====
  //saves the direction into the memory array
  if (turnCount < 4) {
    turnHistory[turnCount].isLeft = turningLeft;
    turnCount++;
  }
}

// ===== WALL AVOIDANCE =====

int scanDirection(int angle) {
  setServoAngle(angle);
  delay(650);
  return getDistance();
}

void avoidWall() {
  stopMotors();
  ledOn(CRGB::Yellow);

  int leftDist = scanDirection(SCAN_LEFT_ANGLE);
  int rightDist = scanDirection(SCAN_RIGHT_ANGLE);

  centerServo();
  delay(650);

  leftBlocked = (leftDist > 0 && leftDist < 25);
  rightBlocked = (rightDist > 0 && rightDist < 25);

  if (leftBlocked && !rightBlocked) {
    turnByAngle(-90);
  }
  else if (rightBlocked && !leftBlocked) {
    turnByAngle(90);
  }
  else {
    if (leftDist > rightDist) {
      turnByAngle(90);
    } else {
      turnByAngle(-90);
    }
  }

  ledOff();
}