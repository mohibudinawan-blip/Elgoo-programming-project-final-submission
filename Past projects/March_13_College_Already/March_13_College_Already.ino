#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// ====== PIN CONSTANT VALUES ======

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
#define BUTTON 2
#define GYRO 0x68
#define TURN_TOLERANCE 5

// ====== PROGRAM CONSTANTS ======
#define SPEED_NORMAL 90
#define SPEED_TURN 93
#define FOLLOW_DISTANCE 30
#define SCAN_LEFT_ANGLE 160
#define SCAN_RIGHT_ANGLE 70
#define SCAN_CENTER_ANGLE 93
#define SPEED_BALANCE_RIGHT -2

// ====== PROGRAM VARIABLES ======
int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;
CRGB leds[NUM_LEDS];
Servo scanServo;

// =========================================================
// SETUP
// =========================================================

void setup() {

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);
  
  pinMode(BUTTON, INPUT_PULLUP);

  digitalWrite(MTR_ENABLE, HIGH);

  Serial.begin(9600);

  scanServo.attach(SERVO);
  centerServo();
  
  while (digitalRead(BUTTON) == HIGH) {}

  delay(500);

  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);
  }

  calibrateGyro();
}

// =========================================================
// MAIN LOOP
// =========================================================

void loop() {
  updateGyroAngle();
  int front = getDistance();

  // ===== OBSTACLE FOLLOWING MODE =====
  if (front > 0 && front < FOLLOW_DISTANCE) {
    followObstacleLeft();
    return;
  }

  // ===== NORMAL DRIVING =====
  ledOn(CRGB::Green);
  driveForward(SPEED_NORMAL);
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

// =========================================================
// ULTRASONIC SENSOR
// =========================================================

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

// =========================================================
// MOTOR FUNCTIONS
// =========================================================

void driveForward(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);

  int leftSpeed  = speed;
  int rightSpeed = speed + SPEED_BALANCE_RIGHT;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(PWR_L, leftSpeed);
  analogWrite(PWR_R, rightSpeed);
}

void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}

// =========================================================
// OBSTACLE FOLLOWING MODE (FIXED)
// =========================================================

void followObstacleLeft() {
  ledOn(CRGB::Blue);

  setServoAngle(SCAN_LEFT_ANGLE);
  delay(120);

  while (true) {
    int front = getDistance();
    int side  = scanDirection(SCAN_LEFT_ANGLE);

    // EXIT IMMEDIATELY WHEN OBSTACLE DISAPPEARS
    if (front > FOLLOW_DISTANCE || front == 0) {
      stopMotors();
      centerServo();
      delay(100);
      return;
    }

    // FOLLOWING LOGIC
    if (side < 15) {
      analogWrite(PWR_L, SPEED_NORMAL);
      analogWrite(PWR_R, SPEED_NORMAL - 35);
    }
    else if (side > 25) {
      analogWrite(PWR_L, SPEED_NORMAL - 35);
      analogWrite(PWR_R, SPEED_NORMAL);
    }
    else {
      driveForward(SPEED_NORMAL);
    }

    delay(30);
  }
}

// =========================================================
// SCAN FUNCTION
// =========================================================

int scanDirection(int angle) {
  setServoAngle(angle);
  delay(200);
  return getDistance();
}



