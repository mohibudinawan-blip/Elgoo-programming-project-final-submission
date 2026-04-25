//implementation is based on code by Eric Rapos
//source:https://rpglab.pages.dev/1063/RobotBase1063.ino
//Modifications by Mohsen Mirza and Mohib Awan: simplified servo and ultra sonic functions, added motor and light level detection (deters robot from white), adjusted speed and threshold values.

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
#define LINE_L A2
#define LINE_C A1
#define LINE_R A0
#define BUTTON 2
#define GYRO 0x68

// ====== PROGRAM CONSTANTS ======
#define SPEED_FORWARD 80
#define SPEED_TURN 70
#define OBSTACLE_DISTANCE 8
#define WHITE_THRESHOLD 300

// ====== PROGRAM VARIABLES ======
int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;
CRGB leds[NUM_LEDS];
Servo scanServo;

void setup() {
  // setup LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  FastLED.clear();
  FastLED.show();

  // Motor pins
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // Line tracking sensor pins
  pinMode(LINE_L, INPUT);
  pinMode(LINE_C, INPUT);
  pinMode(LINE_R, INPUT);

  // Button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(MTR_ENABLE, HIGH);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  setServoAngle(90); // 90 degree position

  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {
    stopMotors();
  }

  delay(300);

  setupGyro();
  calibrateGyro();
}

// ================= MOTOR FUNCTIONS =================

void driveForward(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

void turnRight(int speed) {
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

void turnLeft(int speed) {
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}

// ================= WHITE LINE FUNTIONALITY =================
bool handleWhiteLine() {

  // Read all three line sensors
  int leftVal = analogRead(LINE_L);
  int centerVal = analogRead(LINE_C);
  int rightVal = analogRead(LINE_R);

  // Convert readings to true/false for white detection
  bool leftWhite = (leftVal < WHITE_THRESHOLD);
  bool centerWhite = (centerVal < WHITE_THRESHOLD);
  bool rightWhite = (rightVal < WHITE_THRESHOLD);

  // Case 1: No white detected, move forward
  if (!leftWhite && !centerWhite && !rightWhite) {
    driveForward(SPEED_FORWARD);
    return true;
  }

  // Case 2: White on the right side, turn away from it
  if (rightWhite && !leftWhite) {
    turnRight(70);
    return true;
  }

  // Case 3: White on the left side, turn away from it
  if (leftWhite && !rightWhite) {
    turnLeft(70);
    return true;
  }

  // Case 4: Center sees white or multiple sensors see white, stop and turn
  stopMotors();
  delay(200);
  turnToAngle(90);
  return true;
}

//Main loop
void loop() {

  updateGyroAngle();

  // White line detection
  if (StopWhiteLine()) {
    return;
  }

  int d = getDistance();

  if (d != 0 && d <= OBSTACLE_DISTANCE) {
    stopMotors();
    while (1) {
      // stop at wall
    }
  }

  driveForward(SPEED_FORWARD);
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
  angle = constrain(angle, 0, 180);
  scanServo.write(angle);
  delay(200);
}

// ====== GYRO FUNCTIONS ======

bool setupGyro() { //initialize gyro
  Wire.begin();

  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission();

  // Configure gyro sensitivity (±250 deg/s)
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s
  Wire.endTransmission();

  lastTime = millis();
  return true;
}
// Calibrate gyro (robot must be stationary!)
void calibrateGyro() {
  delay(500);

  long sum = 0;
  
  //take 200 samples and add them
  for (int i = 0; i < 200; i++) { 
    sum += readGyroZ();
    delay(5);
  }

  //average value becomes the baseline
  gyroZOffset = sum / 200;
  currentAngle = 0; //reset angle tracking
}
// Read gyro Z-axis
int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);  // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return Wire.read() << 8 | Wire.read();
}

// MUST be called frequently (e.g., every loop iteration)
// Angle accuracy degrades if this is not called often
void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;   // Time in seconds
  lastTime = now;
  // Read gyro
  gyroZ = readGyroZ();
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);
  currentAngle += gyroRate * dt;
}

//turns the robot until it reaches target angle
void turnToAngle(float targetAngle) {

  currentAngle = 0; //reset angle before turning
  delay(10);

  if (targetAngle > 0) {  //left
    turnLeft(SPEED_TURN);
    while (currentAngle < targetAngle) {
      updateGyroAngle();
    }
  } else { //right
    turnRight(SPEED_TURN);
    while (currentAngle > targetAngle) {
      updateGyroAngle();
    }
  }

  stopMotors(); //stop when angle is reached
  delay(200);
}

// ===== ULTRASONIC SENSOR FUNCTIONS =====

int getDistance() {

  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);

  long duration = pulseIn(US_IN, HIGH, 30000);
  //convert time to distance
  return duration * 0.034 / 2;
}

