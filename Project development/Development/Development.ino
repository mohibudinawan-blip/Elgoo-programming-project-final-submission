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
#define TURN_TOLERANCE 5   // degrees

// ====== PROGRAM CONSTANTS ======
#define SPEED_NORMAL 110
#define SPEED_TURN 90
#define WALL_DISTANCE 20      
#define SCAN_LEFT_ANGLE 160
#define SCAN_RIGHT_ANGLE 70
#define SCAN_CENTER_ANGLE 93


// ====== PROGRAM VARIABLES ======
int16_t gyroZ;                // Raw gyro Z-axis reading
float gyroZOffset = 0;        // Calibration offset
float currentAngle = 0;       // Current angle in degrees
unsigned long lastTime = 0;   // Last read time
CRGB leds[NUM_LEDS];          // Current LED Color values
Servo scanServo;              // Servo

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
  while (digitalRead(BUTTON) == HIGH) {

  }

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

  // Get distance ahead
  int front = getDistance();

  // If valid reading and wall is close, avoid it
  if (front > 0 && front <= WALL_DISTANCE) {
    avoidWall();
  } else {
    // Drive forward normally
    driveForward(SPEED_NORMAL);

    // Apply straight-line correction
    correctHeading();
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

// Set servo angle (0–180 degrees)
void setServoAngle(int angle) {
  static int lastAngle = -1;
  angle = constrain(angle, 0, 180);

  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(15);  // Allow servo to settle
    lastAngle = angle;
  }
}

// Center the servo
void centerServo() {
  setServoAngle(SCAN_CENTER_ANGLE);
}

// ====== GYRO FUNCTIONS ======

// Initialize Gyro Sensor
bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
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
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(10);
  }
  
  gyroZOffset = sum / samples;
  currentAngle = 0;
}

// Read gyro Z-axis
int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);  // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

// MUST be called frequently (e.g., every loop iteration)
// Angle accuracy degrades if this is not called often
void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // Time in seconds
  lastTime = now;
  
  // Read gyro
  gyroZ = readGyroZ();
  
  // Convert to degrees per second (sensitivity = 131 for ±250 deg/s)
  // INVERTED THE SIGN HERE to fix direction!
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);
  
  // Integrate to get angle
  currentAngle += gyroRate * dt;
  
  // Keep angle in range -180 to +180
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

// Reset angle to zero
void resetAngle() {
  currentAngle = 0;
}

// Get current angle
float getAngle() {
  return currentAngle;
}

// ===== STRAIGHT LINE CORRECTION =====
// Uses gyro angle to keep robot driving straight
void correctHeading() {
  float angle = getAngle();

  // Deadzone to prevent wobble
  if (angle > 2) {
    // drifting right → slow right wheel
    analogWrite(PWR_R, SPEED_NORMAL - 20);
    analogWrite(PWR_L, SPEED_NORMAL);
  }
  else if (angle < -2) {
    // drifting left → slow left wheel
    analogWrite(PWR_L, SPEED_NORMAL - 20);
    analogWrite(PWR_R, SPEED_NORMAL);
  }
  else {
    // heading is good → normal speed
    analogWrite(PWR_L, SPEED_NORMAL);
    analogWrite(PWR_R, SPEED_NORMAL);
  }
}

// ===== ULTRASONIC SENSOR FUNCTIONS =====

// Returns distance in centimeters, or 0 if invalid
int getDistance() {
  int validReading = 0;
  int attempts = 0;
  
  while (validReading == 0 && attempts < 3) {
    if (attempts > 0) delay(60);  // Only delay on retries
    
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

// Turn by the relative angle using gyro
void turnByAngle(float targetAngle) {
  // Disable heading correction during turns
analogWrite(PWR_L, SPEED_TURN);
analogWrite(PWR_R, SPEED_TURN);
  resetAngle();
  delay(10);

// Disable correction during turning
analogWrite(PWR_L, SPEED_TURN);
analogWrite(PWR_R, SPEED_TURN);

  if (targetAngle > 0) { // If the angle is positive, turn left
    turnLeftRaw(SPEED_TURN);
    while (abs(getAngle() - targetAngle) > TURN_TOLERANCE) { // keep turning as long as the angle is close enough to the target
      updateGyroAngle();
      delay(5);
    }
  } else { // If the angle is negative, turn right
    turnRightRaw(SPEED_TURN);
    while (abs(getAngle() - targetAngle) > TURN_TOLERANCE) { // tolerance for angle, (turn until it reaches within this angles error)
      updateGyroAngle();
      delay(5);
    }
  }
    stopMotors();
  delay(200);

  // Start next straight segment with a fresh heading reference
  resetAngle();
}

// ===== SCANNING WALL AVOIDANCE =====

int scanDirection(int angle) {
  setServoAngle(angle);
  delay(250);  // servo delay
  return getDistance();
}

void avoidWall() {
  stopMotors();
  ledOn(CRGB::Yellow);

  // Scan left and right
  int leftDist = scanDirection(SCAN_LEFT_ANGLE);
  int rightDist = scanDirection(SCAN_RIGHT_ANGLE);

  // Return servo to center
  centerServo();
  delay(200);
  // Update which side is blocked based on this scan
  leftBlocked = (leftDist < 25);
  rightBlocked = (rightDist < 25);

// ===== MEMORY-BASED TURN DECISION =====

// If left was blocked last time, prefer turning right
if (leftBlocked && !rightBlocked) {
  turnByAngle(-90);   // turn right
}
// If right was blocked last time, prefer turning left
else if (rightBlocked && !leftBlocked) {
  turnByAngle(90);    // turn left
}
// If both sides blocked or both clear, fall back to distance comparison
else {
  if (leftDist > rightDist) {
    turnByAngle(90);   // left
  } else {
    turnByAngle(-90);  // right
  }
}

  ledOff();
}