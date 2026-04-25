#include <Wire.h>

// ====== GYRO VARIABLES ======
int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;

#define GYRO 0x68   // MPU6050 I2C address

// ====== ULTRASONIC PINS ======
#define US_OUT 13
#define US_IN 12

// ====== DISTANCE CONSTANTS ======
#define OBSTACLE_DISTANCE 8   // cm
#define SPEED_TURN 70         // turning speed

// ====== MOTOR FUNCTION DECLARATIONS ======
// take from Motors
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void driveForward(int speed);


// ================= GYRO FUNCTIONS =================

// Initialize the gyro sensor
bool setupGyro() {
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // Set gyro sensitivity to ±250 deg/s
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  lastTime = millis();
  return true;
}

// Calibrate gyro
void calibrateGyro() {
  delay(500);
  long sum = 0;

  for (int i = 0; i < 200; i++) {
    sum += readGyroZ();
    delay(5);
  }

  gyroZOffset = sum / 200;
  currentAngle = 0;
}

// Read raw Z-axis rotation
int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);  // ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);

  return Wire.read() << 8 | Wire.read();
}

// Update angle using gyro integration
void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  gyroZ = readGyroZ();
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);

  currentAngle += gyroRate * dt;
}

// Turn robot until it reaches a target angle
void turnToAngle(float targetAngle) {
  currentAngle = 0;
  delay(10);

  if (targetAngle > 0) {
    turnLeft(SPEED_TURN);
    while (currentAngle < targetAngle) {
      updateGyroAngle();
    }
  } else {
    turnRight(SPEED_TURN);
    while (currentAngle > targetAngle) {
      updateGyroAngle();
    }
  }

  stopMotors();
  delay(200);
}


// ================= ULTRASONIC WALL DETECTION =================

// Return distance in cm
int getDistance() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);

  long duration = pulseIn(US_IN, HIGH, 30000);
  return duration * 0.034 / 2;
}

// Check for wall and stop robot if too close
bool checkWall() {
  int d = getDistance();

  if (d != 0 && d <= OBSTACLE_DISTANCE) {
    stopMotors();
    return true;
  }

  return false;
}