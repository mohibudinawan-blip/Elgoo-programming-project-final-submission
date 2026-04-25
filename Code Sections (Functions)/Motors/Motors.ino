// ====== MOTOR PIN CONSTANTS ======
// Power pins control motor speed (PWM)
#define PWR_R 5
#define PWR_L 6

// Direction pins control motor rotation direction
#define MTR_R 8
#define MTR_L 7

// Enable pin activates the motor driver
#define MTR_ENABLE 3


// ================= MOTOR FUNCTIONS =================

// Drive forward at a given speed (0–255)
void driveForward(int speed) {
  digitalWrite(MTR_L, HIGH);   // Left motor forward
  digitalWrite(MTR_R, HIGH);   // Right motor forward
  analogWrite(PWR_L, speed);   // Set left motor speed
  analogWrite(PWR_R, speed);   // Set right motor speed
}

// Drive backward at a given speed (0–255)
void driveBackward(int speed) {
  digitalWrite(MTR_L, LOW);    // Left motor reverse
  digitalWrite(MTR_R, LOW);    // Right motor reverse
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

// Turn right by running motors in opposite directions
void turnRight(int speed) {
  digitalWrite(MTR_L, HIGH);   // Left motor forward
  digitalWrite(MTR_R, LOW);    // Right motor backward
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

// Turn left by running motors in opposite directions
void turnLeft(int speed) {
  digitalWrite(MTR_L, LOW);    // Left motor backward
  digitalWrite(MTR_R, HIGH);   // Right motor forward
  analogWrite(PWR_L, speed);
  analogWrite(PWR_R, speed);
}

// Immediately stop both motors
void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}
