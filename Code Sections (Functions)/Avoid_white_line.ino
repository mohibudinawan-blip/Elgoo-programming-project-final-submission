// ====== LINE SENSOR PINS ======
#define LINE_L A2
#define LINE_C A1
#define LINE_R A0

// ====== WHITE DETECTION CONSTANT ======
#define WHITE_THRESHOLD 300
#define SPEED_FORWARD 80

// ====== MOTOR FUNCTION DECLARATIONS ======
// pull from Motors
void driveForward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void turnToAngle(float angle);


// ================= WHITE LINE HANDLING =================

// Returns true if the robot handled a white line
bool StopWhiteLine() {

  // Read all three line sensors
  int leftVal = analogRead(LINE_L);
  int centerVal = analogRead(LINE_C);
  int rightVal = analogRead(LINE_R);

  // Convert readings into booleans
  bool leftWhite = (leftVal < WHITE_THRESHOLD);
  bool centerWhite = (centerVal < WHITE_THRESHOLD);
  bool rightWhite = (rightVal < WHITE_THRESHOLD);

  // Case 1: No white detected, safe to move forward
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

  // Case 4: Center sees white or multiple sensors see white, stop turn 90 degrees
  stopMotors();
  delay(200);
  turnToAngle(90);
  return true;
}
