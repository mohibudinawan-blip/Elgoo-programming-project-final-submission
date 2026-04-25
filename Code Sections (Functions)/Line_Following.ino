// ====== LINE SENSOR PINS ======
#define LINE_L A2
#define LINE_C A1
#define LINE_R A0

// ====== LINE FOLLOWING CONSTANTS ======
#define WHITE_THRESHOLD 300
#define SPEED_FORWARD 80
#define SPEED_TURN 70

// ====== MOTOR FUNCTION DECLARATIONS ======
// pull from Motors
void driveForward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();


// ================= WHITE LINE FOLLOWING =================

// Returns true if the robot handled a line following action
bool FollowWhiteLine() {

  // Read all three line sensors
  int leftVal = analogRead(LINE_L);
  int centerVal = analogRead(LINE_C);
  int rightVal = analogRead(LINE_R);

  // Convert readings into booleans
  bool leftWhite = (leftVal < WHITE_THRESHOLD);
  bool centerWhite = (centerVal < WHITE_THRESHOLD);
  bool rightWhite = (rightVal < WHITE_THRESHOLD);

  // Case 1: Center sees white, robot is aligned, go straight
  if (centerWhite && !leftWhite && !rightWhite) {
    driveForward(SPEED_FORWARD);
    return true;
  }

  // Case 2: Left sees white, robot drifted right, correct by turning left
  if (leftWhite && !centerWhite) {
    turnLeft(SPEED_TURN);
    return true;
  }

  // Case 3: Right sees white, robot drifted left, correct by turning right
  if (rightWhite && !centerWhite) {
    turnRight(SPEED_TURN);
    return true;
  }

  // Case 4: All sensors see white, robot is fully on the line, go straight
  if (leftWhite && centerWhite && rightWhite) {
    driveForward(SPEED_FORWARD);
    return true;
  }

  // Case 5: No sensors see white, robot lost the line, stop
  stopMotors();
  return true;
}