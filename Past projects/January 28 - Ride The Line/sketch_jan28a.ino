// Serial PIN Tracking 
#define LEFT A2
#define MIDDLE A1
#define RIGHT A0

// Motor pin tracking
#define Pin_Motor_PWMA 5
#define Pin_Motor_PWMB 6
#define Pin_Motor_AIN1 7
#define Pin_Motor_BIN1 8
#define Pin_Motor_STBY 3

void setup() {
pinMode(LEFT,INPUT);
pinMode(MIDDLE,INPUT);
pinMode(RIGHT,INPUT);

// motors
pinMode(Pin_Motor_PWMA,OUTPUT);
pinMode(Pin_Motor_PWMB,OUTPUT);
pinMode(Pin_Motor_AIN1,OUTPUT);
pinMode(Pin_Motor_BIN1,OUTPUT);
pinMode(Pin_Motor_STBY,OUTPUT);
digitalWrite(Pin_Motor_STBY, HIGH);

Serial.begin(9600);
}

void loop() {

int leftV   = analogRead(LEFT);
int middleV = analogRead(MIDDLE);
int rightV  = analogRead(RIGHT);

Serial.print(leftV);
Serial.print(" ");
Serial.print(middleV);
Serial.print(" ");
Serial.println(rightV);
delay(200);
// ===== MOVEMENT =====

// go straight (middle on black line)
if (middleV < 600 && leftV < 600 && rightV < 600)
{
  analogWrite(Pin_Motor_PWMA, 90);
  analogWrite(Pin_Motor_PWMB, 90);
  digitalWrite(Pin_Motor_AIN1, HIGH);
  digitalWrite(Pin_Motor_BIN1, HIGH);
}

// turn left (left sensor on line)
else if (leftV > 600 && middleV < 600 && rightV < 600)
{
  analogWrite(Pin_Motor_PWMA, 0);
  analogWrite(Pin_Motor_PWMB, 70);
  digitalWrite(Pin_Motor_AIN1, HIGH);
  digitalWrite(Pin_Motor_BIN1, HIGH);
}

// turn right (right sensor on line)
else if (rightV > 600 && middleV < 600 && leftV < 600)
{
  analogWrite(Pin_Motor_PWMA, 70);
  analogWrite(Pin_Motor_PWMB, 0);
  digitalWrite(Pin_Motor_AIN1, HIGH);
  digitalWrite(Pin_Motor_BIN1, HIGH);
}

// stop (all sensors on black)
else if (leftV > 600 && middleV > 600 && rightV > 600)
{
  analogWrite(Pin_Motor_PWMA, 0);
  analogWrite(Pin_Motor_PWMB, 0);
  digitalWrite(Pin_Motor_AIN1, HIGH);
  digitalWrite(Pin_Motor_BIN1, HIGH);
}

}
