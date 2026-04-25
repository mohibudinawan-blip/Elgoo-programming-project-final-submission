#define Pin_Motor_PWMA 5
#define Pin_Motor_PWMB 6
#define Pin_Motor_AIN1 7
#define Pin_Motor_BIN1 8
#define Pin_Motor_STBY 3

void setup() {
  // put your setup code here, to run once:
pinMode(Pin_Motor_PWMA,OUTPUT);
pinMode(Pin_Motor_PWMB,OUTPUT);
pinMode(Pin_Motor_AIN1,OUTPUT);
pinMode(Pin_Motor_BIN1,OUTPUT);
pinMode(Pin_Motor_STBY,OUTPUT);


digitalWrite(Pin_Motor_STBY,HIGH);
}
  void loop() {
  // put your main code here, to run repeatedly:
 delay(2000);
analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);
digitalWrite(Pin_Motor_AIN1,HIGH);
digitalWrite(Pin_Motor_BIN1,HIGH);


delay(2000);
analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);


delay(2000);
analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);
digitalWrite(Pin_Motor_AIN1,HIGH);
digitalWrite(Pin_Motor_BIN1,HIGH);
delay(2000);


analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);
digitalWrite(Pin_Motor_AIN1,LOW);
digitalWrite(Pin_Motor_BIN1,LOW);


delay(2000);
analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);


delay(2000);
analogWrite(Pin_Motor_PWMA,0);
analogWrite(Pin_Motor_PWMB,0);
digitalWrite(Pin_Motor_AIN1,HIGH);
digitalWrite(Pin_Motor_BIN1,HIGH);
delay(2000);

} 

