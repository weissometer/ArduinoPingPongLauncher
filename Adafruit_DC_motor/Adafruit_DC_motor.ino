/*
Adafruit Arduino - Lesson 15. Bi-directional Motor
*/

int renable=2; 
int rMotor5a=3;
int rMotor4a=4;
int lenable=5;
int lMotor2a=6;
int lMotor1a=7;

int vertServo=9;
int lServo=10;
int rServo=11;

int potPin=A0;
int lServoPot=A2;
int rServoPot=A3


//int enablePin1 = 11;
//int in1Pin = 10;
//int in2Pin = 9;
//int switchPin = 7;
//int potPin = 0;

void setup()
{
  pinMode(in1Pin, OUTPUT);// chip2
  pinMode(in2Pin, OUTPUT);//chip7
  pinMode(enablePin1, OUTPUT);//chip1
  pinMode(switchPin, INPUT_PULLUP);
}

void loop()
{
  int speed = analogRead(potPin) / 4;
  boolean reverse = digitalRead(switchPin);
  setMotor(speed, reverse);
}

void setMotor(int speed, boolean reverse)
{
  analogWrite(enablePin1, speed);
  digitalWrite(in1Pin, ! reverse);
  digitalWrite(in2Pin, reverse);
}
