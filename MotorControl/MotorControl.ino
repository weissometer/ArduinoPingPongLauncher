
#include <ir_sensing_arduino_code.h>

#include <"ir_sensing_code.h">

#include <Servo.h>

/*
partly from
Adafruit Arduino - Lesson 15. Bi-directional Motor


*/
/* 
ir_sensing_arduino_code
adapted from "IR commander" code at adafruit.com
*/

// We need to use the 'raw' Atmega168 pin reading methods as digitalRead() is too slow!
#define IRpin_PIN PIND  // Port D Input Register, http://www.arduino.cc/en/Reference/PortManipulation
#define IRpin 2 // Digital pin #2 is Pin D2 on Atmega168,  http://arduino.cc/en/Hacking/PinMapping168 

#define MAXPULSE 65000   // the maximum pulse lungth we'll listen for (65 ms)
#define NUMPULSES 50

#define RESOLUTION 20  // timing resolution: larger is more precise
#define FUZZINESS 40   // allowed % variation in timing signals

uint16_t pulses[NUMPULSES][2];  // pulse pair is high and low pulse (unsigned 16 bit integer)
uint8_t currentpulse = 0; // index for pulses we're storing (unsigned 8 bit integer)

int rEnable=3; 
int rMotor3a=12;
int rMotor4a=4;
int lEnable=5;
int lMotor2a=6;
int lMotor1a=7;

int potPin=A0;

int switchPin = 13;

Servo vertServo;
Servo lServo;
Servo rServo;

int vertpos=0;
int lpos=0; //left servo position
int rpos=0;
int val ;// for knob sweep

void setup()
{
  Serial.begin(9600);
  Serial.println("Ready to decode IR!");
  
  pinMode(rEnable, OUTPUT);
  pinMode(rMotor3a, OUTPUT);
  pinMode(rMotor4a, OUTPUT);
  pinMode(lEnable, OUTPUT);
  pinMode(lMotor2a, OUTPUT);
  pinMode(lMotor1a, OUTPUT);
  
  pinMode(switchPin, INPUT_PULLUP);
  
  vertServo.attach(9);
  lServo.attach(10);
  rServo.attach(11);
  
}

void loop()
{
  // Motor control
  int speed = analogRead(potPin) / 4;
  boolean reverse = digitalRead(switchPin);
  setMotor(speed, reverse);
  
   val = analogRead(potPin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 
  vertServo.write(val);                  // sets the servo position according to the scaled value 
  lServo.write(val);  
  rServo.write(val); 
  delay(15);                           // waits for the servo to get there 
}

void setMotor(int speed, boolean reverse)
{
  analogWrite(rEnable, speed);
  analogWrite(lEnable,speed);
  digitalWrite(rMotor3a, ! reverse);
  digitalWrite(rMotor4a, reverse);
   digitalWrite(lMotor1a, ! reverse);
  digitalWrite(lMotor2a, reverse);
}

// IR shenanigans

void checkIR(void){
   int numberpulses = listenForIR();
  if (IRcompare(numberpulses, leftButton, sizeof(leftButton) / 4)) {
    Serial.println("LEFT");
  }
  else if (IRcompare(numberpulses, rightButton, sizeof(rightButton) / 4)) {
    Serial.println("RIGHT");
  }
  else if (IRcompare(numberpulses, forwardSignal, sizeof(forwardSignal) / 4)) {
    Serial.println("FORWARD");
  }
  else {
    Serial.println("IR signal detected, but no match found (");
    Serial.print(numberpulses);
    Serial.println("pulses long");
  }
  delay(20);
}

boolean IRcompare(int numpulses, int Signal[], int refsize) {
  int count = min(numpulses, refsize);  // Only compare the minimum of the two
  for (int i = 0; i < count - 1; i++) {
    int oncode = pulses[i][1] * RESOLUTION / 10;
    int offcode = pulses[i + 1][0] * RESOLUTION / 10;
    int onsignal = Signal[i * 2];
    int offsignal = Signal[i * 2 + 1];
    int onslop = int((float)onsignal * FUZZINESS / 100);
    int offslop = int((float)offsignal * FUZZINESS / 100);

    // check if on & off signal error > slop
    if ( abs(oncode - onsignal) > onslop) {
      return false;  // we didn't match
    }
    if ( abs(offcode - offsignal) > offslop) {
      return false;  // we didn't match
    }
  }
  return true;  // Everything matched!
}

int listenForIR(void) {
  currentpulse = 0;
  while (1) {
    uint16_t highpulse, lowpulse;  // temporary storage timing
    highpulse = lowpulse = 0; // start out with no pulse length

    while (IRpin_PIN & (1 << IRpin)) {   // pin is HIGH
      // Note: "PIND & (1 << IRpin)" is true iff "IRpin" pin of PIND is high and all other pins are low.
      // In other words, (1 << IRpin) serves as a "bitmask" to evaluate input pin register
      highpulse++;
      delayMicroseconds(RESOLUTION);   // count off another few microseconds
      
      // Check for end of receive buffer (including if pulse is too long):
      if (((highpulse >= MAXPULSE) && (currentpulse != 0)) || currentpulse == NUMPULSES) {
        // printpulses();   // uncomment to show pulse details (1/2)
        return currentpulse;
      }
    }
    // we didn't time out so lets stash the reading
    pulses[currentpulse][0] = highpulse;

    // same as above, but now looking at a low signal
    while (! (IRpin_PIN & _BV(IRpin))) {      // pin is LOW
      // Note: _BV() = Bit Value, a compiler macro defined as #define _BV( bit ) ( 1<<(bit) ).
      lowpulse++;
      delayMicroseconds(RESOLUTION);
      // Check again for end of receive buffer (since we can end on a high or low):
      if (((lowpulse >= MAXPULSE)  && (currentpulse != 0)) || currentpulse == NUMPULSES) {
        // printpulses();  // uncomment to show pulse details (2/2)
        return currentpulse;
      }
    }
    pulses[currentpulse][1] = lowpulse;
    currentpulse++;    // we read one high-low pulse successfully, continue!
  }
}
