/*   Martin Porebski
 *   04/10/2017
 *   Arduino Uno, Atmel Mega 328p, au 1722
 *   Arduino UNO control for the car
 */
// use the standard arduino servo library
#include <Servo.h>

// define which pin is used for the motor
// RM = right motor
#define RMF 5       // foward
#define RMB 4       // back
// LM = left motor
#define LMF 6       // foward
#define LMB 7       // back

// declare servo variables
#define ServoPin 8
#define SC 150      // center positon
#define SL 190      // left
#define SR 80       // right
#define SWait 1000  // wait for the turn to complete
Servo servoControl;



int distance = 100;
// ---------------------------------------  Arduino
void setup()
{
  // set the pins connected to the motor to output
  pinMode(RMB, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(LMB, OUTPUT);
  pinMode(LMF, OUTPUT);

  // connect the servo
  servoControl.attach(ServoPin);
  
  // center the main sensor
  servoCenter();
  

}

/*  Program Loop
 *  Detect for obstacles, if detected turn, else keep driving straight   
 */
void loop()
{
  delay(50);
  
  if(readDistance() > 20)
  {
    goStraight();

  }
  else
  {
    goStop();
    servoLeft();
    servoRight();
  }
  
}

// ---------------------------------------  Car control
void goStop()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);  
}

void goStraight()
{
  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}

void goBack()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, HIGH);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, HIGH);
}

void goRight()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, HIGH);
}

void goLeft()
{
  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, HIGH);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);
}

void spinRight()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, HIGH);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}

void spinLeft()
{
  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, HIGH);
}
// to do velocity

// ---------------------------------------  Servo control
void servoCenter()
{
  servoControl.write(SC);
  delay(SWait);
}
void servoLeft()
{
  servoControl.write(SL);
  delay(SWait);
}
void servoRight()
{
  servoControl.write(SR);
  delay(SWait);
}

// ---------------------------------------  Sensor control



