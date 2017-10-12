/*   Martin Porebski
 *   12/10/2017
 *   Arduino Uno, Atmel Mega 328p, au 1722
 *   Arduino UNO control for the car
 */
// use the standard arduino servo library
#include <Servo.h>

// Define which pin is used for the motor
// RM = right motor
#define RMF 5       // foward
#define RMB 4       // back
// LM = left motor
#define LMF 6       // foward
#define LMB 7       // back

// Declare servo variables
#define ServoPin 8
#define SC 82       // center positon
#define SL 180      // left
#define SR 0        // right
#define SWait 1000  // wait for the turn to complete
Servo servoControl;

// Sonic sensor variables
// Define sensor pins
#define TriggerPin A0                 // used to communicate to sensor to start a ping (send out a sound signal)
#define EchoPin A1                    // used to recieve the information from the sensor about the sound reflection (echo)
// Sensor's physical constraints, us = microseconds
#define MaxSensorDist 500             // how far can the sensor detect in cm
#define MaxSensorDelay 5800           // max time in us the sensor should take to send out a signal
#define USTime1CM 57                  // time us it takes the soundwave to travel 1cm the and back (2cm total)
#define PingOverhead 5                // overhead us
// Sensor constants
unsigned int NoEcho = MaxSensorDist;  // what to return when there is no reflection (over max detection dist)
unsigned int MaxDetectDistCM = 200;   // detect up to 200cm, no need for 500cm, saves time
// Function variables
unsigned int MaxEchoDistTime;
unsigned long EchoTimeout;

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
  
  // setup the sensors
  sensorSetup();
}

/*  Program Loop
 *  Detect for obstacles, if detected turn, else keep driving straight   
 */


int maxWall = 40;
void loop()
{
  
  servoCenter();
  if(readDistance() > maxWall)
  {
    servoLeft();
    if(readDistance() > maxWall)
    {
      servoRight();
      if(readDistance() > maxWall)
      {
        goStraight();
      }
      else
      {
        carTurn(SR);
      }
    }
    else
    {
        carTurn(SL);
    }
  }
  else
  {
    carTurn(SL);
  }
  /*
  // occasianly turn left or tight (wherever the wall is)
  delay(50);
  goStraight();
  if(readDistance() > 20)
  {
    goStraight();
  }
  else
  {
    goStop();

    servoLeft();
    
    if(readDistance() > 20)
    {
      servoCenter();
      while(readDistance() <= 20)
      {
        goLeft();
      }      
    }
    else
    {
      servoCenter();
      while(readDistance() <= 20)
      {
        goRight();
      }
    }
  }*/
}

void carTurn(int dir)
{
  int centerDist = 0;
  int sideDist = 0;
  while((centerDist < maxWall) && (sideDist > (maxWall+5) || sideDist < (maxWall-5)))
  {
    if(dir == SR)
    {
      goLeft();
    }
    else
    {
      goRight();
    }
    delay(100);
    goStop();
    
    servoCenter();
    centerDist = readDistance();
    servoTurn(dir);
    sideDist = readDistance();
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

void goLeft()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}

void goRight()
{
  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);
}

void spinLeft()
{
  // right motor
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, HIGH);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}

void spinRight()
{
  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, HIGH);
}

// to do velocity
void travelStraight(int speed, double distance)
{
  // keep the velocity in [0 .. 100] range
  speed = max(speed, 0);
  speed = min(speed, 100);

  // right motor
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

  // left motor
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}
// ---------------------------------------  Servo control
void servoTurn(int deg)
{
  servoControl.write(deg);
  delay(SWait);
}
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
/*  https://github.com/JRodrigoTech/Ultrasonic-HC-SR04/blob/master/Ultrasonic/Ultrasonic.cpp
 *  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
*/
void sensorSetup()
{
  // Calculate the max reflection distance in us
  MaxEchoDistTime = (MaxDetectDistCM + 1) * USTime1CM;
  
  pinMode(EchoPin, INPUT);
  pinMode(TriggerPin, OUTPUT);

  // Allow for the analog pins to setup
  delay(1000);
  
  // Read a few values for the sensor calibration
  for(int i = 0; i < 4; i++)
  {
    readDistance();
    delay(200);
  }
}

int readDistance()
{
  delay(100);
  return sonarSignal() != 0 ? sonarSignal() : NoEcho;
}

boolean sendOutSoundWave()
{
  // set the trigger pin for the sensor to output
  pinMode(TriggerPin, OUTPUT);

  // the trigger pin should be off (low) at the start
  digitalWrite(TriggerPin, LOW);
  // wait for the pin to go low
  delayMicroseconds(10);

  // turn on the sensor by sending a high voltage through the pin = trigger the sensor to send out a sound signal (ping)
  digitalWrite(TriggerPin, HIGH);
  // wait for the sensor to detect the high on from the pin
  delayMicroseconds(10);

  // turn off the pin
  digitalWrite(TriggerPin, LOW);

  pinMode(TriggerPin, INPUT);

  // if the pin is sending data in, the previous read hasn't completed yet
  if (digitalRead(EchoPin))
  {
    return false;
  }
  else
  {
    // Set the timeout for the signal to be send
    EchoTimeout = micros() + MaxEchoDistTime + MaxSensorDelay;

    // Wait for the signal to send
    while (!digitalRead(EchoPin))
    {
      // If it takes too long, return
      if (micros() > EchoTimeout)
      {
        return false;
      }
    }
    
     // Singal sent, start the reflection (echo) timeout
    EchoTimeout = micros() + MaxEchoDistTime;
    return true;
  }
}

unsigned int sonarSignal()
{
  boolean signalSent = sendOutSoundWave();
  
  // If something went wrong, no signal was sent, return
  if (signalSent == false) 
  {
    return NoEcho;
  }
  else
  {
    // Read from the pin = wait for the sound signal to come back (the echo = the reflection)
    while (digitalRead(EchoPin))
    {
      // If it timesout it means we are beyond the max distance
      if (micros() > EchoTimeout) 
      {
        return NoEcho;
      }
    }
    return ((micros() - (EchoTimeout - MaxEchoDistTime) - PingOverhead) / USTime1CM); // Calculate ping time, include overhead.
  }
}



