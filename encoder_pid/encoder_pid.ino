#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = MS.getMotor(1);

// some constants
const unsigned short ENCODER_TICKS = 40; // number of ticks in one revolution of the encoder
const unsigned short DEGREES_TICK = 360 / ENCODER_TICKS;

// actual P, I, and D values for the PID controller
double p;
double i;
double d;

// PID gains
static double kp = 0.5;
static double ki = 0.5;
static double kd = 0.5;

// starting position of the motor
volatile byte ticks = 0; // ticks of the encoder - essentially position
volatile unsigned long timeSum = 0; // Sum of the velocity queue. One less floating pont operation in the interrupt.
volatile unsigned long tickTimes[5] = {0,0,0,0,0}; // time delta between ticks - used for velocity calculations
volatile unsigned long prevTime = 0;
// 5 elements so that we have an average
volatile byte timeIndex = 0; // where to insert into the tickTimes array

// other
boolean LED = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  attachInterrupt(0,readEncoder, CHANGE); // attaches to pin 2
  
  MS.begin();
  
  delay(5000);
  motor->setSpeed(110);
  motor->run(FORWARD);
}

void loop() {
  //motor->setSpeed(something);
}

void readEncoder() {
  timeSum -= tickTimes[timeIndex]; // subtract the oldest value in the array from running sum
  unsigned long time = millis(); // get when the tick happened - useful for velocity
//  Serial.print("Raw time: ");
//  Serial.println(time);
  //time -= tickTimes[(timeIndex + 4) % 5]; // get a time delta by subtracting the previous time
  
//  Serial.print("Time delta (ms): ");
//  Serial.println(time - prevTime);
  tickTimes[timeIndex] = time - prevTime; // replace the oldest value with the new time delta
  timeSum += time - prevTime; // add the new value to the running sum
  
  prevTime = time;
  timeIndex = (timeIndex+1) % 5; // move up where to insert into the array
  
  ticks = (ticks+1) % ENCODER_TICKS; // basically position
  
  // for debugging:
  digitalWrite(13, !LED);
  LED = !LED;
  
//  Serial.print("Position: ");
//  Serial.println(ticks);
//  Serial.print("Array: ");
//  Serial.print(tickTimes[0]);
//  Serial.print(",");
//  Serial.print(tickTimes[1]);
//  Serial.print(",");
//  Serial.print(tickTimes[2]);
//  Serial.print(",");
//  Serial.print(tickTimes[3]);
//  Serial.print(",");
//  Serial.println(tickTimes[4]);
  
}

double getVelocity() {
  return timeSum / 5; // average velocity over past 5 readings
}

double getPosition() {
  return ticks * DEGREES_TICK; // what angle we're at, more or less
}
