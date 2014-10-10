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
volatile unsigned int timeSum = 0; // Sum of the velocity queue. One less floating pont operation in the interrupt.
volatile unsigned int tickTimes[5] = {0,0,0,0,0}; // time delta between ticks - used for velocity calculations
// 5 elements so that we have an average
volatile byte timeIndex = 0; // where to insert into the tickTimes array

// other
boolean LED = false;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  MS.begin();
  motor->setSpeed(128);
  motor->run(FORWARD);
  
  attachInterrupt(0,readEncoder, CHANGE); // attaches to pin 2
  
  motor->setSpeed(255);
}

void loop() {
  //motor->setSpeed(something);
}

void readEncoder() {
  timeSum -= tickTimes[timeIndex]; // subtract the oldest value in the array from running sum
  int time = millis(); // get when the tick happened - useful for velocity
  time -= tickTimes[(timeIndex + 4) % 5]; // get a time delta by subtracting the previous time
  tickTimes[timeIndex] = time; // replace the oldest value with the new time delta
  timeSum += time; // add the new value to the running sum
  timeIndex = (timeIndex+1) % 5; // move up where to insert into the array
  
  ticks = (ticks+1) % ENCODER_TICKS; // basically position
  
  // for debugging:
  digitalWrite(13, !LED);
  LED = !LED;
//  Serial.print("Time delta (ms): ");
//  Serial.println(time);
//  Serial.print("Position: ");
//  Serial.println(ticks);
}

double getVelocity() {
  return timeSum / 5; // average velocity over past 5 readings
}

double getPosition() {
  return ticks * DEGREES_TICK; // what angle we're at, more or less
}
