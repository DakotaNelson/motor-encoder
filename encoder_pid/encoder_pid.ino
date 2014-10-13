#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = MS.getMotor(1);

// some constants
const unsigned short ENCODER_TICKS = 40; // number of ticks in one revolution of the encoder
const unsigned short DEGREES_TICK = 360 / ENCODER_TICKS;

// PID gains
static double kp = 2.5;
static double ki = 1;
//static double kd = 0.5;

// starting position of the motor
volatile byte ticks = 20; // ticks of the encoder - essentially position
volatile unsigned long timeSum = 0; // Sum of the velocity queue. One less floating pont operation in the interrupt.
volatile unsigned long tickTimes[5] = {0,0,0,0,0}; // time delta between ticks - used for velocity calculations
volatile unsigned long prevTime = 0;
// 5 elements so that we have an average
volatile byte timeIndex = 0; // where to insert into the tickTimes array
volatile bool direction = true; // true = forward, false = backward

// other
boolean LED = true;

// for the PID controller
float error_sum = 0; // used for the I component of PID
unsigned long time;

void setup() {
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  attachInterrupt(0,readEncoder, CHANGE); // attaches to pin 2
  
  MS.begin();
  //motor->setSpeed(255);
  //motor->run(FORWARD);
  delay(1000);
  //motor->setSpeed(0);
  
  time = millis();
}

void loop() {
  float objective = 90;

  float position = getPosition();
  float error = objective - position;
  
  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Error: ");
  Serial.println(error);
  
  float speed = PID(error, millis() - time);
  time = millis(); // update the previous time
  
  setSpeed(speed);
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
  
  if(direction) {
    ticks = (ticks+1) % ENCODER_TICKS; // basically position
  } else {
    ticks = (ticks-1) % ENCODER_TICKS; // going backward
  }
  
  // for debugging:
  digitalWrite(13, LED);
  LED = !LED;
  
  //Serial.print("Pos: ");
  //Serial.println(ticks);
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
  return DEGREES_TICK / 1000*(timeSum / 5); // average velocity over past 5 readings, in deg/s
}

double getPosition() {
  return ticks * DEGREES_TICK; // what angle we're at, more or less
}

float PID(float error, unsigned long timeDelta) {
  error_sum += error * (float(timeDelta)/1000); // accumulate error for the I term
  
  float P = kp * error; // error in degrees
  float I = ki * error_sum; // error_sum in degrees
  //float D = kd * getVelocity(); // velocity in deg/s
  Serial.println("PI:");
  Serial.println(P);
  Serial.println(I);
  
  error_sum *= 0.9; // error sum should decay over time
  return P+I; //+D;
}

void setSpeed(float speed) {
  // now scale to [0-255]
  if(speed > 0) {
    motor->run(FORWARD);
    direction = true;
  } else {
    motor->run(BACKWARD);
    direction = false;
  }
  
  speed = abs(speed);
  
  //if(speed < 80) {
  //  speed = 80;
  //}
  if(speed > 255) {
    speed = 255;
  }
  
  Serial.print("Setting speed: ");
  Serial.println(int(speed));
  motor->setSpeed(int(speed));
}
