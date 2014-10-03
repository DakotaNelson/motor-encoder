#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = MS.getMotor(1);


// actual P, I, and D values
double p;
double i;
double d;

// PID gains
static double kp = 0.5;
static double ki = 0.5;
static double kd = 0.5;

// motor speed - current, ideal, and the command speed to send to the motor shield
int mspeed; // the speed (0-255) at which to run the motor
double current; // current speed
double goal; // ideal speed

void setup() {
  MS.begin();
  motor->setSpeed(128);
  motor->run(BACKWARD);
}

void loop() {
  //motor->setSpeed(mspeed);
}
