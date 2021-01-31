
#include "motor.h"

Motor leftMotor(2, 9, 7, 1);
Motor rightMotor(3, 11, 8, -1);

void leftMotorInterrupt() { leftMotor.feedbackInterrupt(); }
void rightMotorInterrupt() { rightMotor.feedbackInterrupt(); }

void setup()
{
  Serial.begin(115200);
  leftMotor.setupPins();
  rightMotor.setupPins();
  attachInterrupt(digitalPinToInterrupt(leftMotor.getFeedbackPin()), leftMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightMotor.getFeedbackPin()), rightMotorInterrupt, CHANGE);

  //leftMotor.setSpeed(5);
  //rightMotor.setSpeed(5);
}

void loop()
{
  leftMotor.setSpeed( 150 * sin(0.0005*millis()));
  rightMotor.setSpeed(150 * sin(0.0005*millis()));
  //leftMotor.setSpeed(40 * ((millis() / 5000) % 2 == 0 ? 1: -1));
  //rightMotor.setSpeed(40 * ((millis() / 5000) % 2 == 0 ? 1: -1));
  leftMotor.update();
  rightMotor.update();
  /*Serial.print(" ");
  Serial.print(-60.0);
  Serial.print(" ");
  Serial.print(60.0);
  Serial.println();*/
}
