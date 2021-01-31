
#include "motor.h"

Motor::Motor(int feedbackPin, int pwmPin, int dirPin, int side)
  : feedbackPin(feedbackPin), pwmPin(pwmPin), dirPin(dirPin), side(side),
    pid(&filteredSpeed, &outputSpeed, &desiredSpeed, Kp, Ki, Kd, DIRECT)
{
  
}

void Motor::setupPins()
{
  /*Serial.println(feedbackPin);
  Serial.println(pwmPin);
  Serial.println(dirPin);
  Serial.println("----");*/
  pinMode(feedbackPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  analogWrite(pwmPin, 255);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1.0, 1.0);
}

int Motor::getFeedbackPin()
{
  return feedbackPin;
}

void Motor::feedbackInterrupt()
{
  if (digitalRead(feedbackPin)) {
      feedbackTs = micros();
  } else {
    if (feedbackTs != 0) {
      long delta = micros() - feedbackTs;
      if (delta > 5e4) {
        measuredSpeed = 0.0;
      } else {
        measuredSpeed = 111111.0 / (double)delta * (outputSpeed >= 0 ? 1.0 : -1.0);
      }
    }
  }
}

void Motor::setSpeed(double desiredSpeed)
{
  this->desiredSpeed = side*desiredSpeed;
}

void Motor::update()
{
  long delta = micros() - feedbackTs;
  if (delta > 5e4) {
    measuredSpeed = 0.0;
    feedbackTs = 0;
  }
  filteredSpeed += 0.1 * (measuredSpeed - filteredSpeed);
  
  pid.Compute();
  
  /*Serial.print(measuredSpeed);
  Serial.print(" ");
  Serial.print(filteredSpeed);
  Serial.print(" ");
  Serial.print(desiredSpeed);
  Serial.print(" ");
  Serial.print(outputSpeed*30.0);*/
  
  int pwm = 255 - 255 * constrain(abs(outputSpeed), 0.0, 1.0);
  
  analogWrite(pwmPin, pwm);
  digitalWrite(dirPin, outputSpeed >= 0 ? LOW : HIGH);
  
  //float error = desiredSpeed - measuredSpeed;
  //int pwm = 255 - 255 * constrain(error * pConst, 0.f, 1.f);
  //analogWrite(pwmPin, pwm);
  //digitalWrite(dirPin, desiredDir);
}
