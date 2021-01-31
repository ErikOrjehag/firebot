
#include "Arduino.h"
#include <PID_v1.h>

class Motor
{
  int feedbackPin;
  int pwmPin;
  int dirPin;
  int side;
  unsigned long feedbackTs = 0;
  double measuredSpeed = 0;
  double filteredSpeed = 0;
  double desiredSpeed = 0;
  double outputSpeed = 0;
  int desiredDir = LOW;
  double Kp = 0.001;
  double Ki = 0.005;
  double Kd = 0.00000;
  PID pid;

public:
  Motor(int feedbackPin, int pwmPin, int dirPin, int side);
  void setupPins();
  int getFeedbackPin();
  void feedbackInterrupt();
  void setSpeed(double desiredSpeed);
  void update();

};
