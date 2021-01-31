#include <ros2arduino.h>
#include <user_config.h>

//Variabler
float wheelSpeed1=0;
float wheelSpeed2=0;

unsigned long pulseTime1 = 0;
unsigned long pulseTime2 = 0;

int pwmOut1 = 255;
int pwmOut2 = 255;
const float filterAlfa=0.5;
const float pRegConst1=0.01;
const float pRegConst2=0.01;

//Pins
const int feedbackPin1=2;
const int pwmPin1=9;
const int dirPin1=7;

const int feedbackPin2=3;
const int pwmPin2=11;
const int dirPin2=8;



void measureWheelSpeed1() {

  if (digitalRead(feedbackPin1)){
      pulseTime1=micros();
  } else {
      if (pulseTime1!=0){
        int dpulseTime = micros()-pulseTime1;
        float measWheelSpeed = 111111.f / (float)dpulseTime;
        wheelSpeed1 += filterAlfa*(measWheelSpeed-wheelSpeed1);
        pulseTime1=0;
      }
    }
}

void measureWheelSpeed2() {

  if (digitalRead(feedbackPin2)){
      pulseTime2=micros();
  } else {
      if (pulseTime2!=0){
        int dpulseTime = micros()-pulseTime2;
        float measWheelSpeed = 111111.f / (float)dpulseTime;
        wheelSpeed2 += filterAlfa*(measWheelSpeed-wheelSpeed2);
        pulseTime2=0;
      }
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(10, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(11, OUTPUT); //PWM PIN 11  with PWM wire

  //attachInterupt setup
 pinMode(feedbackPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(feedbackPin1), measureWheelSpeed1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(feedbackPin2), measureWheelSpeed2, CHANGE);
}

void setSpeed(float desiredSpeed, int motorNum){
  if (motorNum==1){
  float dSpeed=desiredSpeed-wheelSpeed1;
  int reqPwm=dSpeed*pRegConst1*255;
  reqPwm=255-constrain(reqPwm,0,255);
  analogWrite(pwmPin1,reqPwm);
  Serial.println(dSpeed);
  //Serial.println(reqPwm);
  }
  else {
    float dSpeed=desiredSpeed-wheelSpeed2;
    int reqPwm=dSpeed*pRegConst2*255;
    reqPwm=255-constrain(reqPwm,0,255);
    analogWrite(pwmPin2,reqPwm);
    //Serial.println(reqPwm);      
  }
}

void loop() {
/*
  if (Serial.available()){
    pwmOut1=Serial.parseInt();
    Serial.print("pwm=");
    Serial.println(pwmOut1);
  }
  */
  // Serial.print("pwm2=");
  //Serial.println(pwmOut1);

  setSpeed(65,1);
  setSpeed(65,2);
  
  //analogWrite(pwmPin1,240);
  //analogWrite(pwmPin2,240);

  //Serial.print("wheelSpeed1 ");
  /*
  Serial.print(0);
  Serial.print(",");
  Serial.print(100);
  Serial.print(",");
  Serial.print(wheelSpeed1);
  Serial.print(",");
  Serial.println(wheelSpeed2);
  */
  //Serial.println("  r/min");
  
    /*for(int j = 0;j<1;j++)  {
      i += pulseIn(feedbackPin1, HIGH, 500000); //SIGNAL OUTPUT PIN 9 with  white line,cycle = 2*i,1s = 1000000us，Signal cycle pulse number：27*2
    }
    //i = i >> 3;
    Serial.println(111111 / i); //speed   r/min  (60*1000000/(45*6*2*i))
    i=111111/
    i = 0;*/
}
