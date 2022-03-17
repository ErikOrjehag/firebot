#include <limits.h>
#include "timer.hpp"
#include <Wire.h>

int LED_PIN = 13;
int NUM_MOTORS = 2;
int PIN_ENABLE[] = { 2, A7 };
int PIN_DIR[] = { 7, A0 };
int PIN_STEP[] = { 6, A1 };
int PIN_MS1[] = { 3, A6 };
int PIN_MS2[] = { 4, A3 };
int PIN_MS3[] = { 5, A2 };

long TIMER_INTERVAL = 125L;
long STEPS_PER_REV = 800L;

unsigned int* motor_ticks;
unsigned int* motor_remaining;

bool led = false;
unsigned long t = 0;

void setup() {

  Serial.begin(9600);
  
  Wire.begin(0x8);

  Wire.onReceive(receiveEvent);
  
  motor_ticks = new unsigned int[NUM_MOTORS];
  motor_remaining = new unsigned int[NUM_MOTORS];
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_ticks[i] = UINT_MAX;
    motor_remaining[i] = 0;
    
    pinMode(PIN_ENABLE[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_MS1[i], OUTPUT);
    pinMode(PIN_MS2[i], OUTPUT);
    pinMode(PIN_MS3[i], OUTPUT);
    
    digitalWrite(PIN_ENABLE[i], LOW);
    digitalWrite(PIN_DIR[i], HIGH);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_MS1[i], LOW);
    digitalWrite(PIN_MS2[i], HIGH);
    digitalWrite(PIN_MS3[i], LOW);
  }

  install_timer(125L);
}


#define L 7
const int8_t START = 0x10101010;
const int8_t STOP  = 0x01010101;
int8_t buff[L] = {0, 0, 0, 0, 0, 0, 0};

void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    int8_t c = Wire.read();
    for (int i = 0; i < L-1; i++) {
      buff[i] = buff[i+1];
    }
    buff[L-1] = c;

    if (buff[0] == START && buff[6] == STOP && buff[1] ^ buff[2] ^ buff[3] ^ buff[4] == buff[5]) {
      int16_t s0 = (((uint16_t) buff[1]) << 8) | (uint16_t)buff[2];
      int16_t s1 = (((uint16_t) buff[3]) << 8) | (uint16_t)buff[4];
      Serial.println(buff[1]);
      Serial.println((uint16_t)buff[2]);
      Serial.println(s0);
      if (buff[1] == 3 && buff[2] == 232) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    } else {
    }
  }
}

void loop() {
  int speed = 1000*(sin(millis() / 1000.0));
  set_motor_speed(0, speed);
  set_motor_speed(1, speed);
  delay(10);

  //while (Wire.available()) {
  //  Serial.println("something available");
  //  Wire.read();
  //}
}

void on_steer() {
  t++;
  if (t > 50) {
    t = 0;
    led = !led;
    digitalWrite(LED_PIN, led);
  }
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    //int speed = cmd.readBinArg<int>();
    int speed = 10;
    Serial.print(char(speed));
    set_motor_speed(i, speed);
  }
}

void cmd_on() {
  digitalWrite(LED_PIN, HIGH);
}

void cmd_off() {
  digitalWrite(LED_PIN, LOW);
}

void set_motor_speed(int motor, int speed) {
  unsigned int ticks = (unsigned int) ((1000L * 1000000L)/(TIMER_INTERVAL*STEPS_PER_REV*abs(speed)));
  motor_ticks[motor] = ticks;
  
  //digitalWrite(PIN_ENABLE[motor], ticks != UINT_MAX);
  digitalWrite(PIN_DIR[motor], speed < 0);
  //digitalWrite(PIN_DIR[motor], HIGH);
}

ISR(TIMER1_COMPA_vect) {

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_ticks[i] != UINT_MAX) {
      motor_remaining[i]--;
      if (motor_remaining[i] == 0) {
        motor_remaining[i] = motor_ticks[i];
        digitalWrite(PIN_STEP[i], HIGH);
      }
    }
  }
  
  for (int i = 0; i < 10; i++) __asm__("nop\n\t");

  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(PIN_STEP[i], LOW);
  }
}
