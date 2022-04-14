#include <limits.h>
#include <TPA81.h>
#include "timer.hpp"

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define N_SENSORS 8
SFEVL53L1X sensors[N_SENSORS];
const int xshuts[N_SENSORS] = {
  3, 4, 5, 9, 10, 11, A2, A3
};

#define N_TEMPS 8
int temps[N_TEMPS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
TPA81 tpa;

#define REC_BUF_LEN 7
int8_t rec_buf[REC_BUF_LEN] = {0, 0, 0, 0, 0, 0, 0};

#define N_MOTORS 2
int PIN_ENABLE[] = { 2, 8 };
int PIN_DIR[] = { 7, A0 };
int DIR[] = { LOW, HIGH };
int PIN_STEP[] = { 6, A1 };
long TIMER_INTERVAL = 125L;
long STEPS_PER_REV = 800L;
unsigned int motor_ticks[N_MOTORS] = { UINT_MAX, UINT_MAX };
unsigned int motor_remaining[N_MOTORS] = { 1, 1 };
unsigned long cmd_ts = 0;
#define MAX_TIME_BETWEEN_CMD 1000

#define START 0x02
#define STOP 0x03
#define SEND_BUF_SIZE 3 + N_SENSORS * 2 + N_TEMPS
uint8_t send_buf[SEND_BUF_SIZE];

void setup() {
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  for (int i = 0; i < N_SENSORS; i++) {
    pinMode(xshuts[i], OUTPUT);
    digitalWrite(xshuts[i], LOW);
  }
  delay(10);

  for (int i = 0; i < N_SENSORS; i++) {
    digitalWrite(xshuts[i], HIGH);
    delay(1);
    if (sensors[i].begin() != 0) //Begin returns 0 on a good init
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println("failed to begin. Please check wiring. Freezing...");
      while (1);
    }
    sensors[i].setI2CAddress(0x29 + i*2);
    sensors[i].startRanging();
    sensors[i].setTimingBudgetInMs(33);
    sensors[i].setIntermeasurementPeriod(38);
    Serial.print("Sensor");
    Serial.print(i);
    Serial.println("is online!");
  }
 
  set_motor_speed(0, 0);
  set_motor_speed(1, 0);
  
  install_timer(125L);
}

void loop() {
  
  for (int i = 0; i < N_SENSORS; i++) {
    while (!sensors[i].checkForDataReady())
    {
      delay(1);
    }
    int distance = sensors[i].getDistance();
    send_buf[1+i*2+0] = distance >> 8;
    send_buf[1+i*2+1] = distance & 0xFF;
  }
  
  int ambient = tpa.getAll(temps);
  for (int i = 0; i < N_TEMPS; i++) {
    send_buf[1+N_SENSORS*2+i] = temps[i];
  }
    
  send_buf[0] = START;
  send_buf[SEND_BUF_SIZE - 1] = STOP;
  uint8_t checksum = 0;
  for (int i = 1; i < SEND_BUF_SIZE - 2; i++) {
    checksum ^= send_buf[i];
  }
  send_buf[SEND_BUF_SIZE - 2] = checksum;

  for (int i = 0; i < SEND_BUF_SIZE; i++) {
    Serial.write(send_buf[i]);
  }
  
  while (Serial.available()) {
    for (int i = 0; i < REC_BUF_LEN - 1; i++) {
      rec_buf[i] = rec_buf[i + 1];
    }
    int c = Serial.read();
    rec_buf[REC_BUF_LEN - 1] = c;
    if (rec_buf[0] == START && rec_buf[6] == STOP && rec_buf[1] ^ rec_buf[2] ^ rec_buf[3] ^ rec_buf[4] == rec_buf[5]) {
      int16_t s0 = (((uint16_t) rec_buf[1]) << 8) | (uint8_t)rec_buf[2];
      int16_t s1 = (((uint16_t) rec_buf[3]) << 8) | (uint8_t)rec_buf[4];
      digitalWrite(LED_BUILTIN, HIGH);
      set_motor_speed(0, s0);
      set_motor_speed(1, s1);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  unsigned long now = millis();
  if (now - cmd_ts > MAX_TIME_BETWEEN_CMD) {
    set_motor_speed(0, 0);
    set_motor_speed(1, 0);
  }
}

void set_motor_speed(int motor, int speed) {
  cmd_ts = millis();
  
  unsigned int ticks;
  if (speed == 0) {
    ticks = UINT_MAX;
  } else {
    ticks = (unsigned int) ((1000L * 1000000L) / (TIMER_INTERVAL * STEPS_PER_REV * abs(speed)));
  }
  motor_ticks[motor] = ticks;

  digitalWrite(PIN_ENABLE[motor], ticks == UINT_MAX);
  if (DIR[motor]) {
    digitalWrite(PIN_DIR[motor], speed < 0);
  } else {
    digitalWrite(PIN_DIR[motor], !(speed < 0));
  }
}

ISR(TIMER1_COMPA_vect) {

  for (int i = 0; i < N_MOTORS; i++) {
    if (motor_ticks[i] != UINT_MAX) {
      motor_remaining[i]--;
      if (motor_remaining[i] == 0) {
        motor_remaining[i] = motor_ticks[i];
        digitalWrite(PIN_STEP[i], HIGH);
      }
    }
  }

  for (int i = 0; i < 10; i++) __asm__("nop\n\t");

  for (int i = 0; i < N_MOTORS; i++) {
    digitalWrite(PIN_STEP[i], LOW);
  }
}
