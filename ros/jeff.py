

import RPi.GPIO as GPIO
import time
import microphone
from smbus import SMBus
from tpa81 import TPA81

def servo():
  servoPin = 13

  GPIO.setmode(GPIO.BCM)
  GPIO.setup(servoPin, GPIO.OUT)

  p = GPIO.PWM(servoPin, 50)

  p.start(2.5) # Initialization
  try:
    while True:
      p.ChangeDutyCycle(5)
      time.sleep(0.5)
      p.ChangeDutyCycle(7.5)
      time.sleep(0.5)
      p.ChangeDutyCycle(10)
      time.sleep(0.5)
      p.ChangeDutyCycle(12.5)
      time.sleep(0.5)
      p.ChangeDutyCycle(10)
      time.sleep(0.5)
      p.ChangeDutyCycle(7.5)
      time.sleep(0.5)
      p.ChangeDutyCycle(5)
      time.sleep(0.5)
      p.ChangeDutyCycle(2.5)
      time.sleep(0.5)
  except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()

def ledsAndBuzzer():
  ledPinGreen = 16
  ledPinRed = 25
  ledPinYellow = 12
  buzzPin = 21
  btnPinRed = 23
  btnPinGreen = 24

  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)

  GPIO.setup(ledPinGreen, GPIO.OUT)
  GPIO.setup(ledPinYellow, GPIO.OUT)
  GPIO.setup(ledPinRed, GPIO.OUT)
  GPIO.setup(buzzPin, GPIO.OUT)
  GPIO.setup(btnPinRed, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(btnPinGreen, GPIO.IN, pull_up_down=GPIO.PUD_UP)

  start = time.time()

  try:
    while True:
      
      if int(time.time() - start) % 2 == 0:
        GPIO.output(ledPinYellow, GPIO.HIGH)
        GPIO.output(buzzPin, GPIO.LOW)
      else:
        GPIO.output(ledPinYellow, GPIO.LOW)
        GPIO.output(buzzPin, GPIO.HIGH)
      
      if GPIO.input(btnPinRed) == GPIO.LOW:
        GPIO.output(ledPinRed, GPIO.HIGH)
      else:
        GPIO.output(ledPinRed, GPIO.LOW)
      
      if GPIO.input(btnPinGreen) == GPIO.LOW:
        GPIO.output(ledPinGreen, GPIO.HIGH)
      else:
        GPIO.output(ledPinGreen, GPIO.LOW)

  except KeyboardInterrupt:
    GPIO.cleanup()

def i2c():
  addr = 0x8 # bus address
  bus = SMBus(1) # indicates /dev/ic2-1
  
  numb = 1
  
  print ("Enter 1 for ON or 0 for OFF")
  while numb == 1:
  
    ledstate = input(">>>>   ")
  
    if ledstate == "1":
      bus.write_byte(addr, 0x1) # switch it on
    elif ledstate == "0":
      bus.write_byte(addr, 0x0) # switch it on
    else:
      numb = 0

def tpa81():
  tpa = TPA81(bus_num=1)
  print('Version: ', tpa.softwareVersion())
  print('Ambient Temperature: ', tpa.ambientTemperature())
  for i in range(len(tpa.TPA81_PIXEL)):
      print(f'Pixel {i}: {tpa.pixelTemp(i)}')

def vl53l1x():
  # https://github.com/pimoroni/vl53l1x-python
  pass


if __name__ == "__main__":
  # microphone.block_until_start_signal()
  # ledsAndBuzzer()
  # servo()
  # i2c()
  # tpa81()
  vl53l1x()
