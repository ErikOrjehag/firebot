

import RPi.GPIO as GPIO
import time
import microphone
from smbus import SMBus
from tpa81 import TPA81
import VL53L1X
import time
from multiprocessing import Process, Queue

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

def tpa81_test():
    tpa = TPA81(bus_num=1)
    #tpa.changeAddress(0)
    print('Version: ', tpa.softwareVersion())
    print('Ambient Temperature: ', tpa.ambientTemperature())
    for i in range(len(tpa.TPA81_PIXEL)):
        print('Pixel', (i + 1), ': ', tpa.pixelTemp(i))
    #print('Highest Temp: ', tpa.highestTemp())
    #print("Close")
    #tpa.bus.close()

def tpa81_and_i2c():
  tpa = TPA81(bus_num=1)
  
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
    
    for i in range(len(tpa.TPA81_PIXEL)):
      print('Pixel', (i + 1), ': ', tpa.pixelTemp(i))

tofs = []

def f(i, q):
  try:
    while True:
      q.put((i, tofs[i].get_distance()))
  except KeyboardInterrupt:
    pass

def VL53L1X_test():
  xshut = [17, 27, 22, 10, 9, 11, 5, 6]
  #xshut = [17, 22, 9, 5]

  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)

  for pin in [17, 27, 22, 10, 9, 11, 5, 6]:
    GPIO.setup(pin, GPIO.OUT)

  for pin in [17, 27, 22, 10, 9, 11, 5, 6]:
    GPIO.output(pin, GPIO.LOW)

  for offset, pin in enumerate(xshut, 1):
    GPIO.output(pin, GPIO.HIGH)
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    tof.change_address(0x29 + offset)
    tof.close()

  for offset, pin in enumerate(xshut, 1):
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29 + offset)
    tof.open()
    tof.set_timing(33000, 38)
    tof.start_ranging(0)
    tofs.append(tof)

  i = 1
  ts = time.time()
  try:
      while True:
        if i % 100 == 0:
          print(f"{100.0 / (time.time() - ts):.0f} Hz")
          i = 0
          ts = time.time()
        i = i + 1
        dists = [tof.get_distance() for tof in tofs]
        print(dists)
        # hz = 1.0 / (time.time() - ts)
        # print(f"{hz:.1f}Hz " + ", ".join(f"{d:d}" for d in dists))
  except KeyboardInterrupt:
    pass
  
  for tof in tofs:
    tof.stop_ranging()
  

  

  

if __name__ == "__main__":
  # microphone.block_until_start_signal()
  # ledsAndBuzzer()
  # servo()
  # i2c()
  tpa81_test()
  VL53L1X_test()
  # tpa81_and_i2c()
