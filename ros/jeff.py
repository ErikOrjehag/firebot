

import RPi.GPIO as GPIO
from time import sleep

servoPin = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin, GPIO.OUT)

pwm=GPIO.PWM(servoPin, 50)
pwm.start(0)



def setAngle(angle):
    duty = angle / 18 + 3
    GPIO.output(servoPin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servoPin, False)
    pwm.ChangeDutyCycle(duty)

class Snuffer:
    def __init__(self, port):
        self.port = port

    def snuff(self):
        setAngle(10)

    def unSnuff(self):
        setAngle(170)


snuffer = Snuffer(port=4)
snuffer.snuff()

# setAngle(0)

# sleep(2)


# for vinkel in range(0,150,30):
#     setAngle(vinkel)
#     sleep(1)


# setAngle(0)
# sleep(1)
# setAngle(90)
# sleep(3)
# setAngle(110)

pwm.stop()
GPIO.cleanup()
