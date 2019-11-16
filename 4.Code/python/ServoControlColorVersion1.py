#-*- coding:UTF-8 -*-
#This code uses the PWM library that comes with the system.
import RPi.GPIO as GPIO
import time

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

#Definition of servo pin
ServoPin = 23

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#RGB module pins are initialized into output mode
#Servo pin is initialized into input mode
def init():
    global pwm_servo
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)

    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)

def corlor_light(pos):
    if pos > 150:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos > 125:
	GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos >100:
        GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 75:
	GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos > 50:
	GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 25:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 0:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.HIGH)
    else :
        GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.LOW)
		
def servo_control_color():
    for pos in range(181):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	corlor_light(pos)
	time.sleep(0.009) 
    for pos in reversed(range(181)):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	corlor_light(pos)
	time.sleep(0.009)

#delay 2s		
time.sleep(2)

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
    init()
    pwm_servo.ChangeDutyCycle(2.5 + 10 * 90/180)
    while True:
 	servo_control_color()
		
except KeyboardInterrupt:
    pass
pwm_servo.stop()
GPIO.cleanup()
