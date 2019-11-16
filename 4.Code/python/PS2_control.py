#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import spidev


PSB_SELECT = 1
PSB_L3 =  2
PSB_R3 =  3
PSB_START = 4
PSB_PAD_UP = 5
PSB_PAD_RIGHT = 6
PSB_PAD_DOWN  = 7
PSB_PAD_LEFT  = 8
PSB_L2 = 9
PSB_R2 = 10
PSB_L1 = 11
PSB_R1 = 12
PSB_TRIANGLE = 13
PSB_CIRCLE = 14
PSB_CROSS  = 15
PSB_SQUARE = 16


PS2_DAT_PIN = 10 #MOS
PS2_CMD_PIN = 9  #MIS
PS2_SEL_PIN = 25 #CS  
PS2_CLK_PIN = 11 #SCK


PSS_RX = 5    
PSS_RY = 6    
PSS_LX = 7    
PSS_LY = 8    


enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6
enUPLEFT = 7
enUPRIGHT = 8
enDOWNLEFT = 9
enDOWNRIGHT = 10


IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13


ServoPin = 23


buzzer = 8


OutfirePin = 2


LED_R = 22
LED_G = 27
LED_B = 24


CarSpeedControl = 50


g_ServoState = 0

global PS2_KEY
global X1
global Y1
global X2
global Y2
global Handkey
scan=[0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
Data=[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
MASK = [PSB_SELECT,PSB_L3,PSB_R3,PSB_START,PSB_PAD_UP,PSB_PAD_RIGHT,PSB_PAD_DOWN,PSB_PAD_LEFT,PSB_L2,PSB_R2,PSB_L1,PSB_R1,PSB_TRIANGLE,PSB_CIRCLE,PSB_CROSS,PSB_SQUARE]


GPIO.setmode(GPIO.BCM)


GPIO.setwarnings(False)


def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    global pwm_rled
    global pwm_gled
    global pwm_bled   
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)

	
def spi_init():
    spi = spidev.SpiDev()
    spi.open(0,0)
    GPIO.setup(PS2_CMD_PIN,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(PS2_CLK_PIN,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(PS2_DAT_PIN, GPIO.IN)
    GPIO.setup(PS2_SEL_PIN,GPIO.OUT,initial=GPIO.HIGH)
	
	
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
	
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	

def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	

def upleft():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl-20)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl+20)    


def downleft():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl-20)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl+20)
	

def upright():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl+20)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl-20)   
	

def downright():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl+20)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl-20) 
	

def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)
   

def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)	
	

def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
        time.sleep(0.02)															
        pwm_servo.ChangeDutyCycle(0) 									


def color_led_pwm(iRed,iGreen, iBlue):
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)


def PS2_AnologaData(button):
    return Data[button]	
	

def PS2_ClearData():
    Data[:]=[]
	

def PS2_ReadData(command):
    res = 0
    j = 1 
    i = 0
    for i in range(8):
        if command & 0x01:
            GPIO.output(PS2_CMD_PIN, GPIO.HIGH)
	else:
	    GPIO.output(PS2_CMD_PIN, GPIO.LOW)
	command = command >> 1
	time.sleep(0.000008)
	GPIO.output(PS2_CLK_PIN, GPIO.LOW)
	time.sleep(0.000008)
	if GPIO.input(PS2_DAT_PIN):
	   res = res + j
	j = j << 1
	GPIO.output(PS2_CLK_PIN, GPIO.HIGH)
	time.sleep(0.000008)
    GPIO.output(PS2_CMD_PIN, GPIO.HIGH)
    time.sleep(0.00004)
    return res
	

def PS2_Datakey():
    global Data
    global scan
    index = 0
    i = 0
    PS2_ClearData()
    GPIO.output(PS2_SEL_PIN, GPIO.LOW)
    for i in range(9):
        Data.append( PS2_ReadData(scan[i]))
    GPIO.output(PS2_SEL_PIN, GPIO.HIGH)
	
    Handkey = (Data[4] << 8) | Data[3]
    for index in range(16):
        if Handkey & (1 << (MASK[index] - 1)) == 0:
	    return index+1
    return 0

try:
    init()
    spi_init()
    while True:
        global PS2_KEY
        global CarSpeedControl
        global g_ServoState
        flag = 0
        PS2_KEY = PS2_Datakey()
        #print "PS2_KEY is %d" % PS2_KEY
        
        if PS2_KEY == PSB_SELECT:
            print "PSB_SELECT"
        
        elif PS2_KEY == PSB_L3:
            print "PSB_L3"
            g_Carstate = enSTOP
        
        elif PS2_KEY == PSB_R3:
            print "PSB_R3"
            servo_appointed_detection(90)
        
        elif PS2_KEY == PSB_START:
            print "PSB_START"
        
        elif PS2_KEY == PSB_PAD_UP:
            print "PSB_PAD_UP"
            g_Carstate = enRUN 
        
        elif PS2_KEY == PSB_PAD_RIGHT:
            print "PSB_PAD_RIGHT"
            g_Carstate = enRIGHT
        
        elif PS2_KEY == PSB_PAD_DOWN:
	    print "PSB_PAD_DOWN"
            g_Carstate = enBACK
        
	elif PS2_KEY == PSB_PAD_LEFT:
	    print "PSB_PAD_LEFT"
    	    g_Carstate = enLEFT
        
	elif PS2_KEY == PSB_L2:
	    print "PSB_L2"
	    CarSpeedControl += 20
            if CarSpeedControl > 100:
	        CarSpeedControl = 100
        
        elif PS2_KEY == PSB_R2:
 	    print "PSB_R2"
 	    CarSpeedControl -= 20
            if CarSpeedControl < 0:
	        CarSpeedControl = 0
        
	elif PS2_KEY == PSB_L1:
	    print "PSB_L1"
        
	elif PS2_KEY == PSB_R1:
	    print "PSB_R1"
        
	elif PS2_KEY == PSB_TRIANGLE:
	    #print "PSB_TRIANGLE"
	    color_led_pwm(0,255,0)
	
	elif PS2_KEY == PSB_CIRCLE:
	    #print "PSB_CIRCLE"
	    color_led_pwm(0,0,255)
	   # time.sleep(0.1)
        
	elif PS2_KEY == PSB_SQUARE:
	    #print "PSB_SQUARE"
	    color_led_pwm(255,0,0)
	   # time.sleep(0.1)
        
        elif PS2_KEY == PSB_CROSS:
            #print "PSB_CROSS"
            whistle()
        else:
            g_Carstate = enSTOP
        
        if PS2_KEY == PSB_L1 or PS2_KEY == PSB_R1:
            X1 = PS2_AnologaData(PSS_LX)
            Y1 = PS2_AnologaData(PSS_LY)
            X2 = PS2_AnologaData(PSS_RX)
            Y2 = PS2_AnologaData(PSS_RY)
            
            if Y1 < 5 and  80 < X1 < 180:
                g_Carstate = enRUN
            elif Y1 > 230 and 80 < X1 < 180:
                g_Carstate = enBACK
            elif X1 < 5 and 80 < Y1 < 180:
                g_Carstate = enLEFT
            elif X1 > 230 and 80 < Y1 < 180:
                g_Carstate = enRIGHT
            elif X1 <= 80 and Y1 <= 80:
                g_Carstate = enUPLEFT
            elif X1 >= 180 and Y1 < 80:
                g_Carstate = enUPRIGHT
            elif Y1 >= 180 and X1 <= 80:
                g_Carstate = enDOWNLEFT
	    elif Y1 >= 180 and X1 >= 180:
	        g_Carstate = enDOWNRIGHT
	    else:
	        g_Carstate = enSTOP
				 
	    
	    if X2 < 5 and 110 < Y2 < 150: 
	        g_ServoState = 1        
	    elif X2 > 230 and 110 < Y2 < 150:
	        g_ServoState = 2        
	    else:
	        g_ServoState = 0
        		
	if g_ServoState == 0:
	    if flag != 0:
	        flag =0
	        servo_appointed_detection(90)
	elif g_ServoState == 1:
	    if flag != 1:
	        flag = 1
	        servo_appointed_detection(180)
        elif g_ServoState == 2:
            if flag != 2:
                flag = 2
                servo_appointed_detection(0)
        					  
	if g_Carstate == enSTOP:
	    brake()
	elif g_Carstate == enRUN:
	    run()
	elif g_Carstate == enLEFT:
	    left()
	elif g_Carstate == enRIGHT:
	    right()
	elif g_Carstate == enBACK:
	    back()
	elif g_Carstate == enTLEFT:
	    spin_left()
	elif g_Carstate == enTRIGHT:
	    spin_right()
	elif g_Carstate == enUPLEFT:
	    upleft()
	elif g_Carstate == enUPRIGHT:
	    upright()
	elif g_Carstate == enDOWNLEFT:
	    downleft()
	elif g_Carstate == enDOWNRIGHT:
	    downright()
	else:
	    brake()
        			 
	time.sleep(0.5)
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_servo.stop()
GPIO.cleanup()
				
	
	
	
	
	
	
	
	
	
	
	
	
  	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

    
  

