#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import string
import serial

#Definition of  key value 
run_car  = '1'  
back_car = '2'  
left_car = '3' 
right_car = '4' 
stop_car = '0'  

#Definition of  status value 
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6


#Definition of  motor pins
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of  key
key = 8

#Definition of ultrasonic module pins
EchoPin = 0
TrigPin = 1

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24 

#Definition of servo pin
ServoPin = 23

#Definition of infrared obstacle avoidance module pins
AvoidSensorLeft = 12
AvoidSensorRight = 17

#Definition of buzzer pin
buzzer = 8

#Definition of outfire pin
OutfirePin = 2

#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #The first tracking infrared sensor pin on the left is connected to  BCM port 3 of Raspberry pi
TrackSensorLeftPin2  =  5   #The second tracking infrared sensor pin on the left is connected to  BCM port 5 of Raspberry pi
TrackSensorRightPin1 =  4   #The first tracking infrared sensor pin on the right is connected to  BCM port 4 of Raspberry pi
TrackSensorRightPin2 =  18  #The second tracking infrared sensor pin on the right is connected to  BCM port 18 of Raspberry pi

#Definition of photoresistor pins
LdrSensorLeft = 7
LdrSensorRight = 6

global timecount 
global count 

red = 0
green = 0
blue = 0
NewLineReceived = 0
InputString = ''
InputStringcache = ''
g_CarState = 0 
CarSpeedControl = 50 
g_num = 0
g_packnum = 0 
ReturnTemp = ''
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
StartBit = 0
#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Ultrasonic pin,RGB pin,servo pin initialization
#infrared obstacle avoidance module pin
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
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    GPIO.setup(LdrSensorLeft,GPIO.IN)
    GPIO.setup(LdrSensorRight,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
    #Set the PWM pin and frequency is 2000hz
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
	

	
#Advance
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#back
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#trun left
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#turn right
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#turn left in place
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#turn right in place
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#brake
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

#Button detection
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
	        pass
				
#Ultrasonic function
def Distance_test():
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
        t1 = time.time()
    while GPIO.input(EchoPin):
        pass
        t2 = time.time()
    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    time.sleep(0.01)
    return ((t2 - t1)* 340 / 2) * 100
	
#The servo rotates to the specified angle
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)

#tracking test
def tracking_test():
    global infrared_track_value

    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    infrared_track_value_list = ['0','0','0','0']
    infrared_track_value_list[0] = str(1^ TrackSensorLeftValue1)
    infrared_track_value_list[1] =str(1^ TrackSensorLeftValue2)
    infrared_track_value_list[2] = str(1^ TrackSensorRightValue1)
    infrared_track_value_list[3] = str(1^ TrackSensorRightValue2)
    infrared_track_value = ''.join(infrared_track_value_list)
    

#infrared avoid test
def infrared_avoid_test():
    global infrared_avoid_value

    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    infrared_avoid_value_list = ['0','0']
    infrared_avoid_value_list[0] = str(1 ^ LeftSensorValue)
    infrared_avoid_value_list[1] = str(1 ^ RightSensorValue)
    infrared_avoid_value = ''.join(infrared_avoid_value_list)
    	
# follow light test
def follow_light_test():
    global LDR_value

    LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
    LdrSersorRightValue = GPIO.input(LdrSensorRight)  
    LDR_value_list = ['0','0']
    LDR_value_list[0] = str(LdrSersorLeftValue)
    LDR_value_list[1] = str(LdrSersorRightValue)	
    LDR_value = ''.join(LDR_value_list)
	
#whistle
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)	
	
#Color_LED light the specified color
def color_led_pwm(iRed,iGreen, iBlue):
    print iRed 
    print iGreen
    print iBlue
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    print v_red
    print v_green
    print v_blue
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)
	
#The serial port data is parsed and the corresponding action is specified
def serial_data_parse():
    global NewLineReceived
    global CarSpeedControl
    global g_CarState
    global red
    global green
    global blue
    #Analyze the control instructions of the servo sent by the host computer and execute corresponding operation
     #For exmaple:$4WD,PTZ180# servo turn 180°
    if (InputString.find("$4WD,PTZ", 0, len(InputString)) != -1):
        i = InputString.find("PTZ",  0, len(InputString)) 
        ii = InputString.find("#",  0, len(InputString))
  	if ii > i:
            string = InputString[i+3:ii]
	    m_kp = int(string)
	    servo_appointed_detection(180 - m_kp)
	    NewLineReceived = 0
	    InputString.zfill(len(InputString))
            print "in"
            print InputString
		  
     #Analyze the control instructions sent by the host computer and execute corresponding operation
     #For exmaple:$4WD,CLR255,CLG0,CLB0# color_red
    if (InputString.find("CLR", 0, len(InputString)) != -1):
        i = InputString.find("CLR", 0,  len(InputString)) 
        ii = InputString.find(",CLG",  0,  len(InputString))
	if ii > i:
           string = InputString[i+3:ii]
	   m_kp = int(string)
	   red = m_kp
        i = InputString.find("CLG",  0, len(InputString)) 
        ii = InputString.find(",CLB",  0, len(InputString))
	if ii > i:
           string = InputString[i+3:ii]
	   m_kp = int(string)
	   green = m_kp
        i = InputString.find("CLB",  0, len(InputString)) 
        ii = InputString.find("#",  0,  len(InputString))
	if ii > i:
            string = InputString[i+3:ii]
	    m_kp = int(string)
	    blue = m_kp
            print "red :%d " % red
            print green
            print blue
        color_led_pwm(red, green, blue)		  
        NewLineReceived = 0
        InputString.zfill(len(InputString))
		  
      #Analyze the control instructions sent by the host computer and execute corresponding operation
     #For exmaple:$1,0,0,0,0,0,0,0,0,0#    advance
    if (InputString.find("$4WD", 0, len(InputString)) == -1) and (InputString.find("#",  0, len(InputString)) != -1):
        if InputString[3] == '1':
            g_CarState = enTLEFT
            print "g_CarState: %d" % g_CarState
        elif InputString[3] == '2':
            g_CarState = enTRIGHT
        else:
            g_CarState = enSTOP
        if InputString[5] == '1':
            whistle()
        if InputString[7] == '1':
            CarSpeedControl += 20
        if CarSpeedControl > 100:
            CarSpeedControl = 100
        if InputString[7] == '2':
            CarSpeedControl -= 20
        if CarSpeedControl < 20:
            CarSpeedControl = 20
        if InputString[9] == '1':
            servo_appointed_detection(180)
        if InputString[9] == '2':
            servo_appointed_detection(0)
        if InputString[13] == '1':
            color_led_pwm(255, 255, 255)
        if InputString[13] == '2':
            color_led_pwm(255, 0, 0)
        if InputString[13] == '3':
            color_led_pwm(0, 255, 0)
        if InputString[13] == '4':
            color_led_pwm(0, 0, 255)	
          
        if InputString[15] == '1':
            GPIO.output(OutfirePin,not GPIO.input(OutfirePin) )
            time.sleep(1)
  
        if InputString[17] == '1':
            servo_appointed_detection(90)
        print "carstate:%d" % g_CarState
        if g_CarState != enTLEFT and g_CarState != enTRIGHT:
            print "hellonice"
            print run_car
            print InputString[1]
            if InputString[1] == run_car:
                g_CarState = enRUN
                print "run car"
            elif InputString[1] == back_car:
                g_CarState = enBACK	
            elif InputString[1] == left_car:
                g_CarState = enLEFT
            elif InputString[1] == right_car:
                g_CarState = enRIGHT
            elif InputString[1] == stop_car:
                g_CarState = enSTOP
            else:
                g_CarState = enSTOP				  
        NewLineReceived = 0
        InputString.zfill(len(InputString))	
		  
#The collected sensor data is transmitted by the serial port to the host computer for display
def serial_data_postback():

    global ReturnTemp
    ReturnTemp = ''
    distance = Distance_test()
    ReturnTemp += "$4WD,CSB"
    ReturnTemp += str(int(distance))
    ReturnTemp += ",PV8.4"
    ReturnTemp += ",GS0"
    ReturnTemp += ",LF"
    tracking_test()
    ReturnTemp += infrared_track_value
    ReturnTemp += ",HW"
    infrared_avoid_test()
    ReturnTemp += infrared_avoid_value
    ReturnTemp += ",GM"
    follow_light_test()
    ReturnTemp += LDR_value
    ReturnTemp += "#"
    print ReturnTemp
    ser.write(ReturnTemp)
	
def serialEvent():
    global InputString
    global InputStringcache
    global StartBit
    global NewLineReceived
    InputString = ''
    while True:
        size = ser.inWaiting()
        if size == 0:
           
            break
        else:
            while size != 0:
                serialdatabit = ser.read(1)
                size -= 1
                if serialdatabit == '$':
                    StartBit = 1
                if StartBit == 1:
                    InputStringcache += serialdatabit
                if StartBit == 1 and serialdatabit == '#':
                    NewLineReceived = 1
                    InputString = InputStringcache
                    InputStringcache = ''
                    StartBit = 0
                    size = 0
                    print InputString	
          
try:
    ser = serial.Serial("/dev/ttyAMA0", 9600, timeout = 0.001)
    print "serial.isOpen() = ",ser.isOpen()
    ser.write("serial is on!")  
    timecount = 2000
    count = 100
    init()
    while True:
        serialEvent()
     #   time.sleep(0.4)
	if NewLineReceived == 1:
            print "serialdata:%s" % InputString
	    serial_data_parse()
	    NewLineReceived = 0

        #print "nice to meet you"	
	if g_CarState == enSTOP:
	    brake()          
	elif g_CarState == enRUN:
	    run()
            print "running"
	elif g_CarState == enLEFT:
	    left()
	elif g_CarState == enRIGHT:
	    right()
	elif g_CarState == enBACK:
	    back()
	elif g_CarState == enTLEFT:
	    spin_left()
	elif g_CarState == enTRIGHT:
	    spin_right()
	else:
	    brake()
       
       # print "hello woek"
        
	timecount -= 1
        #print time
	if timecount == 0:
	    count -= 1
            timecount = 2000
       	    if count == 0:
	        serial_data_postback()
	        timecount = 2000
	        count = 100
		    	
except KeyboardInterrupt:
    pass
ser.close()
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_servo.stop()
GPIO.cleanup()
	
	
	
	
	
	
