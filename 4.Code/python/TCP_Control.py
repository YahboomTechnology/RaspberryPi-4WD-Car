#-*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import socket
import time
import string
import threading

#Define pins of button
run_car  = '1'  #advance
back_car = '2'  #back
left_car = '3'  #left
right_car = '4' #right
stop_car = '0'  #stop

#Define pin of servo
front_left_servo = '1'  #front servo turn left
front_right_servo = '2' #front servo turn right
up_servo = '3'          #camera servo up
down_servo = '4'        #camera servo down
left_servo = '6'        #camear servo left
right_servo = '7'       #camera servo right
updowninit_servo = '5'  #camera servo reset
stop_servo = '8'        #servo stop

#Define car status
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6

#Define servo of car
enFRONTSERVOLEFT = 1
enFRONTSERVORIGHT = 2
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOLEFT = 6
enSERVORIGHT = 7
enSERVOSTOP = 8



#set servo is 90
ServoLeftRightPos = 90
ServoUpDownPos = 90
g_frontServoPos = 90
g_nowfrontPos = 0


#Define pin of motor
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Define key
key = 8

#Define ultrasonic pins
EchoPin = 0
TrigPin = 1

#Define RGB light pins
LED_R = 22
LED_G = 27
LED_B = 24 

#Define servo pins
FrontServoPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11

#Define IR avoid pins
AvoidSensorLeft = 12
AvoidSensorRight = 17

#Define buzzer pins
buzzer = 8

#Define outfire pin
OutfirePin = 2

#Define tracking pins
#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #left 1 pin
TrackSensorLeftPin2  =  5   #left 2 pin
TrackSensorRightPin1 =  4   #right 1 pin
TrackSensorRightPin2 =  18  #right 2 pin

#Light sensor pin
LdrSensorLeft = 7
LdrSensorRight = 6

#Define clorful RGB light variables
red = 0
green = 0
blue = 0
#TCP communication packet flags and receive and send data variables
NewLineReceived = 0
InputString = ''
recvbuf = ''
ReturnTemp = ''
#car and servo status variable
g_CarState = 0
g_ServoState = 0
#Speed variable
CarSpeedControl = 80 
#Tracking, avoid， seek-light
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
g_lednum = 0

#Set the GPIO port to BCM encoding
GPIO.setmode(GPIO.BCM)

#Ignore warning messages
GPIO.setwarnings(False)

#Motor pin set to output mode
#Key pin set to input mode
#Ultrasonic, RGB light,servo pin
#IR avoid
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_FrontServo
    global pwm_UpDownServo
    global pwm_LeftRightServo
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
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(FrontServoPin, GPIO.OUT)
    GPIO.setup(ServoUpDownPin, GPIO.OUT)
    GPIO.setup(ServoLeftRightPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    GPIO.setup(LdrSensorLeft,GPIO.IN)
    GPIO.setup(LdrSensorRight,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
    #set pwm pin and frequency 2000Hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    #set servo pin frequency 2000Hz
    pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
    pwm_FrontServo.start(0)
    pwm_UpDownServo.start(0)
    pwm_LeftRightServo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)
	
#car advance
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#car back
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#car turn left
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#car turn right
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#car spin left
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#car spin right
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#car stop	
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)
				
#Distance test
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
	
#Frnot servo angle
def frontservo_appointed_detection(pos): 
    for i in range(18):   
    	pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							
    	#pwm_FrontServo.ChangeDutyCycle(0)	

#cemera servo left and right angle
def leftrightservo_appointed_detection(pos): 
    for i in range(1):   
    	pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							
    	#pwm_LeftRightServo.ChangeDutyCycle(0)	

#cemera servo up and down angle
def updownservo_appointed_detection(pos):  
    for i in range(1):  
    	pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							
    	#pwm_UpDownServo.ChangeDutyCycle(0)	

#Tracking
def tracking_test():
    global infrared_track_value
    #Detect black line, light is on. Low level
    #Detect no black line, light is off. High level
    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    infrared_track_value_list = ['0','0','0','0']
    infrared_track_value_list[0] = str(1 ^TrackSensorLeftValue1)
    infrared_track_value_list[1] = str(1 ^TrackSensorLeftValue2)
    infrared_track_value_list[2] = str(1 ^TrackSensorRightValue1)
    infrared_track_value_list[3] = str(1 ^TrackSensorRightValue2)
    infrared_track_value = ''.join(infrared_track_value_list)
    

#IR avoid 
def infrared_avoid_test():
    global infrared_avoid_value
    #Detect obstacle, light is on. Low level
    #No detect obstacle, light is off. High level
    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    infrared_avoid_value_list = ['0','0']
    infrared_avoid_value_list[0] = str(1 ^LeftSensorValue)
    infrared_avoid_value_list[1] = str(1 ^RightSensorValue)
    infrared_avoid_value = ''.join(infrared_avoid_value_list)
    	
#Seek-light
def follow_light_test():
    global LDR_value
    #
    #
    LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
    LdrSersorRightValue = GPIO.input(LdrSensorRight)  
    LDR_value_list = ['0','0']
    LDR_value_list[0] = str(LdrSersorLeftValue)
    LDR_value_list[1] = str(LdrSersorRightValue)	
    LDR_value = ''.join(LDR_value_list)
	
#Buzzer
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)	
	
#RGB light control
def color_led_pwm(iRed,iGreen, iBlue):
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)

#camera servo up
def servo_up():
    global ServoUpDownPos
    pos = ServoUpDownPos
    updownservo_appointed_detection(pos)
    #time.sleep(0.05)
    pos +=0.7 
    ServoUpDownPos = pos
    if ServoUpDownPos >= 180:
        ServoUpDownPos = 180

#camera servo down	
def servo_down():
    global ServoUpDownPos
    pos = ServoUpDownPos
    updownservo_appointed_detection(pos)
    #time.sleep(0.05)
    pos -= 0.7
    ServoUpDownPos = pos
    if ServoUpDownPos <= 45:
        ServoUpDownPos = 45
    

#camera servo left	
def servo_left():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos += 0.7
    ServoLeftRightPos = pos
    if ServoLeftRightPos >= 180:
        ServoLeftRightPos = 180

#camera servo right
def servo_right():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos -= 0.7 
    ServoLeftRightPos = pos
    if ServoLeftRightPos <= 0:
        ServoLeftRightPos =  0

#Front servo left
def front_servo_left():
    frontservo_appointed_detection(180)

#Front servo right
def front_servo_right():
    frontservo_appointed_detection(0)

#All servo reset
def servo_init():
    servoflag = 0
    servoinitpos = 90
    if servoflag != servoinitpos:        
        frontservo_appointed_detection(servoinitpos)
        updownservo_appointed_detection(servoinitpos)
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.5)
        pwm_FrontServo.ChangeDutyCycle(0)	
        pwm_LeftRightServo.ChangeDutyCycle(0)	
        pwm_UpDownServo.ChangeDutyCycle(0)	

#camera servo reset
def servo_updown_init():
    updownservo_appointed_detection(90)
	
#servo stop
def servo_stop():
    pwm_LeftRightServo.ChangeDutyCycle(0)	
    pwm_UpDownServo.ChangeDutyCycle(0)	
    pwm_FrontServo.ChangeDutyCycle(0)	
		
#tcp data parsing and specify the corresponding action
def tcp_data_parse():
    global NewLineReceived
    global CarSpeedControl
    global g_CarState
    global g_ServoState
    global g_frontServoPos
    global red
    global green
    global blue
    global g_lednum
    #Analysis of servo instructions sent from the host computer
    #	
    if (InputString.find("$4WD,PTZ", 0, len(InputString)) != -1):
        i = InputString.find("PTZ",  0, len(InputString)) 
        ii = InputString.find("#",  0, len(InputString))
  	if ii > i:
            string = InputString[i+3:ii]
	    m_kp = int(string)
	    g_frontServoPos = 180 - m_kp;
	    NewLineReceived = 0
	    InputString.zfill(len(InputString))
		  
    #Analysis of colorful searchlight instructions sent from the host computer
    #Eg:$4WD,CLR255,CLG0,CLB0# RGB light is red
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
        color_led_pwm(red, green, blue)		  
        NewLineReceived = 0
        InputString.zfill(len(InputString))
		  
    #Analysis of car instructions sent from the host computer
    #如:$1,0,0,0,0,0,0,0,0,0#    car advance
    if (InputString.find("$4WD", 0, len(InputString)) == -1) and (InputString.find("#",  0, len(InputString)) != -1):
        if InputString[3] == '1':
            g_CarState = enTLEFT      #spin left
        elif InputString[3] == '2':
            g_CarState = enTRIGHT     #spin right
        else:
            g_CarState = enSTOP

        if InputString[5] == '1':    
            whistle()                 #whistle

        if InputString[7] == '1':
            CarSpeedControl += 20
            if CarSpeedControl > 100:
                CarSpeedControl = 100 #add 
        if InputString[7] == '2':
            CarSpeedControl -= 20
            if CarSpeedControl < 20:  #dece
                CarSpeedControl = 20
        #RGB light
        if InputString[13] == '1':
            g_lednum=g_lednum+1
            if g_lednum == 1:
                color_led_pwm(255, 255, 255)
            elif g_lednum == 2:
                color_led_pwm(255, 0, 0)
            elif g_lednum == 3:
                color_led_pwm(0, 255, 0)
            elif g_lednum == 4:
                color_led_pwm(0, 0, 255)
            elif g_lednum == 5:
                color_led_pwm(255, 255, 0)
            elif g_lednum == 6:
                color_led_pwm(0, 255, 255)
            elif g_lednum == 7:
                color_led_pwm(255, 0, 255)
            else : 
                color_led_pwm(0, 0 ,0)
                g_lednum = 0

        if InputString[13] == '2':
            color_led_pwm(255, 0, 0)
        if InputString[13] == '3':
            color_led_pwm(0, 255, 0)
        if InputString[13] == '4':
            color_led_pwm(0, 0, 255)	
        #Out fire
        if InputString[15] == '1':
            GPIO.output(OutfirePin,not GPIO.input(OutfirePin) )
            time.sleep(1)
        #servo reset
        if InputString[17] == '1':
            g_frontServoPos = 90

        #car status 
        if g_CarState != enTLEFT and g_CarState != enTRIGHT:
            if InputString[1] == run_car:
                g_CarState = enRUN
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
        #servo status
        			
	if InputString[9] == front_left_servo:
	    g_frontServoPos = 180
	elif InputString[9] == front_right_servo:
	    g_frontServoPos = 0
        elif InputString[9] == up_servo:
	    g_ServoState = enSERVOUP
	elif InputString[9] == down_servo:
	    g_ServoState = enSERVODOWN
        elif InputString[9] == left_servo:
	    g_ServoState = enSERVOLEFT
        elif InputString[9] == right_servo:
	    g_ServoState = enSERVORIGHT
        elif InputString[9] == updowninit_servo:
	    g_ServoState = enSERVOUPDOWNINIT
	elif InputString[9] == stop_servo:
	    g_ServoState = enSERVOSTOP
	else:
	    g_ServoState = enSERVOSTOP
            
        NewLineReceived = 0
        InputString.zfill(len(InputString))

	#make car complete movement 
        if g_CarState == enSTOP:
            brake()          
        elif g_CarState == enRUN:
            run()
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

#
def Data_Pack():
    global InputString
    global NewLineReceived
    if recvbuf[0] == '$' and recvbuf.find("#",  0, len(recvbuf)) != -1:
        InputString = recvbuf
        NewLineReceived = 1
        print "InputString: %s" % InputString
		
#The collected sensor data is sent back to the host computer for serial display.
def tcp_data_postback():
    #The collected sensor data is sent back to the host computer for serial display.
    #format：
    #$4WD,CSB120,PV8.3,GS214,LF1011,HW11,GM11#
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
    print "ReturnTemp: %s" % ReturnTemp
    return ReturnTemp
#Define the servo control thread
def ServorThread():
    #Judge the state of the servo and execute the corresponding function
    global g_frontServoPos
    global g_nowfrontPos
    
    if g_ServoState == enSERVOUP:
        servo_up()
    elif g_ServoState == enSERVODOWN:
	servo_down()
    elif g_ServoState == enSERVOLEFT:
	servo_left()
    elif g_ServoState == enSERVORIGHT:
	servo_right()
    elif g_ServoState == enSERVOUPDOWNINIT:
	servo_updown_init()
    elif g_ServoState == enSERVOSTOP:
    	servo_stop()
    
    if g_nowfrontPos != g_frontServoPos:
    	frontservo_appointed_detection(g_frontServoPos)
    	g_nowfrontPos = g_frontServoPos
    	pwm_FrontServo.ChangeDutyCycle(0)	
            
    

		  
try:
    init()
    servo_init()
    global g_ServoState
    global timecount
    global connectflag 
    connectflag = 0
    timecount = 1000
    count = 50
    #
    tcpservicesock= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    tcpservicesock.setblocking(0)
    #Input ip address and port number
    #Note: Please input your own IP address
    tcpservicesock.bind(('192.168.50.1', 8888))
    #
    tcpservicesock.listen(5)
    print "waiting for connection...."
    #
    clientAddrList = []
    thread1 = threading.Thread(target = ServorThread)
    thread1.start()
    thread1.join()

    while True:
        try:
            #
            #print "Start accept!"
            tcpclientsock,addr = tcpservicesock.accept()
            if  tcpclientsock:
                connectflag = 1
        except:
            pass 
        else:
            print "new user :%s " % str(addr)
            #
            tcpclientsock.setblocking(0)
            clientAddrList.append((tcpclientsock,addr))
        for tcpclientsock,addr in clientAddrList:
            try:
                global recvbuf
                global sendbuf
                recvbuf = ''
                #
                print "Start recv!"
                recvbuf = tcpclientsock.recv(128)
                print "Start recv over!"
            except:
                pass
            else:
                if len(recvbuf) > 0:
                     #
                     Data_Pack()
                     if NewLineReceived == 1:
                         #
                         tcp_data_parse()
                else:
                    tcpclientsock.close()
                    clientAddrList.remove((tcpclientsock,addr))
        #
        timecount -= 1
        if timecount == 0:
           count -= 1
           timecount = 1000
           if count == 0:
               sendbuf = ''
               sendbuf = tcp_data_postback()
               if not sendbuf:
                   break
               if  connectflag:
                   #get data
                   tcpclientsock.send(sendbuf)
               timecount = 1000
               count = 50
        ServorThread()
except KeyboardInterrupt:
    pass
tcpclientsock.close()
tcpservicesock.close()
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_FrontServo.stop()
pwm_LeftRightServo.stop()
pwm_UpDownServo.stop()
GPIO.cleanup()	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	




