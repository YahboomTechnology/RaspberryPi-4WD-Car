/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         servo_ultrasonic_avoid.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        Ultrasonic obstacle avoidance with servo
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

#define ON  1       
#define OFF 0       

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor


int key = 10;                 //Define the button is wiringPi port 10 of Raspberry pi 

int EchoPin = 30;             //Define the EchoPin connect to wiringPi port 30 of Raspberry pi 
int TrigPin = 31;             //Define the TrigPin connect to wiringPi port 31 of Raspberry pi 

//Definition of Pin
int LED_R = 3;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
int LED_G = 2;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
int LED_B = 5;           //LED_B is connected to  wiringPi port 5 of Raspberry pi

//Define the servo pin
int ServoPin = 4;

//Initialize servo position forward
int ServoPos = 90;

const int AvoidSensorLeft =  26; //Obstacle avoidance infrared sensor pin on the left is connected to  wiringPi port 26 of Raspberry pi
const int AvoidSensorRight = 0;  //Obstacle avoidance infrared sensor pin on the right is connected to  wiringPi port 0 of Raspberry pi

int LeftSensorValue ;            //Define variables to store data collected by each Obstacle avoidance infrared sensor 
int RightSensorValue ;

void brake();
void spin_left(int);
void spin_right(int);
void back(int);
float Distance_test();
void bubble(unsigned long *, int);
void corlor_led(int, int, int);

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    ServoPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.08.16
* @brief         The servo rotates to the specified angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 15; i++)     //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(ServoPin, pos);//Generate the PWM in the analog mode
  }
}

/**
* Function       servo_color_carstate
* @author        Danny
* @date          2017.08.16
* @brief          servo_color_carstate
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_color_carstate()
{
  float distance;

  int iServoPos = 0;
  int LeftDistance = 0;    
  int RightDistance = 0;   
  int FrontDistance = 0;   
  corlor_led(ON, OFF, OFF);//LED_red
  back(80);               //Avoid stop suddenly
  brake();

  //The servo rotates to 0°(right)to measure distance
  servo_appointed_detection(0);
  delay(500);
  distance = Distance_test();  
  RightDistance = distance;   
  //printf("rightdistance :%d\n",RightDistance);
 
  //The servo rotates to 180°(left)to measure distance
  servo_appointed_detection(180);
  delay(500);
  distance = Distance_test();  
  LeftDistance = distance;
  // printf("leftdistance :%d\n",LeftDistance);

  //The servo rotates to 90°(front)to measure distance
  servo_appointed_detection(90);
  delay(500);
  distance = Distance_test();
  FrontDistance = distance;
  //printf("FrontDistance:%d\n",FrontDistance);
 
  if (LeftDistance < 30 && RightDistance < 30 && FrontDistance < 30  )
  {
    //LED_magenta
    corlor_led(ON, OFF, ON);
    spin_right(700);
  }
  else if ( LeftDistance >= RightDistance) 
  {
    //LED_blue
    corlor_led(OFF, OFF, ON);
    spin_left(350);
  }
  else if (LeftDistance < RightDistance ) 
  {
    //LED_magenta
    corlor_led(ON, OFF, ON);
    spin_right(350);
  }
}

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         advance
* @param[in1]    LeftSpeed
* @param[in2]    RightSpeed
* @param[out]    void
* @retval        void
* @par History   
*/
void run(int LeftSpeed, int RightSpeed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, LeftSpeed);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, RightSpeed);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         brake
* @param[in]
* @param[out]    void
* @retval        void
* @par History   
*/
void brake()
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         turn left(left wheel stop, right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void left()
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, LOW); 
  softPwmWrite(Left_motor_pwm, 0);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW);
  softPwmWrite(Right_motor_pwm, 80);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         turn right(right wheel stop, left wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, 80);

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         turn left in place(left wheel back, right wheel advance)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left(int time)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, 100);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, 100);

  delay(time);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         turn right in place(left wheel zdvance, right wheel back)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right(int time)
{
   //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, 100);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 100);

  delay(time);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         back
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void back(int time)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, 80);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 80);

  delay(time);
}

/**
* Function       corlor_led
* @author        Danny
* @date          2017.08.16
* @brief         7 different colors formed by different combinations of R,G and B
* @param[in1]    v_iRed:Red
* @param[in2]    v_iGreen:Green
* @param[in3]    v_iBlue:Blue
* @retval        void
* @par History   
*/
void corlor_led(int v_iRed, int v_iGreen, int v_iBlue)
{
  v_iRed == ON ? digitalWrite(LED_R, HIGH): digitalWrite(LED_R, LOW);
 
  v_iGreen == ON ? digitalWrite(LED_G, HIGH) : digitalWrite(LED_G, LOW);
  
  v_iBlue == ON ? digitalWrite(LED_B, HIGH) : digitalWrite(LED_B, LOW);
}

/**
* Function       Distance
* @author        Danny
* @date          2017.08.16
* @brief         measure the distance by Ultrasonic
* @param[in]     void
* @param[out]    void
* @retval        float:distance
* @par History   
*/
float Distance()
{
	float distance;
	struct timeval tv1;
	struct timeval tv2;
	struct timeval tv3;
	struct timeval tv4;
	long start, stop;
	
	digitalWrite(TrigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(TrigPin, HIGH);      //Input a high level of at least 10 US to the Trig pin
	delayMicroseconds(15);
	digitalWrite(TrigPin, LOW);
    
    gettimeofday(&tv3, NULL);        
	start = tv3.tv_sec * 1000000 + tv3.tv_usec;
	while(!digitalRead(EchoPin) == 1)
	{
		gettimeofday(&tv4, NULL);   
		stop = tv4.tv_sec * 1000000 + tv4.tv_usec;
		
		if ((stop - start) > 30000)  //Maximum time value (5m)：10/340=0.03s
		{
			return -1;              
		}
	} 
	
	gettimeofday(&tv1, NULL);      
    start = tv1.tv_sec*1000000+tv1.tv_usec;
	while(!digitalRead(EchoPin) == 0)
	{
		gettimeofday(&tv3,NULL);   
		stop = tv3.tv_sec*1000000+tv3.tv_usec;
		if ((stop - start) > 30000)
		{
			return -1;
		}
	}                    
	gettimeofday(&tv2, NULL);      

	start = tv1.tv_sec * 1000000 + tv1.tv_usec;
	stop = tv2.tv_sec * 1000000 + tv2.tv_usec;

	distance = (float)(stop - start)/1000000 * 34000 / 2;
	return distance;
}

/**
* Function       Distane_test
* @author        Danny
* @date          2017.08.16
* @brief         Remove the maximum, minimum of the 5 datas, and get average values of 3 datas to improve accuracy of test
* @param[in]     void
* @param[out]    void
* @retval        float:distance
* @par History   
*/
float Distance_test()
{
  float distance;
  unsigned long ultrasonic[5] = {0};
  int num = 0;
  while (num < 5)
  {
     distance = Distance();

	 while((int)distance == -1)
	 {
		 distance = Distance();
	 }
    //Filter out data greater than 500 or smaller than 0 in the test distance
    while ( distance >= 500 || (int)distance == 0)
    {
         distance = Distance();
    }
    ultrasonic[num] = distance;
    num++;
	delay(10);
  }
  num = 0;
  bubble(ultrasonic, 5);
  distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
  
  printf("distance:%f\n",distance);      
  return distance;
}

/**
* Function       bubble
* @author        Danny
* @date          2017.08.16
* @brief         Bubble sorting 
* @param[in1]    a:Ultrasonic array first address
* @param[in2]    n:Size of Ultrasonic array 
* @param[out]    void
* @retval        void
* @par History   
*/
void bubble(unsigned long *a, int n)

{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}

/**
* Function       key_scan
* @author        Danny
* @date          2017.08.16
* @brief         key detection (including software-key debounce)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void key_scan()
{
  while (digitalRead(key));       //Loops this code when the key is not pressed
  while (!digitalRead(key))       //When the key is pressed
  {
    delay(10);	                  
    if (digitalRead(key)  ==  LOW)//Re-determine whether the key was pressed
    {
      delay(100);
      while (!digitalRead(key));  //Determine whether the key is released
    }
  }
  return;
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         When key is pressed, Ultrasonic obstacle avoidance with servo mode is opened
*                Infrared obstacle avoidance module assisted obstacle avoidance (ultrasonic obstacle avoidance has blind zone)
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
  float distance;
  
  //Initialize wiringPi
  wiringPiSetup();	

  //Initialize the motor drive IO as the output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //Create two software-controlled PWM pins to control motor speed 
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);

  //Initialize the RGB IO as the output mode
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  //Initialize the key interface as the input mode
  pinMode(key, INPUT);

  //Initialize ultrasonic pin
  pinMode(EchoPin, INPUT);    
  pinMode(TrigPin, OUTPUT);  

  servo_appointed_detection(ServoPos);
  
  //Initialize the servo interface as the input mode
  pinMode(ServoPin, OUTPUT);

  //Initialize the AvoidSensor interface as the input mode
  pinMode(AvoidSensorLeft, INPUT);
  pinMode(AvoidSensorRight, INPUT);

  key_scan();
  
  while(1)
  {
   distance = Distance_test();        
   if (distance > 50  )    
   {
    // There is obstacle, the indicator light of the infrared obstacle avoidance module is on, and the port level is LOW
    // There is no obstacle, the indicator light of the infrared obstacle avoidance module is off, and the port level is HIGH
    LeftSensorValue = digitalRead(AvoidSensorLeft);
    RightSensorValue = digitalRead(AvoidSensorRight);

    if (LeftSensorValue == HIGH && RightSensorValue == LOW)
    {
      spin_left(120); 
    }
    else if (RightSensorValue == HIGH && LeftSensorValue == LOW)
    {
      spin_right(120);
    }
    else if (RightSensorValue == LOW && LeftSensorValue == LOW)
    {
      spin_right(120);
    }

	run(150, 150);
    corlor_led(OFF, ON, OFF);
  }
  else if ((distance >= 30 && distance <= 50))
  {
    // There is obstacle, the indicator light of the infrared obstacle avoidance module is on, and the port level is LOW
    // There is no obstacle, the indicator light of the infrared obstacle avoidance module is off, and the port level is HIGH
    LeftSensorValue = digitalRead(AvoidSensorLeft);
    RightSensorValue = digitalRead(AvoidSensorRight);

    if (LeftSensorValue == HIGH && RightSensorValue == LOW)
    {
      spin_left(120);
    }
    else if (RightSensorValue == HIGH && LeftSensorValue == LOW)
    {
      spin_right(120);
    }
    else if (RightSensorValue == LOW && LeftSensorValue == LOW)
    {
      spin_right(120);
    }

    run(120, 120);
  }
  else if (  distance < 30  )
  {
    servo_color_carstate();
  }
 }
	return;
}



