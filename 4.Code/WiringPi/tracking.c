/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         tracking.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        tracking
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

int key = 10;                 //Key connects to wiringPi port 10 of Raspberry pi 

//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      9                 21                  7                   1
const int TrackSensorLeftPin1  =  9;   //The first tracking infrared sensor pin on the left is connected to  wiringPi port 9 of Raspberry pi
const int TrackSensorLeftPin2  =  21;  //The second tracking infrared sensor pin on the left is connected to  wiringPi port 21 of Raspberry pi
const int TrackSensorRightPin1 =  7;   //The first tracking infrared sensor pin on the right is connected to  wiringPi port 7 of Raspberry pi
const int TrackSensorRightPin2 =  1;   //The second tracking infrared sensor pin on the right is connected to  wiringPi port 1 of Raspberry pi

//Define variables to store data collected by each tracking infrared pin
int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         advance
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void run(int left_speed, int right_speed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, left_speed );

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         brake
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void brake(int time)
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);

  delay(time * 100);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         turn left(left wheel stop, right wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void left(int left_speed, int right_speed)
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW);
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         turn right(right wheel stop, left wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void right(int left_speed, int right_speed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, left_speed);

  //Left motor stop
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         turn left in place(left wheel back, right wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left(int left_speed, int right_speed)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         turn right in place(left wheel zdvance, right wheel back)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right(int left_speed, int right_speed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, right_speed);
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
  softPwmWrite(Left_motor_pwm, 40);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 40);

  delay(time );
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
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         When key is pressed, Ultrasonic obstacle avoidance mode is opened
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
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
  
   //Initialize the key interface as the input mode
  pinMode(key, INPUT);

  //Initialize the tracksensor IO as the input mode
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);

  key_scan();
  
  while(1)
  {
   // When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
   // When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
   TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
   TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
   TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
   TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

   //4 tracking pins level status
   // 0 0 X 0
   // 1 0 X 0
   // 0 1 X 0
   //Turn right in place,speed is 150,delay 80ms
   //Handle right acute angle and right right angle
   if ( (TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW)
   {
     spin_right(150, 150);
     delay(80);
   }
   //4 tracking pins level status
   // 0 X 0 0       
   // 0 X 0 1 
   // 0 X 1 0       
   //Turn right in place,speed is 150,delay 80ms   
   //Handle left acute angle and left right angle 
   else if ( TrackSensorLeftValue1 == LOW && (TrackSensorRightValue1 == LOW ||  TrackSensorRightValue2 == LOW))
   {
     spin_left(150, 150);
     delay(80);
   }
    // 0 X X X
    //Left_sensor1 detected black line
   else if ( TrackSensorLeftValue1 == LOW)
   {
     spin_left(150, 150);
    // delay(10);
   }
    // X X X 0
    //Right_sensor2 detected black line
   else if ( TrackSensorRightValue2 == LOW )
   {
     spin_right(150, 150);
    // delay(10);
   }
    //4 tracking pins level status
    // X 0 1 X
   else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
   {
     left(0, 150);
   }
    //4 tracking pins level status
    // X 1 0 X  
   else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
   {
     right(150, 0);
   }
    //4 tracking pins level status
    // X 0 0 X
   else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
   {
     run(150, 150);
   }
    //When the level of 4 pins are 1 1 1 1 , the car keeps the previous running state.
 }
 return;
}

