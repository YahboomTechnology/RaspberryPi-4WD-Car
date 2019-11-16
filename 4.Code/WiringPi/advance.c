/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         advance.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        advance
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

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         advance
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int time)
{
  //Left motor advance 
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  //This code is for setting the PWM of the specified pin
  softPwmWrite(Left_motor_pwm, 150);    //pwm:0-255. The left and right wheels are slightly different

  //Right motor advance 
  digitalWrite(Right_motor_go, HIGH);   
  digitalWrite(Right_motor_back, LOW);  
  //This code is for set the PWM of the specified pin
  softPwmWrite(Right_motor_pwm, 150);   //pwm:0-255. The left and right wheels are slightly different

  delay(time * 100); //Unit:ms
  return;
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         The car will delay 2s and advance 
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
  wiringPiSetup();
  
  //Initializethe motor drive IO as the output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //int softPwmCreate(int pin,int initialValue,int pwmRange);
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);   
  
  delay(2000);   
  
  while(1)
  {
   run(10);       
  }
  return;
}
