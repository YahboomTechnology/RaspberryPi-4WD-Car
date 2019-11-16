/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         CarRun.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        CarRun
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
* @par History   
*/
void run(int time)
{
  //Left motor advance 
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, 150);   

  //Right motor advance 
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, 150);  

  delay(time * 100);
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
* @brief         turn left (left wheel stop, right wheel advance)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void left(int time)
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, 0);      

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, 150); 

  delay(time * 100);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         turn right (left wheel advance, right wheel stop)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void right(int time)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH); 
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, 150);    

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, 0);     

  delay(time * 100);
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
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, HIGH); 
  softPwmWrite(Left_motor_pwm, 150);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         turn right in place(left wheel advance, right wheel back)
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
  softPwmWrite(Left_motor_pwm, 150);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         back 
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int time)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, HIGH); 
  softPwmWrite(Left_motor_pwm, 150);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         delay 2s，advance 1s，back 1s,turn left 2s,turn right 2s,
*                turn left in place 3s, turn right in place 3s,stop 0.5s
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
  
  while(1)
  {
   delay(2000);      
   run(10);          
   back(10);         
   left(20);         
   right(20);        
   spin_left(30);    
   spin_right(30);   
   brake(5);         
  }
  return;
}
