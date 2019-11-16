/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Servo_control.c
* @author       xiaozhen
* @version      V1.0
* @date         2019.02.14
* @brief        Servo_control
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
  
//Define the servo pin
int ServoPin = 4;     //front servo J1---4(wiringPi)，horizontal servo J2-----13(wiringPi)，vertical servo J3-----14(wiringPi)
/*If you need to control two other servos, please modify the definition of this pin.*/
//int ServoPin = 13;
//int ServoPin = 14;
/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int myangle)
{
  int PulseWidth;                     //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500;  //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
  return;
}

/**
* Function       servo_control_color
* @author        Danny
* @date          2017.08.16
* @brief         The servo turns from 0-180, then turns from 180 to 0.
*                At the same time, the 180 degree angle is divided into 7 sections to represent 7 different colors.
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_control()
{
  int pos = 0;
  for (pos = 0; pos < 180; pos++)
  {
    servo_pulse(pos);
    delay(20);
  }

  for (pos = 180; pos > 0; pos--)
  {
    servo_pulse(pos);
    delay(20);
  }
  return;
}


/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         Delay 0.5 s, Servo_control mode is opened
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
  //Initialize wiringPi
  wiringPiSetup();
  
  //Initialize the servo interface as the input mode
  pinMode(ServoPin, OUTPUT);
  
  int ServoPos = 90;
  servo_pulse(ServoPos);
  
  while(1)
  {
   delay(500);
   servo_control();
  }
  return;
}


