/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         ServoControlColor.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        Servo_Control_Color
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>

#define ON  1   
#define OFF 0   

//Definition of Pin
int LED_R = 3;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
int LED_G = 2;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
int LED_B = 5;           //LED_B is connected to  wiringPi port 5 of Raspberry pi

//Define the servo pin
int ServoPin = 4;

void corlor_light(int);
void corlor_led(int, int, int);

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
void servo_control_color()
{
  int pos = 0;
  for (pos = 0; pos < 180; pos++)
  {
    servo_pulse(pos);
    corlor_light(pos);
    delay(20);
  }

  for (pos = 180; pos > 0; pos--)
  {
    servo_pulse(pos);
    corlor_light(pos);
    delay(20);
  }
  return;
}

/**
* Function       corlor_light
* @author        Danny
* @date          2017.08.16
* @brief         According to the angle of rotation ,light up the corresponding color 
* @param[in]     pos 
* @param[out]    void
* @retval        void
* @par History   
*/
void corlor_light(int pos)
{
  if (pos > 150)
  {
    corlor_led(ON, OFF, OFF);
  }

  else if (pos > 125)
  {
    corlor_led(OFF, ON, OFF);
  }

  else if (pos > 100)
  {
    corlor_led(OFF, OFF, ON);
  }

  else if (pos > 75)
  {
    corlor_led(OFF, ON, ON);
  }

  else if (pos > 50)
  {
    corlor_led(ON, ON, OFF);
  }

  else if (pos > 25)
  {
    corlor_led(ON, OFF, ON);
  }

  else if (pos > 0)
  {
    corlor_led(ON, ON, ON);
  }
  else
  {
    corlor_led(OFF, OFF, OFF);
  }
}

/**
* Function        corlor_led
* @author         Danny
* @date           2017.08.16
* @brief          7 different colors formed by different combinations of R,G and B
* @param[in1]     Red
* @param[in2]     Green
* @param[in3]     Blue
* @retval         void
* @par History    
*/
void corlor_led(int v_iRed, int v_iGreen, int v_iBlue)
{
  v_iRed == ON ? digitalWrite(LED_R, HIGH): digitalWrite(LED_R, LOW);
 
  v_iGreen == ON ? digitalWrite(LED_G, HIGH) : digitalWrite(LED_G, LOW);
  
  v_iBlue == ON ? digitalWrite(LED_B, HIGH) : digitalWrite(LED_B, LOW);
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         Delay 0.5 s, Servo_control_color mode is opened
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
  
  //Initialize the RGB IO as the output mode
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  int ServoPos = 90;
  servo_pulse(ServoPos);
  
  while(1)
  {
   delay(500);
   servo_control_color();
  }
  return;
}


