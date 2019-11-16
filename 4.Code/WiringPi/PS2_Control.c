/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         PS2_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        PS2 Handle control 4WD car
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wiringPiSPI.h>

/*Define Handle button*/
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

/*Define PS2 pins*/
#define PS2_DAT_PIN   12//MOS
#define PS2_CMD_PIN   13//MIS
#define PS2_SEL_PIN   6 //CS
#define PS2_CLK_PIN   14//SCK

/*The last 4 data sent back are the data of the rocker.*/
#define PSS_RX 5        //Right rocker X-axis data
#define PSS_RY 6        //Right rocker Y-axis data
#define PSS_LX 7        //left rocker X-axis data
#define PSS_LY 8        //Left rocker Y-axis data

/*Car running status enumeration*/
enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT
}enCarState;

//Define pins
int Left_motor_go = 28;       //Left motor forward AIN1 connects Raspberry's wiringPi code 28 port
int Left_motor_back = 29;     //Left motor back AIN2 connects Raspberry's wiringPi code 29 port

int Right_motor_go = 24;      //Right motor forward BIN1 connects Raspberry's wiringPi code 24 port
int Right_motor_back = 25;    //Right motor back BIN2 connects Raspberry's wiringPi code 25 port

int Left_motor_pwm = 27;      //Left motor speed control PWMA connects Raspberry's wiringPi code 27 port
int Right_motor_pwm = 23;     //Right motor speed control PWMB connects Raspberry's wiringPi code 23 port

int buzzer = 10;              

/*Car initial speed control*/
int CarSpeedControl = 150;

/*Define servo pins*/
int ServoPin = 4;

/*Car motion state and steering gear motion status flag*/
int g_CarState = enSTOP;  //1advance 2back 3left 4right 0stop
int g_ServoState = 0;     //1Left shake 2 Right shake

int LED_R = 3;           //LED_R connect Raspberry's wiringPi code 3 port
int LED_G = 2;           //LED_G connect Raspberry's wiringPi code 2 port
int LED_B = 5;           //LED_B connect Raspberry's wiringPi code 5 port

unsigned char PS2_KEY;
unsigned char X1,Y1,X2,Y2; 

//Button value  
unsigned short MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
	};	
	
unsigned short  Handkey;
unsigned char scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         Define a pulse function to generate PWM values in an analog manner
* 				The time base pulse is 20ms, and the high level of the pulse is 0.5-2.5ms.
* 				Control 0-180 degrees
* @param[in1]    ServPin:Servo control pin
* @param[in2]    myangle:Servo rotates at a specified angle
* @param[out]    void
* @retval        void
* @par History   no
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    
  PulseWidth = (myangle * 11) + 500; //Convert the angle to a pulse width of 500-2480
  digitalWrite(ServoPin, HIGH);      //Set the servo interface level high
  delayMicroseconds(PulseWidth);     //Microseconds of the delay pulse width value
  digitalWrite(ServoPin, LOW);       //Set the servo interface level LOW
  delay(20 - PulseWidth / 1000);     //Remaining time in the delay period
  return;
}

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void run()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         brake
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
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
* @brief         Turn left(Left wheel stop,right wheel advance)
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
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         Left wheel back，Right wheel advance
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void spin_left()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       upleft
* @author        Danny
* @date          2017.08.16
* @brief         (Left wheel advance,Right wheel advance. There is a difference between the two )
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void upleft()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30);     

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);   
}

/**
* Function       downleft
* @author        Danny
* @date          2017.08.16
* @brief         (Left wheel back,Right wheel back. There is a difference between the two )
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void downleft()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);        
  digitalWrite(Left_motor_back, HIGH);      
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);        
  digitalWrite(Right_motor_back, HIGH);     
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);
}

/**
* Function       upright
* @author        Danny
* @date          2017.08.16
* @brief         (Left wheel advance,Right wheel advance. There is a difference between the two )
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void upright()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);   
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);    
}

/**
* Function       downright
* @author        Danny
* @date          2017.08.16
* @brief         (Left wheel back,Right wheel back. There is a difference between the two )
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void downright()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     

  //Right motor back
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, HIGH);
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);   
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         (Left wheel advance, right wheel stop)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         (Right wheel stop，Left wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void spin_right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         back
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void back()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH);
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.08.16
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void whistle()
{
  digitalWrite(buzzer, LOW);   //sound
  delay(100);                 
  digitalWrite(buzzer, HIGH);  //no sound
  delay(1);                  

  digitalWrite(buzzer, LOW);   //sound
  delay(200);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(2);                    
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     pos: angle
* @param[out]    void
* @retval        void
* @par History   no
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 15; i++)    
  {
    servo_pulse(ServoPin, pos); 
  }
}

/**
* Function       color_led_pwm
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in1]    v_iRed:（0-255）
* @param[in2]    v_iGreen:（0-255）
* @param[in3]    v_iBlue:（0-255）
* @param[out]    void
* @retval        void
* @par History   no
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  softPwmWrite(LED_R, v_iRed);
  softPwmWrite(LED_G, v_iGreen);
  softPwmWrite(LED_B, v_iBlue);
  return;
}

/**
* Function       PS2_Init
* @author        Danny
* @date          2017.08.16
* @brief         PS2 initialization
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void PS2_Init()
{
	//Initialize an SPI channel, 0 for SPI channel 0, and 500000 for SPI clock speed
	wiringPiSPISetup(0,500000);
	pinMode(PS2_CMD_PIN, OUTPUT);
	pinMode(PS2_CLK_PIN, OUTPUT);
	pinMode(PS2_DAT_PIN, INPUT);
	pinMode(PS2_SEL_PIN, OUTPUT);
	
	digitalWrite(PS2_CLK_PIN,HIGH);
	digitalWrite(PS2_SEL_PIN,HIGH);
	digitalWrite(PS2_CMD_PIN,HIGH);
}

/**
* Function       PS2_AnologData
* @author        Danny
* @date          2017.08.16
* @brief         Read PS2 Anolog Data
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
//PS2 Anolog value range：0~255
unsigned char PS2_AnologData(unsigned char button)
{
	return Data[button];
}

/**
* Function       PS2_ClearData
* @author        Danny
* @date          2017.08.16
* @brief         Clear PS2 Data
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void PS2_ClearData()
{
	memset(Data, 0 ,sizeof(Data)/sizeof(Data[0]));
	return;
}

/**
* Function       PS2_ReadData
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     command:
* @param[out]    void
* @retval        void
* @par History   no
*/
unsigned char PS2_ReadData(unsigned char command)
{
	unsigned char i,j = 1;
	unsigned char res = 0; 
    for(i=0; i<=7; i++)          
    {
		if(command&0x01)
		digitalWrite(PS2_CMD_PIN,HIGH);
		else
		digitalWrite(PS2_CMD_PIN,LOW);
		command = command >> 1;
		delayMicroseconds(10);
		digitalWrite(PS2_CLK_PIN,LOW);
		delayMicroseconds(10);
		if(digitalRead(PS2_DAT_PIN) == HIGH) 
			res = res + j;
		j = j << 1; 
		digitalWrite(PS2_CLK_PIN,HIGH);
		delayMicroseconds(10);	 
    }
    digitalWrite(PS2_CMD_PIN,HIGH);
	delayMicroseconds(50);
 //   printf("res:%d\n",res);	
    return res;	
}

/**
* Function       PS2_DataKey
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/

// Process the data of the read PS2
// Pressed as 0, not pressed as 1
unsigned char PS2_DataKey()
{
	unsigned char index = 0, i = 0;
   	PS2_ClearData();
	digitalWrite(PS2_SEL_PIN,LOW);
	for(i=0;i<9;i++)	//Update scan key of PS2 handle
	{
		Data[i] = PS2_ReadData(scan[i]);
//		printf("data[%d]:%d\n",i,Data[i]);	
	} 
	digitalWrite(PS2_SEL_PIN,HIGH);
	
	Handkey=(Data[4]<<8)|Data[3];     
	for(index=0;index<16;index++)
	{	 
		if((Handkey&(1<<(MASK[index]-1)))==0)
		{
			return index+1;
		}
//		printf("index:%d\n",index+1);
	}
	return 0;      //No any button key
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         Parsing the instructions sent by the handle and executing the corresponding instructions
* @param[in]     void
* @retval        void
* @par History   no
*/
void main()
{
	//wiringPi Initialization
	wiringPiSetup();
	//PS2 Initialization
    PS2_Init();
	
    //Initialize motor drive IO as output mode
    pinMode(Left_motor_go, OUTPUT);
    pinMode(Left_motor_back, OUTPUT);
    pinMode(Right_motor_go, OUTPUT);
    pinMode(Right_motor_back, OUTPUT);
 
    softPwmCreate(Left_motor_pwm,0,255); 
    softPwmCreate(Right_motor_pwm,0,255);
  
    //Initialize Buzzer IO as output mode
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, HIGH);
 
    //Initialize RGB IO as output mode
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    softPwmCreate(LED_R,0,255); 
    softPwmCreate(LED_G,0,255); 
    softPwmCreate(LED_B,0,255);
  
    //Initialize Servo IO as output mode
    pinMode(ServoPin, OUTPUT);
	
    while(1)
    {	   
      int flag = 0;
      PS2_KEY = PS2_DataKey();	 
	  switch(PS2_KEY)
	  {
		//select key press
	    case PSB_SELECT: 	puts("PSB_SELECT");  break;
		//press L3 key,stop
	    case PSB_L3:     	g_CarState = enSTOP;  puts("PSB_L3");  break; 
        //press R3 key,Servo rotate 90°		
	    case PSB_R3:     	servo_appointed_detection(90);	puts("PSB_R3");  break; 
        //press start key,		
	    case PSB_START:  	puts("PSB_START");  break;  
		//press UP key,Car advance
	    case PSB_PAD_UP: 	g_CarState = enRUN;   puts("PSB_PAD_UP");  break; 
        //press Right key, car turn right		
	    case PSB_PAD_RIGHT:	g_CarState = enRIGHT; puts("PSB_PAD_RIGHT");  break;
		//press Down key, car back	
	    case PSB_PAD_DOWN:	g_CarState = enBACK;  puts("PSB_PAD_DOWN");  break;
        //press Left key, car turn left	
	    case PSB_PAD_LEFT:	g_CarState = enLEFT;  puts("PSB_PAD_LEFT");  break; 
		//press L2 key, add speed
	    case PSB_L2:      	CarSpeedControl += 50;
                            if (CarSpeedControl > 255)
                            {
                              CarSpeedControl = 255;
                            }
						    puts("PSB_L2");  break; 
		//press R2 key, decelerate speed
	    case PSB_R2:      	CarSpeedControl -= 50;
                            if (CarSpeedControl < 50)
                            {
                              CarSpeedControl = 100;
                            }
						    puts("PSB_R2");  break; 
		//press L1 key
	    case PSB_L1:      	puts("PSB_L1");  break; 
		//press R1 key
	    case PSB_R1:      	puts("PSB_R1");  break; 
        //press triangle key, Green light		
	    case PSB_TRIANGLE:	color_led_pwm(0, 255, 0);
                            delay(100);
						    puts("PSB_TRIANGLE");  break; 
		//press circle key, Blue light
	    case PSB_CIRCLE:  	color_led_pwm(0, 0, 255);
                            delay(100);
					        puts("PSB_CIRCLE");  break; 
	    
	    //press square key, Red light
	    case PSB_SQUARE:  	color_led_pwm(255, 0, 0);
                            delay(100);
						    puts("PSB_SQUARE");  break;

		//press cross key, buzzer whistle
		case PSB_CROSS:		whistle();
							color_led_pwm(0, 0, 0);break;
							
	    default: g_CarState = enSTOP; break; 
	   }
      
	  //Read the analog value of the rokcer data when L1 or R1 is pressed
	  if(PS2_KEY == PSB_L1 || PS2_KEY == PSB_R1)
	  {
		X1 = PS2_AnologData(PSS_LX);
		Y1 = PS2_AnologData(PSS_LY);
		X2 = PS2_AnologData(PSS_RX);
		Y2 = PS2_AnologData(PSS_RY);
		
        /*Left rocker control car*/		
	    if (Y1 < 5 && X1 > 80 && X1 < 180) 
		{
		    g_CarState = enRUN;		//advance
		}
		else if (Y1 > 230 && X1 > 80 && X1 < 180) 
	    {
			g_CarState = enBACK;	//back		  
		}
		else if (X1 < 5 && Y1 > 80 && Y1 < 180) 
		{
			g_CarState = enLEFT;    //turn left
		}
	    else if (Y1 > 80 && Y1 < 180 && X1 > 230)
		{
			g_CarState = enRIGHT;   //turn right
	    }
	    else if (Y1 <= 80 && X1 <= 80) 
		{
		    g_CarState = enUPLEFT;  //upleft
		}
		else if (Y1 <= 80 && X1 >= 180) 
		{
			g_CarState = enUPRIGHT;	//upright
		}
	    else if (X1 <= 80 && Y1 >= 180) 
		{
			g_CarState = enDOWNLEFT;//downleft	
		}
	    else if (Y1 >= 180 && X1 >= 180) 
		{
			g_CarState = enDOWNRIGHT;//downright		  
		}
	    else
		{
			g_CarState = enSTOP;     //stop
	    }

		/*Right rocker control servo*/
	    if (X2 < 5 && Y2 > 110 && Y2 < 150) 
	    {
		    g_ServoState = 1;        //servo rotate left
		}
		else if (Y2 > 110 && Y2 < 150 && X2 > 230)
		{
		    g_ServoState = 2;        //servo rotate right
		}
		else
		{
		    g_ServoState = 0;        //servo reset
		}
	  }
	
     //Flag is used as a state machine
     switch (g_ServoState)
     {
       case 0: if (flag != 0) {
               flag = 0;
               servo_appointed_detection(90);
               } break;
       case 1: if (flag != 1) {
               flag = 1;
               servo_appointed_detection(180);
               } break;
       case 2: if (flag != 2) {
               flag = 2;
               servo_appointed_detection(0);
               } break;
       default: break;
     }
	 
	 //Car movement state judgment
     switch (g_CarState)
     {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
	  case enUPLEFT: upleft();break;
	  case enUPRIGHT: upright();break;
	  case enDOWNLEFT:downleft();break;
	  case enDOWNRIGHT:downright();break;
      default: brake(); break;
     }
	 
     //In order to avoid the frequent restart caused by sending the handle command too frequently, we need to delay
     delay(50);	
   }
} 
