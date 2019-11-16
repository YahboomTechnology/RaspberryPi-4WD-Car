/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         tcp_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <wiringSerial.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>	       
#include <stdlib.h>
#include <pthread.h>

#define N 1024

#define run_car     '1'
#define back_car    '2'
#define left_car    '3'
#define right_car   '4'
#define stop_car    '0'

#define front_left_servo  '1'
#define front_right_servo '2'
#define up_servo    '3'
#define down_servo  '4'
#define left_servo  '6'
#define right_servo  '7'
#define updowninit_servo '5'
#define stop_servo  '8'


enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;


enum {
  enFRONTSERVOLEFT = 1,
  enFRONTSERVORIGHT,
  enSERVOUP,
  enSERVODOWN,
  enSERVOUPDOWNINIT,
  enSERVOLEFT,
  enSERVORIGHT,
  enSERVOSTOP,
  enSERVOFRONTINIT
} enServoState;


int Left_motor_go = 28;       
int Left_motor_back = 29;     

int Right_motor_go = 24;      
int Right_motor_back = 25;    

int Left_motor_pwm = 27;      
int Right_motor_pwm = 23;     


//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      9                  21                  7                   1
const int TrackSensorLeftPin1  =  9;    
const int TrackSensorLeftPin2  =  21;  
const int TrackSensorRightPin1 =  7;   
const int TrackSensorRightPin2 =  1;   


int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;
char infrared_track_value[5] = {0};


const int AvoidSensorLeft =  26; 
const int AvoidSensorRight = 0;  

int LeftSensorValue ;           
int RightSensorValue ;
char infrared_avoid_value[3] = {0};


const int LdrSensorLeft =  11;   
const int LdrSensorRight = 22;   

int LdrSersorLeftValue ;         
int LdrSersorRightValue ;
char LDR_value[3] = {0};


int buzzer = 10;                


int CarSpeedControl = 150;


int FrontServoPin = 4;
int ServoUpDownPin = 13;
int ServoLeftRightPin = 14;


int ServoUpDownPos = 90;
int ServoLeftRightPos = 90;

int FrontServoLeftRightPos = 90;

int ServoFlags;

int EchoPin = 30;         
int TrigPin = 31;         

int LED_R = 3;           
int LED_G = 2;           
int LED_B = 5;           
int g_lednum = 0;        

int OutfirePin = 8;      

char recvbuf[N] = {0};        
char InputString[N] = {0};    
int NewLineReceived = 0;      
int g_CarState = enSTOP;      
int g_ServoState = enSERVOSTOP;
char ReturnTemp[N] = {0};     
unsigned char g_frontservopos = 90;

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in1]    ServPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    
  PulseWidth = (myangle * 11) + 500; 
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 20; i++)    
  {
    servo_pulse(FrontServoPin, pos); 
  } 
}

/**
* Function       Distance
* @author        Danny
* @date          2017.08.16
* @brief         
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
	digitalWrite(TrigPin, HIGH);      
	delayMicroseconds(10);
	digitalWrite(TrigPin, LOW);

    gettimeofday(&tv3, NULL);        
	start = tv3.tv_sec * 1000000 + tv3.tv_usec;
	while(!digitalRead(EchoPin) == 1)
	{
		gettimeofday(&tv4, NULL);    
		stop = tv4.tv_sec * 1000000 + tv4.tv_usec;
		
		if ((stop - start) > 30000)  
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
	printf("distance: %f\n", distance);
	return distance;
}

/**
* Function       track_test
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void track_test()
{

  TrackSensorLeftValue1 = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2 = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

  infrared_track_value[0] =((TrackSensorLeftValue1 == LOW)? '1' : '0');
  infrared_track_value[1] =((TrackSensorLeftValue2 == LOW)? '1' : '0');
  infrared_track_value[2] =((TrackSensorRightValue1 == LOW)?'1': '0');
  infrared_track_value[3] =((TrackSensorRightValue2 == LOW)? '1': '0');
  return;
}

/**
* Function       infrared_avoid_test
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void infrared_avoid_test()
{ 
  
  LeftSensorValue  = digitalRead(AvoidSensorLeft);
  RightSensorValue = digitalRead(AvoidSensorRight);
  infrared_avoid_value[0]=((LeftSensorValue == LOW) ?'1' : '0');
  infrared_avoid_value[1]=((RightSensorValue == LOW) ? '1' : '0');
  return;
}

/**
* Function       follow_light_test
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void follow_light_test()
{

  LdrSersorRightValue = digitalRead(LdrSensorRight);
  LdrSersorLeftValue  = digitalRead(LdrSensorLeft);

  LDR_value[0]=((LdrSersorLeftValue == LOW) ? '0' :  '1');
  LDR_value[1]=((LdrSersorRightValue == LOW)? '0' :  '1');
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

  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
*  @author       Danny
* @date          2017.08.16
* @brief         
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
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void left()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, 0);

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void right()
{

  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History  
*/
void spin_right()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void back()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void whistle()
{
  digitalWrite(buzzer, LOW);  
  delay(100);                  
  digitalWrite(buzzer, HIGH);  
  delay(1);                    

  digitalWrite(buzzer, LOW);   
  delay(200);                  
  digitalWrite(buzzer, HIGH);  
  delay(2);                    
  return;
}

/**
* Function       servo_up
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_up()
{
    int pos, i;
    pos = ServoUpDownPos;
	servo_pulse(ServoUpDownPin, pos); 
	pos += 2;
	ServoUpDownPos = pos;
	if (ServoUpDownPos >= 180)
	{
		ServoUpDownPos = 180;
	}
}

/**
* Function       servo_down
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_down()
{
    int pos, i;
    pos = ServoUpDownPos;
   	servo_pulse(ServoUpDownPin, pos); 
	pos -= 2;
	ServoUpDownPos = pos;
	if (ServoUpDownPos <= 45)
	{
		ServoUpDownPos = 45;
	}
}

/**
* Function       servo_left
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_left()
{
    int pos, i;
    pos = ServoLeftRightPos;
   	servo_pulse(ServoLeftRightPin, pos); 
	pos += 2;
	ServoLeftRightPos = pos;
	if (ServoLeftRightPos >= 180)
	{
		ServoLeftRightPos = 180;
	}
}

/**
* Function       servo_right
* @author        Danny
* @date          2017.08.16
* @brief        
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_right()
{  
    int pos, i;
    pos = ServoLeftRightPos;
   	servo_pulse(ServoLeftRightPin, pos); 
	pos -= 2;
	ServoLeftRightPos = pos;
	if (ServoLeftRightPos <= 0)
	{
		ServoLeftRightPos = 0;
	}
}

/**
* Function       front_servo_left
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void front_servo_left()
{
    servo_appointed_detection(180);
}

/**
* Function       front_servo_right
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History  
*/
void front_servo_right()
{ 
      servo_appointed_detection(0);
}
/**
* Function       servo_init
* @author        Danny
* @date          2017.08.16
* @brief        
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_init()
{
	int i = 0;
	for (i = 0; i < 10; i++)
	{
	servo_pulse(ServoLeftRightPin, 90); 
	}
	for (i = 0; i < 10; i++)
	{
	servo_pulse(ServoUpDownPin, 90);
	}
	for (i = 0; i < 10; i++)
	{
	servo_pulse(FrontServoPin, 90);
	}
}

/**
* Function       servo_updown_init
* @author        Danny
* @date          2017.08.16
* @brief        
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_updown_init()
{
	int i = 0;
	for (i = 0; i < 10; i++)
	{
		servo_pulse(ServoUpDownPin, 90);
	}
}

/**
* Function       servo_front_init
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_front_init()
{
	servo_pulse(FrontServoPin, 90);
}

/**
* Function       servo_stop
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_stop()
{
}

/**
* Function       color_led_pwm
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in1]    v_iRed（0-255）
* @param[in2]    v_iGreen（0-255）
* @param[in3]    v_iBlue（0-255）
* @param[out]    void
* @retval        void
* @par History   
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  softPwmWrite(LED_R, v_iRed);
  softPwmWrite(LED_G, v_iGreen);
  softPwmWrite(LED_B, v_iBlue);
  return;
}

/**
* Function       StringFind
* @author        Danny
* @date          2017.08.16    
* @brief        
* @param[in]     pSrc; pDst:v_iStartPos
* @param[out]    void
* @retval        void
* @par History   
*/
int StringFind(const char *pSrc, const char *pDst, int v_iStartPos)  
{  
    int i, j;  
    for (i = v_iStartPos; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j] !='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
} 

/**
* Function       tcp_data_parse
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void tcp_data_parse()
{
  	if (StringFind((const char *)InputString, (const char *)"PTZ", 0) > 0)
	{
		int m_kp, i, ii;
        
		i = StringFind((const char *)InputString, (const char *)"PTZ", 0); 
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			char m_skp[5] = {0};
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			
			m_kp = atoi(m_skp);        

			g_frontservopos = 180 - m_kp;
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
  	}

  	 if (StringFind((const char *)InputString, (const char *)"CLR", 0) > 0)
 	{
		int m_kp, i, ii, red, green, blue;
		char m_skp[5] = {0};
		i = StringFind((const char *)InputString, (const char *)"CLR", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{			
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			red =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLG", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			green =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLB", 0);
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			blue =  m_kp;
			color_led_pwm(red, green, blue);
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
	}


  if (StringFind((const char *)InputString, (const char *)"4WD", 0) == -1 &&
		                    StringFind((const char *)InputString,(const char *)"#",0) > 0)
  {

    if (InputString[3] == '1')      
    {
      g_CarState = enTLEFT;
    }
    else if (InputString[3] == '2') 
    {
      g_CarState = enTRIGHT;
    }
    else
    {
      g_CarState = enSTOP;
    }

    if (InputString[5] == '1')     
    {
      whistle();
    }

    if (InputString[7] == '1')     
    {
      CarSpeedControl += 50;
      if (CarSpeedControl > 255)
      {
        CarSpeedControl = 255;
      }
    }
    if (InputString[7] == '2')    
    {
      CarSpeedControl -= 50;
      if (CarSpeedControl < 50)
      {
        CarSpeedControl = 50;
      }
    }
	
    if (InputString[13] == '1')
   
    {
		g_lednum++;
        if(g_lednum == 1){
		color_led_pwm(255, 255, 255);
		}
		else if(g_lednum == 2)
		{
			color_led_pwm(255,0,0);
		}
		else if(g_lednum == 3)
        {
			color_led_pwm(0,255,0);
		}
		else if(g_lednum == 4)
		{
			color_led_pwm(0,0,255);
		}
		else if(g_lednum == 5)
		{
			color_led_pwm(255,255,0);
		}
		else if(g_lednum == 6)
		{
			color_led_pwm(0,255,255);
		}
		else if(g_lednum == 7)
		{
			color_led_pwm(255,0,255);
		}
		else{
		   color_led_pwm(0,0,0);
			   g_lednum=0;
		}
    }
    if (InputString[13] == '2')
    {
      color_led_pwm(255, 0, 0);
    }
    if (InputString[13] == '3')
    {
      color_led_pwm(0, 255, 0);
    }
    if (InputString[13] == '4') 
    {
      color_led_pwm(0, 0, 255);
    }

    if (InputString[15] == '1')  
    {
      digitalWrite(OutfirePin, !digitalRead(OutfirePin));
    }

    if (InputString[17] == '1') 
    {
		g_frontservopos = 90; 
	}

    if (g_CarState != enTLEFT && g_CarState != enTRIGHT)
    {
      switch (InputString[1])
      {
        case run_car:   g_CarState = enRUN;  break;
        case back_car:  g_CarState = enBACK;  break;
        case left_car:  g_CarState = enLEFT;  break;
        case right_car: g_CarState = enRIGHT;  break;
        case stop_car:  g_CarState = enSTOP;  break;
        default: g_CarState = enSTOP; break;
      }
    }


	switch(InputString[9])
	{
		case front_left_servo: g_frontservopos = 180 ;break;
		case front_right_servo: g_frontservopos = 0;break;
	    case up_servo:  g_ServoState = enSERVOUP; break;
		case down_servo: g_ServoState = enSERVODOWN;break;
		case left_servo: g_ServoState = enSERVOLEFT;break;
		case right_servo: g_ServoState = enSERVORIGHT;break;
		case updowninit_servo: g_ServoState = enSERVOUPDOWNINIT;break;
		case stop_servo: g_ServoState = enSERVOSTOP;break;
	}

    memset(InputString, 0x00, sizeof(InputString));       
    NewLineReceived = 0;

    switch (g_CarState)
    {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
      default: brake(); break;
    }
  }
}

/**
* Function       itoa
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @retval        void
* @par History   
*/
void itoa(int i, char *string)
{
	sprintf(string,"%d", i);
	return;
}

/**
* Function       tcp_data_postback
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @retval        void
* @par History  
*/
char *tcp_data_postback()
{

  char *p= ReturnTemp;
  char str[25];
  float distance;
  memset(ReturnTemp,0,sizeof(ReturnTemp));

  distance = Distance();
  if((int)distance == -1)
  {
	  distance = 0;
  }
  strcat(p, "$4WD,CSB");
  itoa((int)distance, str);
  strcat(p, str);

  strcat(p, ",PV8.4");

  strcat(p, ",GS0");

  strcat(p, ",LF");
  track_test();
  strcat(p,infrared_track_value);

  strcat(p, ",HW");
  infrared_avoid_test();
  strcat(p,infrared_avoid_value);

  strcat(p, ",GM");
  follow_light_test();
  strcat(p, LDR_value);
  strcat(p, "#");
  //printf("ReturnTemp_first:%s\n",p);
  return ReturnTemp;
}


/**
* Function       Data_Pack
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void Data_Pack()
{ 
   if(recvbuf[0] == '$' && StringFind((const char *)recvbuf, (const char *)"#", 0) > 0)
   {
	    strcpy(InputString,recvbuf); 
		//printf("InputString:%s\n",InputString);
		NewLineReceived = 1;
   }
   else
   {
		NewLineReceived = 0;
   }
   return;
}

/**
* Function       do_client_recv
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void *do_client_recv(void *arg)
{
	int n = 0;
	int sockfd = *(int *)arg;

	while(1)
	{
		memset(recvbuf,0,sizeof(recvbuf));
		n = recv(sockfd,recvbuf,sizeof(recvbuf),0);	
		if(n < 0)
		{
			perror("Fail to recv!\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}else if(n == 0){
			printf("clinet_recv is not connect\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}		
		printf("Recv %d bytes : %s\n",n,recvbuf);
	
		Data_Pack();
		if (NewLineReceived == 1)
		{
			tcp_data_parse();
		}
	}
	close(sockfd);
	free(arg);
	pthread_exit(NULL);
}

void *servo_control()
{
	int i_ServoState = 0;
	int i_frontservopos = 0;
    while(1)
	{
		
	   switch (g_ServoState)
	   {
	    case enSERVOUP: 			
			servo_up();
			break;
        case enSERVODOWN: 			
			servo_down();
			break;
	    case enSERVOLEFT:			
			servo_left();
			break;
	    case enSERVORIGHT: 			
			servo_right();
	   		break;
	    case enSERVOUPDOWNINIT:
			servo_updown_init();
			break;
	    case enSERVOSTOP: 			
			servo_stop();
			break;
	   
	   }
		if (g_frontservopos != i_frontservopos)
		{
			servo_appointed_detection(g_frontservopos);
			i_frontservopos = g_frontservopos;
		}
      sleep(1);
	}
    
	 pthread_exit(NULL);
}

/**
* Function       do_client_postback
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void *do_client_postback(void *arg)
{
	int n = 0;
	int sockfd = *(int *)arg;
	char str[1024] ={0};
	char *pstr = str;
	while(1)
	{   
        
        memset(str,0,sizeof(str));
		strcpy(str,tcp_data_postback());
		puts(str);
	
		n = send(sockfd,str,strlen(str),0);	
		if(n < 0)
		{
			perror("Fail to send!\n");	
	        break;
			//	exit(EXIT_FAILURE);
		}else if(n == 0){
			printf("clinet_postback is not connect\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}
        printf("send %d bytes : %s\n",n,str);	
        sleep(4000);		
	}
//	close(sockfd);
//	free(arg);//释放堆区空间
	pthread_exit(NULL);
}
/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in1]    
* @param[in2]    ip
* @param[in3]    port
* @retval        void
* @par History   
*/
int main(int argc, const char *argv[])
{
   char buf[1024] = {0};
   
   int listen_fd = 0;
   
   int connect_fd = 0;
   
   struct sockaddr_in my_addr;
   
   struct sockaddr_in client_addr;
  
   int len = sizeof(my_addr);
   int n = 0 ;
   int *pconnect_fd = NULL;
  
   pthread_t tid1;
   pthread_t tid2;
   pthread_t tid3;
   int ret = 0;
   
 
   wiringPiSetup();

  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  

  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);


  pinMode(AvoidSensorLeft, INPUT);
  pinMode(AvoidSensorRight, INPUT);
  
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);
  
  pinMode(LdrSensorLeft, INPUT);
  pinMode(LdrSensorRight, INPUT);
  
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  

  pinMode(EchoPin, INPUT);   
  pinMode(TrigPin, OUTPUT);  

  pinMode(OutfirePin, OUTPUT);
  digitalWrite(OutfirePin,HIGH);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  softPwmCreate(LED_R,0,255); 
  softPwmCreate(LED_G,0,255); 
  softPwmCreate(LED_B,0,255); 

  pinMode(FrontServoPin, OUTPUT);
  pinMode(ServoUpDownPin, OUTPUT);
  pinMode(ServoLeftRightPin, OUTPUT);
  

  servo_init();
  

	if(argc < 1)
	{
		fprintf(stderr,"Usage : %s ip port!\n",argv[0]);	
		exit(EXIT_FAILURE);
	}

	//1.Create a listening socket through a socket
	listen_fd = socket(AF_INET,SOCK_STREAM,0);
	if(listen_fd < 0)
	{
		perror("Fail to socket");	
		exit(EXIT_FAILURE);
	}

	//2. Fill the server's ip address and port
    //Note: This is populated with the IP address of our Raspberry Pi and the port we specified.
    // The ip address is subject to your own machine!
	memset(&my_addr,0,sizeof(my_addr));	
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(atoi("8888"));
	my_addr.sin_addr.s_addr = inet_addr("192.168.50.1");

	//3.Bind ip and port
	if(bind(listen_fd,(struct sockaddr *)&my_addr,len) < 0)
	{
		perror("Fail to bind");	
		exit(EXIT_FAILURE);
	}

	//4.Listen for client connections
	listen(listen_fd,5);
	printf("Listen....\n");
	
	while(1)
	{
		
		pconnect_fd = (int *)malloc(sizeof(int));

		
		*pconnect_fd = accept(listen_fd,(struct sockaddr *)&client_addr,&len);		
		if(*pconnect_fd < 0)
		{
			perror("Fail to accept");	
			exit(EXIT_FAILURE);
		}

		printf("=============================================\n");
		printf("connect_fd : %d\n",*pconnect_fd);
		printf("client IP : %s\n",inet_ntoa(client_addr.sin_addr));
		printf("client port : %d\n", ntohs(client_addr.sin_port));

		ret = pthread_create(&tid1,NULL,do_client_recv,(void *)pconnect_fd);
		
		
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
				
		ret = pthread_create(&tid3,NULL,servo_control,NULL);
		
		
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
		ret = pthread_create(&tid2,NULL,do_client_postback,(void *)pconnect_fd);
		
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
		
		pthread_detach(tid1);
		pthread_detach(tid3);
		pthread_detach(tid2);
	}
	close(listen_fd);
    exit(EXIT_SUCCESS);
}




