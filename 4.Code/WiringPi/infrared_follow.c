/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         infrared_follow.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        红外跟随实验
* @details
* @par History  见如下说明
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

const int FollowSensorLeft =  26;  //Obstacle avoidance infrared sensor pin on the left is connected to  wiringPi port 26 of Raspberry pi
const int FollowSensorRight = 0;   //Obstacle avoidance infrared sensor pin on the right is connected to  wiringPi port 0 of Raspberry pi


int LeftSensorValue ;              //Define variables to store data collected by each Obstacle avoidance infrared sensor 
int RightSensorValue ;


/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         小车前进
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void run()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         小车刹车
* @param[in]     void 
* @param[out]    void
* @retval        void
* @par History   无
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
* @brief         小车左转(左轮不动，右轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void left()
{
  //左电机停止
  digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 0);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 100);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         小车右转(右轮不动，左轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void right()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 100);

  //右电机停止
  digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, 150);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         小车后退
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);       //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);    //左电机后退使能
  softPwmWrite(Left_motor_pwm, 150);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);     //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH);  //右电机后退使能
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       key_scan
* @author        Danny
* @date          2017.08.16
* @brief         按键检测(包含软件按键去抖)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void key_scan()
{
  while (digitalRead(key));       //当按键没有被按下一直循环
  while (!digitalRead(key))       //当按键被按下时
  {
    delay(10);	                  //延时10ms
    if (digitalRead(key)  ==  LOW)//第二次判断按键是否被按下
    {
      delay(100);
      while (!digitalRead(key));  //判断按键是否被松开
    }
  }
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先调用按键扫描函数，接着红外避障模式开启
* @param[in]     void
* @retval        void
* @par History   无
*/
void main()
{ 
  //wiringPi初始化
  wiringPiSetup();
  
  //初始化电机驱动IO口为输出方式
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //创建两个软件控制的PWM脚
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);

  //定义按键接口为输入接口
  pinMode(key, INPUT);

  //定义左右传感器为输入接口
  pinMode(FollowSensorLeft, INPUT);
  pinMode(FollowSensorRight, INPUT);

  //调用按键扫描函数
  key_scan(); 
  
  while(1)
  {
    //遇到跟随物,红外跟随模块的指示灯亮,端口电平为LOW
    //未遇到跟随物,红外跟随模块的指示灯灭,端口电平为HIGH
    LeftSensorValue  = digitalRead(FollowSensorLeft);
    RightSensorValue = digitalRead(FollowSensorRight);

    if (LeftSensorValue == LOW && RightSensorValue == LOW)
    {
      run();        //当两侧均检测到跟随物时调用前进函数
    }
    else if (LeftSensorValue == LOW && RightSensorValue == HIGH)
    {
      spin_left(2); //左边探测到有跟随物，有信号返回，原地向左转
    }
    else if (RightSensorValue == LOW && LeftSensorValue == HIGH)
    {
      spin_right(2);//右边探测到有跟随物，有信号返回，原地向右转
    }
    else if (LeftSensorValue == HIGH && RightSensorValue == HIGH)
    {
      brake();     //当两侧均未检测到跟随物时停止
    }
  }
  return;
}
