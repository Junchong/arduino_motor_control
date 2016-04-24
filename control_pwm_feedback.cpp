/* 
 * Use Arduino and rotating encoder to control dc brushless motor
 * majunchong  2016.04.20
 */
#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <MsTimer2.h>                   //定时器库的 头文件

const int encoder_A_pin = 2;    //encoder channel A input
const int encoder_B_pin = 3;    //encoder channel A input
const int led_pin = 13;
const int pwm_pin = 9;                   //use pin-9 to output a annalog voltage to make a fake pwm.

unsigned long en_A_sum=0;
unsigned long en_B_sum=0;
unsigned long en_AB_sum=0;

float voltage_x = 0;
float voltage_y = 0;
float base_vel_x = 0;
float base_vel_y = 0;
float k = 255.0/2500.0;   //for my little dc motor, the Pulse-Width Modulation (PWM) = 255,the velocity of rpm = 2450

int inter_time = 500;  //the period of the timer-interrupt,unit is ms.

float Kp =0.8; //PID-Controler parameters

ros::NodeHandle nh;

geometry_msgs::Point vel;
ros::Publisher pub_velocity("velocity",&vel);

geometry_msgs::Point pwm;

void messageCb(const geometry_msgs::Point& msg)
{
    pwm.x = msg.x;
    pwm.y = msg.y;
} 

ros::Subscriber<geometry_msgs::Point> sub_joy("pwm",&messageCb);

void setup()
{
  nh.initNode();
  nh.advertise(pub_velocity);
  nh.subscribe(sub_joy);
  
  pinMode(led_pin, OUTPUT);
  pinMode(encoder_A_pin, INPUT);
  pinMode(encoder_B_pin, INPUT);
  pinMode(pwm_pin,OUTPUT);

  attachInterrupt(0, inter0, RISING);//当int.0电平Rise时,触发中断函数inter0
  attachInterrupt(1, inter1, RISING);//当int.0电平Rise时,触发中断函数inter1

  //pin 2 for interrupt 0;
  //pin 3 for interrupt 1;
  //LOW                  低电平触发
  //CHANGE            电平变化，高电平变低电平、低电平变高电平
  //RISING              上升沿触发
  //FALLING            下降沿触发
  //HIGH                 高电平触发(该中断模式仅适用于Arduino due)
  

  MsTimer2::set(inter_time, Timer);      // 中断设置函数，每 500ms 进入一次中断
  MsTimer2::start();                    //开始计时
}

void inter0()//中断函数
{
    //en_A_sum++;
    en_AB_sum++;
}

void inter1()//中断函数
{
    //en_B_sum++;
    en_AB_sum++;
}

void Timer()
{
	//vel.x = en_A_sum*2*60/500;  //the rpm of encoder from channel A
	//vel.y = en_B_sum*2*60/500;  //the rpm of encoder from channel B
	vel.z = en_AB_sum*2*60/1000;  

	//en_A_sum = 0;
	//en_B_sum = 0;
	en_AB_sum = 0;
	digitalWrite(led_pin,HIGH);
}

void loop()
{
  digitalWrite(led_pin,LOW);
  
  base_vel_x = k*pwm.x;
  base_vel_y = k*pwm.y;

  voltage_x = (pwm.x - vel.z)*k*Kp + base_vel_x;

  if(voltage_x>255.0) voltage_x=255.0;
  if(voltage_x<=0.0) voltage_x=0.0;

  analogWrite(pwm_pin,voltage_x);

  vel.x = voltage_x;
  vel.y = base_vel_x;

  pub_velocity.publish(&vel);
    
  nh.spinOnce();
}