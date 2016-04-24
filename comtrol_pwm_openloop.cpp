/* 
 * Use Arduino and rotating encoder to control dc brushless motor
 * majunchong  2016.04.20
 */
#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

const int led_pin = 13;
const int pwm_pin = 9;                   //use pin-9 to output a annalog voltage to make a fake pwm.

float voltage_x = 0;
float voltage_y = 0;

ros::NodeHandle nh;

geometry_msgs::Point voltage;
ros::Publisher pub_velocity("voltage",&voltage);

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
  pinMode(pwm_pin,OUTPUT);
}

void loop()
{
  voltage_x = pwm.x;

  if(voltage_x>255.0) voltage_x=255.0;
  if(voltage_x<=0.0) voltage_x=0.0;

  analogWrite(pwm_pin,voltage_x);
  analogWrite(led_pin,voltage_x);

  voltage.x = voltage_x;

  pub_velocity.publish(&voltage);
    
  nh.spinOnce();
}
