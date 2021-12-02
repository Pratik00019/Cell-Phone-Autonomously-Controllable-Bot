#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PWM_MIN  0
#define PWMRANGE  255
#define d  353.4
#define x_min  0
#define x_max  1.13


ros::NodeHandle nh;

float u1, u2;
float pwm1, pwm2;
float x,y,z;

int m1_pwm = 6;
int m1_dir = 26;
int m2_pwm = 7;
int m2_dir = 30;

void onTwist(const geometry_msgs::Twist &msg);
float mapFloat(float x1, float out_min, float out_max);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
void onTwist(const geometry_msgs::Twist &msg)
{

    x = msg.linear.x;
    y = msg.linear.y;
    z = msg.angular.z;

    u1= 2.5 * (x +y)*cos(z);
    u2= 2.5 * (x +y)*sin(z);
   

    pwm2 = mapFloat(fabs(u2), PWM_MIN, PWMRANGE);//assuming u2 as left
    pwm1 = mapFloat(fabs(u1), PWM_MIN, PWMRANGE);//assuming u3 as right

    // Set direction pins and PWM
    if (u1 < 0)
    {
        analogWrite(m1_pwm, pwm1);
        digitalWrite(m1_dir, LOW);

    }
    else
    {
        analogWrite(m1_pwm, pwm1);
        digitalWrite(m1_dir, HIGH);
    };
    if (u2 < 0)
    {
        analogWrite(m2_pwm, pwm2);
        digitalWrite(m2_dir, LOW);

    }
    else
    {
        analogWrite(m2_pwm, pwm2);
        digitalWrite(m2_dir, HIGH);
    }


}

float mapFloat(float x1, float out_min, float out_max)
{

    return min(out_min + ((x1-x_min)/(x_max-x_min))*(out_max-out_min), out_max);

}


void setup() {
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub);
}
void loop()
{
 nh.spinOnce();
}
