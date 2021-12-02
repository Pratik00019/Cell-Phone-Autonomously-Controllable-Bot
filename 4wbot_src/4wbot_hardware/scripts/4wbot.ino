#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PWM_MIN  0
#define PWMRANGE  255
#define x_min  0
#define x_max  0.6


ros::NodeHandle nh;

float u1, u2;
float pwm1, pwm2;
float x, y, th;

int m1_pwm = 45;
int m1_dir = 24;
int m2_pwm = 6;
int m2_dir = 28;
float wheelTrack = 0.012;

void onTwist(const geometry_msgs::Twist &msg);
float mapFloat(float x1, float out_min, float out_max);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
void onTwist(const geometry_msgs::Twist &msg)
{

  x = msg.linear.x;
  th = -msg.angular.z;

  if (x == 0) {
    // Turn in place
    u1 = (th * wheelTrack / 2.0) * 40 ;  //u1 =right
    u2 = -u1;
  }
  else if (th == 0) {
    u1 = u2 = x;
  }
  else {
    // Rotation about a point in space
    u2 = x - th * wheelTrack / 2.0;
    u1 = x + th * wheelTrack / 2.0;
  }


  pwm2 = mapFloat(fabs(u2), PWM_MIN, PWMRANGE);//assuming u2 as left
  pwm1 = mapFloat(fabs(u1), PWM_MIN, PWMRANGE);//assuming u1 as right

  // Set direction pins and PWM
  if (u1 < 0)
  {
    digitalWrite(m1_dir, LOW);
    analogWrite(m1_pwm, pwm1);


  }
  else
  {
    digitalWrite(m1_dir, HIGH);
    analogWrite(m1_pwm, pwm1);

  }
  if (u2 < 0)
  {
    digitalWrite(m2_dir, LOW);
    analogWrite(m2_pwm, pwm2);


  }
  else
  {
    digitalWrite(m2_dir, HIGH);
    analogWrite(m2_pwm, pwm2);

  }


}


float mapFloat(float x1, float out_min, float out_max)
{

  return min(out_min + ((x1 - x_min) / (x_max - x_min)) * (out_max - out_min), out_max);

}


void setup() {
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_dir, OUTPUT);
  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);

  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub);
}
void loop()
{
  nh.spinOnce();
}
