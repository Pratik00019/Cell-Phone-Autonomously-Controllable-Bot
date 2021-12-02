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

//void onTwist(const geometry_msgs::Twist &msg)
//{
//  x = msg.linear.x;
//  th = msg.angular.z;
//
//  // Cap values at [-1 .. 1]
//  // float x = max(min(msg.linear.x, 1.0f), -1.0f);
//  // float z = max(min(msg.angular.z, 1.0f), -1.0f);
//
//  // Calculate the intensity of left and right wheels. Simple version.
//  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
//  double l = (2*msg.linear.x - wheelTrack*msg.angular.z) / (2);    
//  double r = (2*msg.linear.x + wheelTrack*msg.angular.z) / (2);
//
//  float xl = abs(l);
//  float xr = abs(r);
//
//  // Then map those values to PWM intensities.
//  // PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
////float  lPwm = (-1.4094*pow(xl,2)) + (107.37*xl) + 1.5459;    // y = f(x)
//// float rPwm = (-1.6446*pow(xr,2)) + (108.64*xr) + 1.7743;
//
//float  lPwm = mapFloat(l, PWM_MIN, PWMRANGE);    // first parenthesis fabs(l)*radius
//float  rPwm = mapFloat(r, PWM_MIN, PWMRANGE);    // first parenthesis fabs(r)*radius
//
//  // Set direction and PWM 
//  digitalWrite(m2_dir, !(l > 0));
//  digitalWrite(m1_dir, !(r > 0));
//  analogWrite(m2_pwm, lPwm);
//  analogWrite(m1_pwm, rPwm);
//}

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
