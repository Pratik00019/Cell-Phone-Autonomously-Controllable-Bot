#include <ros.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

/////////////////////////TELEOP PARAMETERS//////////////////////////
#define PWM_MIN 30      /* Minimum PWM value*/
#define PWMRANGE 100   /* Maximum PWM value*/

// Robot model specs
const double L_wheelbase = 0.12; // in m
const double min_speed = 0.20;    // in m/s PER WHEEL
const double max_speed = 0.25;    // Max speed in m/s PER WHEEL
const double radius = 0.03;

uint16_t lPwm;
uint16_t rPwm;

/////////////////////////TELEOP MOTOR PINS/////////////////////////

const int L_DIR = 24;
const int L_PWM = 45;

const int R_DIR = 28;
const int R_PWM = 6;

///////////////////////////////////////////////////////////////

void setupPins();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);
void stop();

/////////////////////////// GLOBAL VARIABLES /////////////////////////// 

float msg_l_x;
float msg_a_z;

/////////////////////////// PID params //////////////////////////

//double kp_l = 2;
//double kp_r = 2;
//double kd_l = 1;
//double kd_r = 1;

//unsigned long curr_t, prev_t;
//double dt;
//double error_l, error_r;
//double lastError_l, lastError_r;
//double Setpoint_l, Setpoint_r;
//
//double rateError_l, rateError_r;
//
//int dc_l, dc_r;
//int count, count1;
//int prev_c_l, prev_c_r;
//double speed_left, speed_right;

///////////////////////////////ROS PARAMETERS////////////////////////////////////////////
ros::NodeHandle nh;
float l, r;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onTwist);
////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  setupPins();

  Serial.begin(115200);
}

///////////////////////////////////////////////////////////////////////////

void setupPins()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);

  pinMode(R_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  stop();
}

void stop()
{
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void onTwist(const geometry_msgs::Twist &msg)
{
  msg_l_x = msg.linear.x;
  msg_a_z = msg.angular.z;

  l = (2 * msg.linear.x - L_wheelbase * msg.angular.z) / (2);
  r = (2 * msg.linear.x + L_wheelbase * msg.angular.z) / (2);

  float xl = abs(l);
  float xr = abs(r);

  lPwm = mapPwm(l, PWM_MIN, PWMRANGE);
  rPwm = mapPwm(r, PWM_MIN, PWMRANGE);

  // Set direction and PWM
  digitalWrite(L_DIR, !(l < 0));
  digitalWrite(R_DIR, !(r < 0));
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}


void loop()
{
  nh.spinOnce();
  delay(10);
}

float mapPwm(float x, float out_min, float out_max) 
{
  x = (x < 0) ? -x : x ;
  return (x == 0) ? 0 : out_max * ((x + min_speed) / max_speed);
}
