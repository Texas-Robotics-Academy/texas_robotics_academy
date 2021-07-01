#include "texas_robotics_academy/texbot_wrapper.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#define MIN_SPEED -100.0
#define MAX_SPEED 100.0

TexBot::TexBot() {
  button = 0; // Needs to be manually reset because it isn't constantly published
  twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);
  scanner_sub = n.subscribe("/scan", 5, &TexBot::wallSensorCB, this);
  line_sub = n.subscribe("/color_sensor", 5, &TexBot::lineSensorCB, this);
  button_sub = n.subscribe("/button", 5, &TexBot::buttonCB, this);
  // The "true" at the end means "send the latest message to any new publishers"
  lcd1_pub = n.advertise<std_msgs::String>("/lcd1", 5, true);
  lcd2_pub = n.advertise<std_msgs::String>("/lcd2", 5, true);
  ledLeft_pub = n.advertise<std_msgs::Bool>("/led/led_2_link/lamp", 5, true);
  ledRight_pub = n.advertise<std_msgs::Bool>("/led/led_1_link/lamp", 5, true);
  timer_pub = n.advertise<std_msgs::Empty>("/stimer", 5, true);

  // Sets dimensions for the matrices used for
  // BotNRoll speed computations
  A.resize(2,2);
  x.resize(2,1);
  B.resize(2,1);

}

void TexBot::wallSensorCB(const sensor_msgs::LaserScan::ConstPtr& scan) {
  int rangeSize = scan->ranges.size();
  right_dist = scan->ranges[0];
  left_dist = scan->ranges[rangeSize-1];
}

void TexBot::lineSensorCB(const std_msgs::Int16MultiArray::ConstPtr& msg){
  for(int i = 0; i<8;i++){
    values[i] = msg->data[i];
  }
}

void TexBot::buttonCB(const std_msgs::UInt8::ConstPtr& msg){
  button = msg->data;
}

// Controlling the linear and angular velocity
// of the BotNRoll Robot.User can specify speed
// of left and right wheel of robot to control
// robot motion
void TexBot::move(float speedL, float speedR) {

  if((speedL < -100. || speedL > 100) || (speedR < -100 || speedR > 100.0)){
    ROS_ERROR_STREAM("Error: Speed of wheel must be between -100 and 100!");
    ROS_ERROR_STREAM("Your inputs were " << speedL << ", " << speedR);

    speedL = std::max((float)MIN_SPEED, std::min(speedL, (float)MAX_SPEED));
    speedR = std::max((float)MIN_SPEED, std::min(speedR, (float)MAX_SPEED));

    ROS_ERROR_STREAM("Clamped to " << speedL << ", " << speedR);
  } else{
    // Solving for x in A*x = B
    // Converts wheel speeds to Twist Message

    // Distance between the left and right wheel
    double WHEEL_SEPARATION = 0.3;

    A(0,0) = 1;
    A(0,1) = -1*(WHEEL_SEPARATION/2);
    A(1,0) = 1;
    A(1,1) = (WHEEL_SEPARATION/2);
    A = A.inverse();

    // Scaling down speed from -100.0 to 100.0 to -1.0 to 1.0
    B(0, 0) = (speedL/100.0);
    B(1, 0) = (speedR/100.0);

    // Computing x (linear velocity) and z (angular velocity)
    x = A * B;

    // Sending BotNROll Robot move directions
    twist_msg.linear.x = x(0,0);
    //TODO Hotfix, we should fix the actual math
    twist_msg.angular.z = -x(1,0);
    twist_pub.publish(twist_msg);
  }

}

double TexBot::leftObstacleSensor() {
  return left_dist;
}

double TexBot::rightObstacleSensor() {
  return right_dist;
}

int TexBot::readLineSensor(int sensorNum){
  if(sensorNum<0 || sensorNum> 7){
    ROS_ERROR("Not a valid sensor number");
    return -1;
  }
  return values[sensorNum];
}

int TexBot::readButton(){
  int buttonVal = button;
  return buttonVal;
}

void TexBot::ledLeft(bool on){
  std_msgs::Bool msg;
  msg.data = on;
  ledLeft_pub.publish(msg);
}
void TexBot::ledRight(bool on){
  std_msgs::Bool msg;
  msg.data = on;
  ledRight_pub.publish(msg);
}

void TexBot::startTimer(){
  std_msgs::Empty msg;
  timer_pub.publish(msg);
}


// LCD displays
#define LCD_SIZE 16
#define LCD_WRITE(func, ...) \
  char str[LCD_SIZE+1]; \
  snprintf(str, LCD_SIZE+1, __VA_ARGS__); \
  func(str);

void TexBot::lcd1(const std::string &string){
  std_msgs::String msg;
  msg.data = string;
  lcd1_pub.publish(msg);
}
void TexBot::lcd2(const std::string &string){
  std_msgs::String msg;
  msg.data = string;
  lcd2_pub.publish(msg);
}
