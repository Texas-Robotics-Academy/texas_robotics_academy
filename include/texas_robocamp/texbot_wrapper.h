#ifndef TEXBOT_WRAPPER_H
#define TEXBOT_WRAPPER_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16MultiArray.h"
#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8.h"
#include <Eigen/Dense>
#include <sstream>

class TexBot
{
public:
    TexBot();
    void move(float speedL, float speedR);
    double leftObstacleSensor();
    double rightObstacleSensor();
    int readLineSensor(int sensorNum);
    int readButton();
    void ledLeft(bool on);
    void ledRight(bool on);
    void startTimer();

    void lcd1(const std::string &line);
    void lcd2(const std::string &line);
    template<typename ... T> void lcd1(T ... args);
    template<typename ... T> void lcd2(T ... args);

protected:
  ros::NodeHandle n;
  ros::Publisher twist_pub;
  ros::Publisher lcd1_pub;
  ros::Publisher lcd2_pub;
  ros::Publisher ledLeft_pub;
  ros::Publisher ledRight_pub;

  ros::Publisher timer_pub;

  ros::Subscriber scanner_sub;
  ros::Subscriber line_sub;
  ros::Subscriber button_sub;
  geometry_msgs::Twist twist_msg;
  double left_dist;
  double right_dist;
  int values[8];
  int button;
  void wallSensorCB(const sensor_msgs::LaserScan::ConstPtr& scan);
  void lineSensorCB(const std_msgs::Int16MultiArray::ConstPtr& array);
  void buttonCB(const std_msgs::UInt8::ConstPtr& array);
  // Variables for convertion from
  // wheel speed to Twist Message
  Eigen::MatrixXd A;
  Eigen::MatrixXd x;
  Eigen::MatrixXd B;
};

// Defined in the header so the compiler can expand it
template<typename ... T> void TexBot::lcd1(T ... args) {
  std::stringstream ss;
  ss.precision(3);
  ss << std::fixed;
  int dummy[sizeof...(T)] = {(ss << args, 0)...};
  lcd1(ss.str());
}
template<typename ... T> void TexBot::lcd2(T ... args) {
  std::stringstream ss;
  ss.precision(3);
  ss << std::fixed;
  int dummy[sizeof...(T)] = {(ss << args, 0)...};
  lcd2(ss.str());
}

#endif
