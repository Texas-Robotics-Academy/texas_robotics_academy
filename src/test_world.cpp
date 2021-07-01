#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

class TestWorld {

	ros::Time old = ros::Time();
	ros::Time oldTwo = ros::Time();
	ros::Time oldThree = ros::Time();
	ros::Time oldFour = ros::Time();
	ros::Time oldFive = ros::Time();
	
	public:
		
		//the following functions calculate the rate at which 
		//the relevant topic is published at by calculating the 
		//difference between the time a message was received
		//and the time of the the immedeitly older message. 

		void colorSensorCB(const std_msgs::Int16MultiArray::ConstPtr &msg) {
			ros::Time time = ros::Time::now();
			ROS_INFO("Color Sensor Pub Rate: %i secs %i nsecs", (time.sec - old.sec), (time.nsec - old.nsec));
			old = time;
		}
		
		void obstacleSensorCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
			ros::Time time = ros::Time::now();
			ROS_INFO("Obstacle Sensor Pub Rate: %i secs %i nsecs", (time.sec - oldTwo.sec), (time.nsec - oldTwo.nsec));
			oldTwo = time;
		}

		void buttonSensorCB(const std_msgs::UInt8::ConstPtr &msg) {
			ros::Time time = ros::Time::now();
			ROS_INFO("Button Pub Rate: %i secs %i nsecs", (time.sec - oldThree.sec), (time.nsec - oldThree.nsec));
			oldThree = time;
		}

		void ledTwoSensorCB(const std_msgs::Bool::ConstPtr &msg) {
			ros::Time time = ros::Time::now();
			ROS_INFO("LED 2 Pub Rate: %i secs %i nsecs", (time.sec - oldFour.sec), (time.nsec - oldFour.nsec));
			oldFour = time;
		}

		void ledOneSensorCB(const std_msgs::Bool::ConstPtr &msg) {
			ros::Time time = ros::Time::now();
			ROS_INFO("LED 1 Pub Rate: %i secs %i nsecs", (time.sec - oldFive.sec), (time.nsec - oldFive.nsec));
			oldFive = time;
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "TestWorld");
	ros::NodeHandle node;
	TestWorld tw;
	
	//subscribe to each sensor topic in order to get
	//the rate at which they are published
	ros::Subscriber subColor = node.subscribe("/color_sensor", 
			100, &TestWorld::colorSensorCB, &tw);
	ros::Subscriber subObstacle = node.subscribe("/scan", 
			100, &TestWorld::obstacleSensorCB, &tw);
	ros::Subscriber subButton = node.subscribe("/button", 
			100, &TestWorld::buttonSensorCB, &tw);
	ros::Subscriber subLedOne = node.subscribe("/led/led_2_link/lamp", 
			100, &TestWorld::ledTwoSensorCB, &tw);
	ros::Subscriber subLedTwo = node.subscribe("/led/led_1_link/lamp", 
			100, &TestWorld::ledOneSensorCB, &tw);
	ros::spin();

	return 0;
}
