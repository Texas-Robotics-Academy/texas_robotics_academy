// ROS Node that implements teleop
// for the BotNRoll Robot
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ncurses.h>
#include <signal.h>
#include <std_msgs/Empty.h>

#define FORWARD 1
#define REVERSE 2
#define TURNING 3
#define STOP 0

#define INIT_LIN_SPEED 0.1
#define INIT_ANG_SPEED 0.5

ros::Publisher teleopPub;

void signalHandler(int signum) {

  //Stop the robot before we exit
  geometry_msgs::Twist stop;
  stop.linear.x = 0;
  stop.angular.z = 0;
  teleopPub.publish(stop);

  ros::shutdown();
  endwin();
  exit(0);
}

void showSpeed(double linear, double angular) {
  move(12,0);
  printw("Linear Speed: %.2f\n", linear);
  printw("Angular Speed: %.2f\n", angular);
  refresh();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_texbot");
	ros::NodeHandle n;

  signal(SIGINT, signalHandler);

	// Publishes Twist Messages to '/cmd_vel' control TexBot URDF
	teleopPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher resetPub = n.advertise<std_msgs::Empty>("/reset",1000);
  std_msgs::Empty resetMsg;

	char input;
	// Message type that TexBot uses to move around in simulation
	geometry_msgs::Twist twistMessage;
	double linearSpeed = INIT_LIN_SPEED;
  double angularSpeed = INIT_ANG_SPEED;
	int moving = STOP;
  int direction = 1;
  
  initscr();
  noecho();
	// Prints teleop instructions for user.
	printw("Move Forward           : W\n");
	printw("Turn Left              : A\n");
	printw("Turn Right             : D\n");
  printw("Reverse                : X\n");
	printw("Stop                   : S\n");
	printw("Increase Linear Speed  : I\n");
	printw("Decrease Linear Speed  : O\n");
	printw("Increase Angular Speed : K\n");
	printw("Decrease Angular Speed : L\n");
  printw("Reset Robot            : R\n");
	printw("Quit Teleop            : Q\n\n");
	
  refresh();
  cbreak();

  showSpeed(linearSpeed, angularSpeed);

  while(ros::ok()) {

    input = getch();
    if(input != -1) {

      switch(input) {
        case 'W':
        case 'w':
          moving = FORWARD;
          break;
        case 'X':
        case 'x':
          moving = REVERSE;
          break;
        case 'r':
        case 'R':
         resetPub.publish(resetMsg);
        case 'S':
        case 's':
          moving = STOP;
          break;
        case 'D':
        case 'd':
          direction = 1;
          moving = TURNING;
          break;
        case 'A':
        case 'a':
          direction = -1;
          moving = TURNING;
          break;
        case 'I':
        case 'i':
          linearSpeed = (linearSpeed < 0.9) ? (linearSpeed + 0.1) : linearSpeed;
          showSpeed(linearSpeed, angularSpeed);
          break;
        case 'O':
        case 'o':
          linearSpeed = (linearSpeed > 0.1) ? (linearSpeed - 0.1) : linearSpeed;
          showSpeed(linearSpeed, angularSpeed);
          break;
        case 'K':
        case 'k':
          angularSpeed = (angularSpeed < 2.0) ? (angularSpeed + 0.1) : angularSpeed;
          showSpeed(linearSpeed, angularSpeed);
          break;
        case 'L':
        case 'l':
          angularSpeed = (angularSpeed > 0.1) ? (angularSpeed - 0.1) : angularSpeed;
          showSpeed(linearSpeed, angularSpeed);
         break;
        case 'q':
        case 'Q':
        case '\x03':
          raise(SIGINT);
      }

      if(moving == FORWARD) {
        // Update speed in case I or O were hit
        twistMessage.linear.x = linearSpeed;
        twistMessage.angular.z = 0;
      }
      else if (moving == REVERSE) {
        twistMessage.linear.x = -linearSpeed;
        twistMessage.angular.z = 0;
      }
      else if (moving == TURNING) {
        twistMessage.linear.x = 0;
        twistMessage.angular.z = angularSpeed * direction;
      }
      else if (moving == STOP){
        twistMessage.linear.x = 0;
        twistMessage.angular.z = 0;
      }

		  teleopPub.publish(twistMessage);

    }
  }

  return 0;
}
