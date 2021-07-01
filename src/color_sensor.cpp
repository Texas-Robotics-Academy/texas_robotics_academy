#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/Int16MultiArray.h"
#include <math.h> 

class SensorConvert{
  ros::NodeHandle nh_;
	ros::Publisher val_pub;
  
  public:
    SensorConvert(){
      val_pub = nh_.advertise<std_msgs::Int16MultiArray>("/color_sensor",1000);
    }
    void sensorCB(const sensor_msgs::Image::ConstPtr &msg){
        std_msgs::Int16MultiArray values;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		    cv::Mat image = cv_ptr->image;
        int height = image.rows;
        int width = image.cols;
        int subWidth = width/8;
        int subHeight = height/4;
        int heightOffset = subHeight*1.5;
        int numSensors = 8;
        int colorOffset = 255;

        cv::Mat greyMat;
        cv::cvtColor(image,greyMat, CV_BGR2GRAY);
        for(int i = 0;i<numSensors;i++){
          cv::Rect roi1(i*subWidth,heightOffset,subWidth,subHeight);
          //ROS_INFO("roi x: %d, roi width: %d",roi1.x,roi1.width);
          cv::Mat image_roi_test = greyMat(roi1);
          cv::Scalar darkness = mean(image_roi_test);
          ROS_INFO("Brightness of section %d: %f",i, colorOffset - round(darkness.val[0]));
          values.data.push_back(colorOffset-round(darkness.val[0]));
        }
        val_pub.publish(values);
        cv::waitKey(1);
    }
  
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "color_sensor_publisher");
  
  ros::NodeHandle node;

  SensorConvert sc;
  ros::Subscriber sub = node.subscribe("/texbot/camera1/image_raw"
    , 100, &SensorConvert::sensorCB, &sc);

  ros::spin();
  return 0;
}
