#include <ros/ros.h>
#include "drone_detector/drone_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_detect");
  ros::NodeHandle nh("~");//this nodehandle indicates that all the topics are following drone_detect group by "/drone_detect"

  detect::DroneDetector drone_detector(nh);//define a drone detector
  drone_detector.test();

  ros::spin();
  return 0;
}
