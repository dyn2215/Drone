#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <vector>
#include <iostream>
#include <random>
#include <string>
#include <map>


double random_double(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
    std::cout << "initializing random goals" << std::endl;

    ros::init(argc, argv, "random_goal_node");

    ros::NodeHandle nh, pnh{"~"};// the topics will be like /node_namespace/node_name

    ros::Publisher random_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1); 
    
    int id = -1;
    pnh.getParam("id",id);
    double map_min =-2.2;
    double map_max = 2.2;

    ros::Rate loop_rate(0.1); 
    srand(time(0) + id);

    //ROS_INFO("time(0) = %f.",time(0));


    while(ros::ok()) 
    {
      geometry_msgs::PoseStamped random_goal_msg;
      random_goal_msg.header.frame_id = "map";
      random_goal_msg.pose.position.x = random_double(map_min,map_max);
      random_goal_msg.pose.position.y = random_double(map_min,map_max);
      random_goal_msg.pose.position.z = 0;
      random_goal_msg.pose.orientation.x = 0;
      random_goal_msg.pose.orientation.y = 0;
      random_goal_msg.pose.orientation.z = 0;
      random_goal_msg.pose.orientation.w = 1;
      //ROS_INFO("goal %f %f generated.",random_goal_msg.pose.position.x,random_goal_msg.pose.position.y);
      random_goal_pub.publish(random_goal_msg);
      loop_rate.sleep();
    }
    return 0;
};


//  *header: 
//   seq: 2
//   stamp: 
//     secs: 546
//     nsecs: 704000000
//   frame_id: "map"
// pose: 
//   position: 
//     x: 1.39577364922
//     y: 0.219623327255
//     z: 0.0
//   orientation: 
//     x: 0.0
//     y: 0.0
//     z: 0.0
//     w: 1.0
// ---