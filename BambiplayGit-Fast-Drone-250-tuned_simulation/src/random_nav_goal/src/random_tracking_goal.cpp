#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <vector>
#include <iostream>
#include <random>
#include <string>
#include <map>
#include <nav_msgs/Odometry.h>
#include <unistd.h> 

class TrackingGoalPublisher
{
private:
    ros::NodeHandle nh, pnh{"~"};
    ros::Subscriber random_tracking_sub;
    ros::Publisher random_tracking_pub;
    void targetOdomCb(const nav_msgs::OdometryConstPtr & odom);
    const std::string robot_prefix{"robot"};

public:
    explicit TrackingGoalPublisher();//constructor
};

TrackingGoalPublisher::TrackingGoalPublisher(){
    ros::Rate loop_rate(1); 
    int id = -1;
    pnh.getParam("id",id);
    int drone_id = id - 1;
    ROS_INFO("waiting for goals!");
    random_tracking_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_"+ std::to_string(drone_id) +"_ego_planner_node/move_base_simple/goal", 1); 
    random_tracking_sub = nh.subscribe<nav_msgs::Odometry>("/robot" + std::to_string(id) + "/odom", 1,
     &TrackingGoalPublisher::targetOdomCb, this);
    loop_rate.sleep();
}

//command topic: /drone_0_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped
int main(int argc, char **argv)
{
    std::cout << "initializing random tracking" << std::endl;

    ros::init(argc, argv, "random_tracking_node");

    TrackingGoalPublisher random_tracking_node;
    ros::spin();
    

    return 0;
};

void TrackingGoalPublisher::targetOdomCb(const nav_msgs::OdometryConstPtr & msg)
{   
    ROS_INFO("received odom of robots");
    geometry_msgs::PoseStamped tracking_goal_msg;
    tracking_goal_msg.header.frame_id = "world";
    tracking_goal_msg.pose.position.x = msg->pose.pose.position.x;
    tracking_goal_msg.pose.position.y = msg->pose.pose.position.y;
    tracking_goal_msg.pose.position.z = 0;
    tracking_goal_msg.pose.orientation.x = 0;
    tracking_goal_msg.pose.orientation.y = 0;
    tracking_goal_msg.pose.orientation.z = 0;
    tracking_goal_msg.pose.orientation.w = 1;
    sleep(0.5);
    random_tracking_pub.publish(tracking_goal_msg);
    ROS_INFO("publishing drone goal: %f %f", tracking_goal_msg.pose.position.x, tracking_goal_msg.pose.position.y);
}