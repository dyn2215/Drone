#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>                           
#include <random>
#include <cmath>

using namespace std;
using namespace ros;

class TargetPubNode{
private:
    ros::NodeHandle nh;
    ros::Subscriber start_subscriber;
    ros::Publisher pose0_pub;
    ros::Publisher pose1_pub;
    ros::Publisher pose2_pub;
    geometry_msgs::PoseStamped pose0,pose1,pose2;

    void show_msg_callback(const std_msgs::String::ConstPtr& msg);
    
public:
    TargetPubNode();
};

TargetPubNode::TargetPubNode()
{
    pose0.header.frame_id = "world";
    pose0.header.seq=0;
    pose0.header.stamp=ros::Time::now();
    pose0.pose.position.x = 25;
    pose0.pose.position.y = 7;
    pose0.pose.position.z = 0;
    pose0.pose.orientation.x = 0;
    pose0.pose.orientation.y = 0;
    pose0.pose.orientation.z = 0;
    pose0.pose.orientation.w = 1;

    pose1=pose0;
    pose1.pose.position.y = 2;
    pose2=pose0;
    pose2.pose.position.y = -3;

    pose0_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_0_ego_planner_node/move_base_simple/goal", 10);
    pose1_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_1_ego_planner_node/move_base_simple/goal", 10);
    pose2_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_2_ego_planner_node/move_base_simple/goal", 10);
    start_subscriber = nh.subscribe<std_msgs::String>("/start",10,&TargetPubNode::show_msg_callback,this);
}

void TargetPubNode::show_msg_callback(const std_msgs::StringConstPtr& msg)
{
    cout<<"Target published!\n";
    pose0_pub.publish(pose0);
    pose1_pub.publish(pose1);
    pose2_pub.publish(pose2);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "target_pub_node");
    TargetPubNode node;
    ros::spin();        //循环等待回调函数
    return 0;
}