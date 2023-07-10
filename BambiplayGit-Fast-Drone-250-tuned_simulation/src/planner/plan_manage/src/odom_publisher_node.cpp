#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h> //must have this head
#include <cstdio>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/static_transform_broadcaster.h>

class OdomPublisherNode {
private:
    const std::string drone_prefix{"drone"};
    ros::NodeHandle nh, pnh{"~"};// the topics will be like /node_namespace/node_name

    int num_uav;

    std::vector<geometry_msgs::Point> uavInitPos;
    std::vector<float> uavInitPos_;

    std::vector<ros::Subscriber> uavOdomSub; //std::vector can store a determinitic 
    std::vector<ros::Publisher> uavOdomPub; //std::vector can store a determinitic 
    ros::Subscriber linkStatesSub;
    // tf2_ros::TransformBroadcaster tfBr;
    // tf2_ros::StaticTransformBroadcaster staticTfBr;
    
    void uavOdomCb(const nav_msgs::OdometryConstPtr & odom, int id);

public:
    explicit OdomPublisherNode();//constructer
};

int main(int argc, char **argv)
{
    std::cout << "jet" << std::endl;

    ros::init(argc, argv, "odom_publisher_node");

    OdomPublisherNode odom_publisher_node;
    ros::spin();
    // while(ros::ok()) 
    // {
	// 	OdomPublisherNode odom_publisher_node;                   
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    return 0;
};

OdomPublisherNode::OdomPublisherNode() {
    nh.param<int>("/uav_num", num_uav, 3);
    nh.param("/uav_init_pos", uavInitPos_, std::vector<float>(0));

    uavInitPos.resize(num_uav);
    uavOdomSub.resize(num_uav);
    uavOdomPub.resize(num_uav);

    std::cout << "fuck" << std::endl;
    for (int i = 0; i < num_uav; i++) {
        std::cout << "fuck" << i << std::endl;
        uavInitPos[i].x = uavInitPos_[i * 2];
        uavInitPos[i].y = uavInitPos_[i * 2 + 1];
    }
    // UAV Odom Pub
   
    // UAV Odom subscribe
    for (int i = 0; i < num_uav; i++) {
        uavOdomPub[i] = nh.advertise<nav_msgs::Odometry>(
            drone_prefix + std::to_string(i)+ "/odom_true",6);

        uavOdomSub[i] = nh.subscribe<nav_msgs::Odometry>(
                drone_prefix + std::to_string(i)+ "/mavros/local_position/odom", 6,
                boost::bind(&OdomPublisherNode::uavOdomCb, this, _1, i));
    }
}

void OdomPublisherNode::uavOdomCb(const nav_msgs::OdometryConstPtr & msg, int id) {
    // geometry_msgs::TransformStamped tf;
    nav_msgs::Odometry odom_true;
    odom_true.header.stamp    = msg->header.stamp;
	odom_true.header.frame_id = "world";
    //position 是不是一开始加了个这玩意儿以后越来越漂
    odom_true.pose.pose.position.x = msg->pose.pose.position.x + uavInitPos[id].x;
    odom_true.pose.pose.position.y = msg->pose.pose.position.y + uavInitPos[id].y;
    odom_true.pose.pose.position.z = msg->pose.pose.position.z;

    //orientation
    odom_true.pose.pose.orientation = msg->pose.pose.orientation;
    //twist
    odom_true.twist = msg->twist;

    uavOdomPub[id].publish(odom_true);
}