// this ROS node aims to calibrate the drone odometry with UWB measurements, via GTSAM
// the UWB measurements are published by /simulations/uwb_sim_node
// the odometry is published by /simulations/droneX/draftnode, where X is the drone number
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>                                               

#include <random>
#include <cmath>

class UwbCaliNode {
private:
    //nodehandle
    ros::NodeHandle nh, pnh{"~"};

    //configs
    int uav_num{3}; // number of uavs
    int self_id{0}; // self id
    std::vector<std::string> global_before_name; // topic names (before calibrations)
    std::vector<std::string> global_before_name; // topic names (before calibrations)
    std::string global_after_name; // topic name (after calibrations), only for self_id
    std::string uwb_sim_result_name;
    double sigma_r{0.3};    // in meter
    double sigma_Y{5};    // in degree
    double sigma_P{10};    // in degree

    //subscribers and publishers
    std::vector<ros::Subscriber> global_before_sub;
    ros::Subscriber uwb_sim_result_sub;
    ros::Publisher global_after_pub;
    
    // variables
    // uwb测量结果
    uwb_msgs::UwbResultStamped uwb_sim_result;
    std::vector<geometry_msgs::PoseStamped> pos_before_stamped;
    geometry_msgs::PoseStamped pos_after_stamped;
 
    // callbacks
    void GlobalBeforeCb(const nav_msgs::OdometryConstPtr &msg, int uav_id); // 无人机位置回调函数
    void UwbResultCb(const ros::TimerEvent &timerEvent); // UWB测量结果回调函数
    void UpdatePose(); // 更新测量值
public:
    UwbCaliNode();   // 构造函数
};

UwbCaliNode::UwbCaliNode(){
    nh.param<int>("/uav_num", uav_num, 3);
    pnh.param<int>("self_id", self_id, 0);
    pnh.param<double>("sigma_r", sigma_r, 0.1);
    pnh.param<double>("sigma_Y", sigma_Y, 5);
    pnh.param<double>("sigma_P", sigma_P, 10);

    // initialize vectors
    global_before_name.resize(uav_num);
    global_before_sub.resize(uav_num);
    pos_before_stamped.resize(uav_num);


    for(int i=0; i<uav_num; i++){
        // initialize topic names
        global_before_name[i] = "/drone" + std::to_string(i) + "/global_position_cali";
        uwb_sim_result_name[i] = "/drone" + std::to_string(i) + "/uwb_sim_result";
        // subscribe to groundtruth_global_names
        groundtruth_global_sub[i] = nh.subscribe<nav_msgs::Odometry>(groundtruth_global_name[i], 5, boost::bind(&UwbSimNode::GroundtruthCb, this, _1, i));
        // publish uwb_sim_range and uwb_sim_Y, uwb_sim_P
        uwb_sim_range_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_range_name[i], 5);
        uwb_sim_Y_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_Y_name[i], 5);
        uwb_sim_P_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_P_name[i], 5);
    }
}

void UwbCaliNode::GlobalBeforeCb(const ros::TimerEvent &timerEvent) {
    timer_cnt++;
    // update measurements
    Measureupdate();
    // publish measurements
    for(int i=0; i<uav_num; i++){
        uwb_sim_range_pub[i].publish(uwb_sim_range[i]);
        uwb_sim_Y_pub[i].publish(uwb_sim_Y[i]);
        uwb_sim_P_pub[i].publish(uwb_sim_P[i]);
    }
    return;
}

void UwbSimNode::Measureupdate(){
    // assign range, yaw, pitch measurements for each uav
    for(int i=0; i<uav_num; i++){
        uwb_sim_range[i].header = latest_header;
        uwb_sim_Y[i].header = latest_header;
        uwb_sim_P[i].header = latest_header;
        for(int j=0; j<uav_num; j++){
            uwb_sim_range[i].uav_ids[j] = j;
            uwb_sim_Y[i].uav_ids[j] = j;
            uwb_sim_P[i].uav_ids[j] = j;
            if(i == j){
                uwb_sim_range[i].uav_values[j] = 0;
                uwb_sim_Y[i].uav_values[j] = 0;
                uwb_sim_P[i].uav_values[j] = 0;
                continue;
            }
            // range
            uwb_sim_range[i].uav_values[j] = sqrt(pow(uav_gt_pos[i].x - uav_gt_pos[j].x, 2) + 
                                                pow(uav_gt_pos[i].y - uav_gt_pos[j].y, 2) + 
                                                pow(uav_gt_pos[i].z - uav_gt_pos[j].z, 2)) + 
                                                randn(0, sigma_r);
        }
    }

    int i=timer_cnt%uav_num;
    // transform to local frame using convertCoordinate function
    for(int j=0; j<uav_num; j++){
        if(i == j){
            uwb_sim_range[i].uav_values[j] = 0;
            continue;
        }
        geometry_msgs::Point original;
        original.x = uav_gt_pos[i].x - uav_gt_pos[j].x;
        original.y = uav_gt_pos[i].y - uav_gt_pos[j].y;
        original.z = uav_gt_pos[i].z - uav_gt_pos[j].z;
        geometry_msgs::Point transformed;
        // transform to local frame
        transformed = convertCoordinate(original, uav_gt_quat[i]);       
        // yaw
        uwb_sim_Y[i].uav_values[j] = atan2(transformed.y, transformed.x) + randn(0,sigma_Y)*M_PI/180;
        // pitch
        uwb_sim_P[i].uav_values[j] = atan2(transformed.z, sqrt(pow(transformed.x, 2) + pow(transformed.y, 2))) + randn(0,sigma_P)*M_PI/180;
    }

}

void UwbSimNode::GroundtruthCb(const nav_msgs::OdometryConstPtr &msg, int index) {
    uav_gt_pos[index] = msg->pose.pose.position;
    uav_gt_quat[index] = msg->pose.pose.orientation;    
    latest_header = msg->header;
    return;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_sim_node");
    UwbSimNode node;
    ros::spin();
}