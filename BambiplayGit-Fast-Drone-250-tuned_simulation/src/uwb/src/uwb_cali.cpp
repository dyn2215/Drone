// this ROS node aims to calibrate the drone odometry with UWB measurements, via GTSAM
// the UWB measurements are published by uwb_sim.cpp
// this node subscribes to the UWB measurements and drone odometry, and publishes the calibrated drone odometry
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>                                               

#include <random>
#include <cmath>

// this function is used to generate a random number obeying Gaussian distribution
double randn(double mu, double sigma)
{
    // set random seed    
    std::random_device rd; // Non-determinstic seed source
    static std::default_random_engine e{rd()};
    static std::normal_distribution<double> norm(0, 1);
    return mu + sigma * norm(e);
}

// this function converts a coordinate (x,y,z) to (x1,y1,z1), (x,y,z) is the coordinate in S, (x1,y1,z1) is the coordinate in S1
// S1 has the same origin with S, but has a different orientation, quaternion (qx,qy,qz,qw)
// the rotation matrix from S to S1 is R = [qx^2-qy^2-qz^2+qw^2, 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);
//                                          2*(qx*qy+qz*qw), -qx^2+qy^2-qz^2+qw^2, 2*(qy*qz-qx*qw);
//                                          2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), -qx^2-qy^2+qz^2+qw^2]
// the coordinate (x,y,z) in S can be converted to (x1,y1,z1) in S1 by R*[x;y;z]
geometry_msgs::Point convertCoordinate(geometry_msgs::Point p, geometry_msgs::Quaternion q)
{
    geometry_msgs::Point p1;
    double x = p.x;
    double y = p.y;
    double z = p.z;
    double qx = q.x;
    double qy = q.y;
    double qz = q.z;
    double qw = q.w;
    p1.x = (qx*qx-qy*qy-qz*qz+qw*qw)*x + 2*(qx*qy-qz*qw)*y + 2*(qx*qz+qy*qw)*z;
    p1.y = 2*(qx*qy+qz*qw)*x + (-qx*qx+qy*qy-qz*qz+qw*qw)*y + 2*(qy*qz-qx*qw)*z;
    p1.z = 2*(qx*qz-qy*qw)*x + 2*(qy*qz+qx*qw)*y + (-qx*qx-qy*qy+qz*qz+qw*qw)*z;
    return p1;
}

class UwbSimNode {
private:
    //nodehandle
    ros::NodeHandle nh, pnh{"~"};

    //configs
    int uav_num{3};
    std::vector<std::string> groundtruth_global_name;
    std::vector<std::string> uwb_sim_range_name;
    std::vector<std::string> uwb_sim_Y_name;
    std::vector<std::string> uwb_sim_P_name;
    double timer_duration{0.01};    // 100 Hz
    double sigma_r{0.3};    // in meter
    double sigma_Y{5};    // in degree
    double sigma_P{10};    // in degree

    //subscribers and publishers
    std::vector<ros::Subscriber> groundtruth_global_sub;
    std::vector<ros::Publisher> uwb_sim_range_pub;
    std::vector<ros::Publisher> uwb_sim_Y_pub;
    std::vector<ros::Publisher> uwb_sim_P_pub;

    // timer
    ros::Timer timer; // timer, in order to publish the uwb measurement at a certain frequency
    int timer_cnt;

    // variables
    // uwb测量结果,第一层为无人机id,随timer更新
    std::vector<uwb_msgs::UwbResultStamped> uwb_sim_range;
    std::vector<uwb_msgs::UwbResultStamped> uwb_sim_Y;
    std::vector<uwb_msgs::UwbResultStamped> uwb_sim_P;
    // 存储无人机gt位置的一维数组,随odom_true话题更新
    std::vector<geometry_msgs::Point> uav_gt_pos;
    std::vector<geometry_msgs::Quaternion> uav_gt_quat;
    std_msgs::Header latest_header; // latest header of the groundtruth_global topic

    // callbacks
    void GroundtruthCb(const nav_msgs::OdometryConstPtr &msg, int uav_id); // 无漂移的真实值的回调函数
    void SimMeasureCb(const ros::TimerEvent &timerEvent); // Timer回调函数,更新测量值并输出
    void Measureupdate(); // 更新测量值
public:
    UwbSimNode();
};

UwbSimNode::UwbSimNode(){
    nh.param<int>("/uav_num", uav_num, 3);
    pnh.param<double>("timer_duration", timer_duration, 0.01);
    pnh.param<double>("sigma_r", sigma_r, 0.1);
    pnh.param<double>("sigma_Y", sigma_Y, 5);
    pnh.param<double>("sigma_P", sigma_P, 10);

    // initialize vectors
    groundtruth_global_name.resize(uav_num);
    uwb_sim_range_name.resize(uav_num);
    uwb_sim_Y_name.resize(uav_num);
    uwb_sim_P_name.resize(uav_num);
    groundtruth_global_sub.resize(uav_num);
    uwb_sim_range_pub.resize(uav_num);
    uwb_sim_Y_pub.resize(uav_num);
    uwb_sim_P_pub.resize(uav_num);
    uwb_sim_range.resize(uav_num);
    uwb_sim_Y.resize(uav_num);
    uwb_sim_P.resize(uav_num);
    uav_gt_pos.resize(uav_num);
    uav_gt_quat.resize(uav_num);
    for(int i=0; i<uav_num; i++){
        uwb_sim_range[i].uav_ids.resize(uav_num);
        uwb_sim_range[i].uav_values.resize(uav_num);
        uwb_sim_Y[i].uav_ids.resize(uav_num);
        uwb_sim_Y[i].uav_values.resize(uav_num);
        uwb_sim_P[i].uav_ids.resize(uav_num);
        uwb_sim_P[i].uav_values.resize(uav_num);
        // initialize topic names
        groundtruth_global_name[i] = "/drone" + std::to_string(i) + "/odom_true";
        uwb_sim_range_name[i] = "/drone" + std::to_string(i) + "/uwb_sim_range";
        uwb_sim_Y_name[i] = "/drone" + std::to_string(i) + "/uwb_sim_Y";
        uwb_sim_P_name[i] = "/drone" + std::to_string(i) + "/uwb_sim_P";
        // subscribe to groundtruth_global_names
        groundtruth_global_sub[i] = nh.subscribe<nav_msgs::Odometry>(groundtruth_global_name[i], 5, boost::bind(&UwbSimNode::GroundtruthCb, this, _1, i));
        // publish uwb_sim_range and uwb_sim_Y, uwb_sim_P
        uwb_sim_range_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_range_name[i], 5);
        uwb_sim_Y_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_Y_name[i], 5);
        uwb_sim_P_pub[i] = nh.advertise<uwb_msgs::UwbResultStamped>(uwb_sim_P_name[i], 5);
    }

    timer = nh.createTimer(ros::Duration(timer_duration), &UwbSimNode::SimMeasureCb, this);
}

void UwbSimNode::SimMeasureCb(const ros::TimerEvent &timerEvent) {
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