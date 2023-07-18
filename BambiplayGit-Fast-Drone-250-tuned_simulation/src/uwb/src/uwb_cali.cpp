// this ROS node aims to calibrate the drone odometry with UWB measurements, via GTSAM
// the UWB measurements are published by /simulations/uwb_sim_node
// the odometry is published by /simulations/droneX/draftnode, where X is the drone number

// ROS related
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>              
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>                                 

// GTSAM related
#include "2DRYEFactors.hpp"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>  // contains NoiseModelFactorN

//  C++ related
#include <random>
#include <cmath>

using namespace gtsam;
using namespace std;

double angle_protect(double angle)
{
    while(abs(angle) > M_PI)
    {
        if(angle > M_PI)
            angle -= 2 * M_PI;
        else
            angle += 2 * M_PI;
    }
    return angle;
}

class UwbCaliNode {
private:
    //nodehandle
    ros::NodeHandle nh, pnh{"~"};

    //configs
    int uav_num{3}; // number of uavs
    int self_id{0}; // self id
    double sigma_r{0.3};    // measurement noise, in meter
    double sigma_Y{5};    // measurement noise, in degree
    double sigma_P{10};    // measurement noise, in degree

    std::string odom_name_before; //corresponding topic name (before calibration)
    std::vector<std::string> odom_names_after; // topic names (after calibrations)

    //subscribers and publishers
    std::vector<ros::Subscriber> odom_after_subs;   // subscribers for odometries after calibration
    ros::Subscriber odom_before_sub;    // subscriber for odometry before calibration
    ros::Publisher odom_after_pub;  // publisher for odometry after calibration
    
    // variables
    uwb_msgs::UwbResultStamped uwb_measurements;
    std::vector<nav_msgs::Odometry> odoms_after;
    nav_msgs::Odometry odom_before;
    double odom_after_RPY[3]={0,0,0};   // roll, pitch, yaw
    geometry_msgs::Pose2D driftcorrection; // drift correction between odom_before and odom_after, odoms_after[self_id] = odom_before + drift
 
    // callbacks
    void OdomAfterCb(const nav_msgs::OdometryConstPtr &msg, int uav_id);    // callback for odometries after calibration
    void OdomBeforeCb(const nav_msgs::OdometryConstPtr &msg);   // callback for odometry before calibration
    void UwbResultCb(const const uwb_msgs::UwbResultStampedConstPtr &msg);   // callback for uwb measurements
    void UpdateOdom();  // update odometry after calibration
    void UpdateDrift(); // update drift estimation using GTSAM
public:
    UwbCaliNode();   // 构造函数
};

UwbCaliNode::UwbCaliNode(){
    nh.param<int>("/uav_num", uav_num, 3);
    pnh.param<int>("self_id", self_id, 0);
    pnh.param<double>("sigma_r", sigma_r, 0.1);
    pnh.param<double>("sigma_Y", sigma_Y, 5);
    pnh.param<double>("sigma_P", sigma_P, 10);
    
    driftcorrection.x=driftcorrection.y=driftcorrection.theta=0;

    // initialize vectors
    odom_name_before= "/drone" + std::to_string(self_id) + "/global_odom_sim";
    odom_names_after.resize(uav_num);
    odom_after_subs.resize(uav_num);
    odoms_after.resize(uav_num);

    for(int i=0; i<uav_num; i++){
        odom_names_after[i] = "/drone" + std::to_string(i) + "/global_odom_cali";
        if(i==self_id) continue;
        odom_after_subs[i] = nh.subscribe<nav_msgs::Odometry>(odom_names_after[i], 5, boost::bind(&UwbCaliNode::OdomAfterCb, this, _1, i));
    }

    odom_before_sub = nh.subscribe<nav_msgs::Odometry>(odom_name_before, 5, &UwbCaliNode::OdomBeforeCb, this);
    odom_after_pub = nh.advertise<nav_msgs::Odometry>(odom_names_after[self_id], 5);
}

void UwbCaliNode::OdomAfterCb(const nav_msgs::OdometryConstPtr &msg, int uav_id){
    odoms_after[uav_id] = *msg;
    return;
}

void UwbCaliNode::OdomBeforeCb(const nav_msgs::OdometryConstPtr &msg){
    odom_before = *msg;
    UpdateOdom();
    odom_after_pub.publish(odoms_after[self_id]);
    return;
}

void UwbCaliNode::UwbResultCb(const uwb_msgs::UwbResultStampedConstPtr &msg){
    uwb_measurements = *msg;
    // update drift correction
    UpdateDrift();
    return;
}

void UwbCaliNode::UpdateOdom(){
    nav_msgs::Odometry odom_temp;   // in order to maintain continuity of odoms_after[self_id]
    double odom_RPY_temp[3]={0,0,0};    // in order to maintain continuity of odoms_after[self_id]
    // update odometry after calibration
    odom_temp = odom_before;
    odom_temp.pose.pose.position.x += driftcorrection.x;
    odom_temp.pose.pose.position.y += driftcorrection.y;
    // convert to RPY
    tf2::Quaternion gtquat;
    tf2::convert(odom_before.pose.pose.orientation,gtquat);
    tf2::Matrix3x3 m(gtquat);
    m.getRPY(odom_RPY_temp[0], odom_RPY_temp[1], odom_RPY_temp[2]);
    // update RPY
    odom_RPY_temp[2] = odom_RPY_temp[2] + driftcorrection.theta;
    // convert back to quaternion
    tf2::Quaternion newquat;
    newquat.setRPY(odom_RPY_temp[0], odom_RPY_temp[1], odom_RPY_temp[2]);
    odom_temp.pose.pose.orientation = tf2::toMsg(newquat);
    // update
    odoms_after[self_id]=odom_temp;
    odom_after_RPY[0]=odom_RPY_temp[0];
    odom_after_RPY[1]=odom_RPY_temp[1];
    odom_after_RPY[2]=odom_RPY_temp[2];
    return;
}

// update drift estimation using GTSAM
void UwbCaliNode::UpdateDrift(){    
    // self pose before calibration
    double self_xyz_before[3]={odoms_after[self_id].pose.pose.position.x,
    odoms_after[self_id].pose.pose.position.y,odoms_after[self_id].pose.pose.position.z};
    double self_RPY_before[3]={odom_after_RPY[0],odom_after_RPY[1],odom_after_RPY[2]};
    double prior_noise_xyY[3]={0,0,0};  // needs to be tuned

    // noise models
    double rangenoise=sigma_r;
    double elevnoise=sigma_P;
    double yawnoise=sigma_Y;
    noiseModel::Diagonal::shared_ptr rnmodel = noiseModel::Diagonal::Sigmas(Vector1(rangenoise));
    noiseModel::Diagonal::shared_ptr enmodel = noiseModel::Diagonal::Sigmas(Vector1(elevnoise));
    noiseModel::Diagonal::shared_ptr ynmodel = noiseModel::Diagonal::Sigmas(Vector1(yawnoise));
    
    NonlinearFactorGraph graph;
    Pose2 priorMean(self_xyz_before[0], self_xyz_before[1], self_RPY_before[2]);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(prior_noise_xyY[0], prior_noise_xyY[1], prior_noise_xyY[2]));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

   // add factors
    for(int i=0; i<uav_num; i++){
        if(i==self_id) continue;
        double other_xyz[3]={odoms_after[i].pose.pose.position.x,
        odoms_after[i].pose.pose.position.y,odoms_after[i].pose.pose.position.z};
        double uwb_range = uwb_measurements.uav_r[i];
        double uwb_P = uwb_measurements.uav_P[i];
        double uwb_Y = uwb_measurements.uav_Y[i];
        graph.add(RangeFactor(1, uwb_range, other_xyz[0], other_xyz[1], other_xyz[2], rnmodel));
        graph.add(ElevFactor(1, uwb_P, other_xyz[0], other_xyz[1], other_xyz[2], enmodel));
        graph.add(YawFactor(1, uwb_Y, other_xyz[0], other_xyz[1], other_xyz[2], ynmodel));
    }
    // graph.print("Factor Graph:\n");

    // set initial values and optimize
    Values initial;
    initial.insert(1, priorMean);
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    // get new drift correction
    double r,p,y;
    tf2::Quaternion gtquat;
    tf2::convert(odom_before.pose.pose.orientation,gtquat);
    tf2::Matrix3x3 m(gtquat);
    m.getRPY(r, p, y);
    Pose2 newpose = result.at<Pose2>(1);
    driftcorrection.x = newpose.x() - odom_before.pose.pose.position.x;
    driftcorrection.y = newpose.y() - odom_before.pose.pose.position.y;
    driftcorrection.theta = angle_protect(newpose.theta() - y);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_cali_node");
    UwbCaliNode node;
    ros::spin();
}