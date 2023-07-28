// this ROS node aims to calibrate the drone odometry with UWB measurements, via GTSAM
// the UWB measurements are published by /simulations/uwb_sim_node
// the odometry is published by /simulations/droneX/draftnode, where X is the drone number

// ROS related
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
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
#include <gtsam/nonlinear/NonlinearFactor.h>  // contprior_noise_factorains NoiseModelFactorN

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
    unsigned int protective_counter{0};
    unsigned int protective_count{100*2};
    int uav_num{3}; // number of uavs
    int bs_num{0};  // number of base stations
    int self_id{0}; // self id
    double sigma_r{0.3};    // measurement noise, in meter
    double sigma_Y{5};    // measurement noise, in degree
    double sigma_P{10};    // measurement noise, in degree
    double uwb_frequency{100};  // uwb frequency, in Hz
    double walk_sigma_x, walk_sigma_y, walk_sigma_Y; // random walk noise, in meter

    std::string odom_name_before; //corresponding topic name (before calibration)
    std::string uwb_measurements_name;  // corresponding topic name
    std::vector<std::string> odom_names_after; // topic names (after calibrations)

    //subscribers and publishers
    std::vector<ros::Subscriber> odom_after_subs;   // subscribers for odometries after calibration
    ros::Subscriber odom_before_sub;    // subscriber for odometry before calibration
    ros::Subscriber uwb_result_sub; // subscriber for uwb measurements
    ros::Publisher odom_after_pub;  // publisher for odometry after calibration
    
    // variables
    uwb_msgs::UwbResultStamped uwb_measurements;
    std::vector<nav_msgs::Odometry> odoms_after;
    std::vector<geometry_msgs::Pose> bs_gt_poses;
    nav_msgs::Odometry odom_before;
    double odom_after_RPY[3]={0,0,0};   // roll, pitch, yaw
    geometry_msgs::Pose2D driftcorrection; // drift correction between odom_before and odom_after, odoms_after[self_id] = odom_before + drift
    double low_pass_alpha_x=1; // 低通滤波参数, draft_delta = clamp(draft_new - draft_old, -alpha, alpha)
    double low_pass_alpha_y=1;
    double low_pass_alpha_Y=5*M_PI/180;
    double low_pass_beta =0.3; // 低通滤波参数，draft = draft_old + beta * draft_delta
 
    // callbacks
    void OdomAfterCb(const nav_msgs::OdometryConstPtr &msg, int uav_id);    // callback for odometries after calibration
    void OdomBeforeCb(const nav_msgs::OdometryConstPtr &msg);   // callback for odometry before calibration
    void UwbResultCb(const uwb_msgs::UwbResultStampedConstPtr &msg);   // callback for uwb measurements
    void UpdateOdom();  // update odometry after calibration
    void UpdateDrift(); // update drift estimation using GTSAM
public:
    UwbCaliNode();   // 构造函数
};

UwbCaliNode::UwbCaliNode(){
    cout<<"uwb cali node starting"<<endl;
    nh.param<int>("/uav_num", uav_num, 3);
    nh.param<int>("/bs_num", bs_num, 0);
    nh.param<double>("/simulations/uwb_sim_node/timer_duration", uwb_frequency, 0);
    uwb_frequency=1/uwb_frequency;
    cout<<"uav_num: "<<uav_num<<endl;
    cout<<"bs_num: "<<bs_num<<endl;

    pnh.param<int>("self_id", self_id, 0);
    pnh.param<double>("walk_sigma_x", walk_sigma_x, 0.05);
    pnh.param<double>("walk_sigma_y", walk_sigma_y, 0.05);
    pnh.param<double>("walk_sigma_Y", walk_sigma_Y, 2);  // in degree
    pnh.param<double>("sigma_r", sigma_r, 0.1);
    pnh.param<double>("sigma_Y", sigma_Y, 5);
    pnh.param<double>("sigma_P", sigma_P, 10);

    low_pass_alpha_x = walk_sigma_x*sqrt(1/uwb_frequency)*10; // 低通滤波参数, draft_delta = clamp(draft_new - draft_old, -alpha, alpha)
    low_pass_alpha_y = low_pass_alpha_x;
    low_pass_alpha_Y = walk_sigma_Y*sqrt(1/uwb_frequency)*10;
    low_pass_beta = 0.5; // 低通滤波参数，draft = draft_old + beta * draft_delta

    driftcorrection.x=driftcorrection.y=driftcorrection.theta=0;

    // initialize vectors
    odom_name_before= "/drone" + std::to_string(self_id) + "/global_odom_sim";
    uwb_measurements_name = "/drone" + std::to_string(self_id) + "/uwb_sim_result";
    odom_names_after.resize(uav_num);
    odom_after_subs.resize(uav_num);
    odoms_after.resize(uav_num);
    bs_gt_poses.resize(bs_num);
    
    // initialize basestation positions(stationary)
    for(int i=0; i<bs_num; i++){
        nh.param<double>("/bs"+std::to_string(i)+"_x", bs_gt_poses[i].position.x, 0.0);
        nh.param<double>("/bs"+std::to_string(i)+"_y", bs_gt_poses[i].position.y, 0.0);
        nh.param<double>("/bs"+std::to_string(i)+"_z", bs_gt_poses[i].position.z, 0.0);
        double R,P,Y;
        nh.param<double>("/bs"+std::to_string(i)+"_R", R, 0.0);
        nh.param<double>("/bs"+std::to_string(i)+"_P", P, 0.0);
        nh.param<double>("/bs"+std::to_string(i)+"_Y", Y, 0.0);
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(R, P, Y);
        bs_gt_poses[i].orientation=tf2::toMsg(myQuaternion);
        cout<<"bs"<<i<<"_x: "<<bs_gt_poses[i].position.x<<endl;
        cout<<"bs"<<i<<"_y: "<<bs_gt_poses[i].position.y<<endl;
        cout<<"bs"<<i<<"_z: "<<bs_gt_poses[i].position.z<<endl;
        cout<<"bs"<<i<<"_R: "<<R<<endl;
        cout<<"bs"<<i<<"_P: "<<P<<endl;
        cout<<"bs"<<i<<"_Y: "<<Y<<endl;
    }

    for(int i=0; i<uav_num; i++){
        odom_names_after[i] = "/drone" + std::to_string(i) + "/global_odom_cali";
        if(i==self_id) continue;
        odom_after_subs[i] = nh.subscribe<nav_msgs::Odometry>(odom_names_after[i], 5, boost::bind(&UwbCaliNode::OdomAfterCb, this, _1, i));
    }

    odom_before_sub = nh.subscribe<nav_msgs::Odometry>(odom_name_before, 5, &UwbCaliNode::OdomBeforeCb, this);
    uwb_result_sub = nh.subscribe<uwb_msgs::UwbResultStamped>(uwb_measurements_name, 5, &UwbCaliNode::UwbResultCb, this);
    odom_after_pub = nh.advertise<nav_msgs::Odometry>(odom_names_after[self_id], 5);
    cout<<"uwb cali node has started"<<endl;
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
    if(protective_counter<protective_count){
        protective_counter++;
        return;
    }
    // update drift correction
    UpdateDrift();
    return;
}

void UwbCaliNode::UpdateOdom(){
    nav_msgs::Odometry odom_temp = odom_before;   // in order to maintain continuity of odoms_after[self_id]
    double odom_RPY_temp[3]={0,0,0};    // in order to maintain continuity of odoms_after[self_id]
    // update odometry after calibrationprior_noise_factor
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
    double self_xyz_now[3]={odoms_after[self_id].pose.pose.position.x,
    odoms_after[self_id].pose.pose.position.y,odoms_after[self_id].pose.pose.position.z};
    double self_RPY_now[3]={odom_after_RPY[0],odom_after_RPY[1],odom_after_RPY[2]};
    double prior_noise_factor= sqrt(0.5);
    double prior_noise_xyY[3]={walk_sigma_x*prior_noise_factor,walk_sigma_y*prior_noise_factor,walk_sigma_Y*prior_noise_factor};  // needs to be tuned

    // noise models
    double rangenoise=sigma_r;
    double elevnoise=sigma_P;
    double yawnoise=sigma_Y;
    noiseModel::Diagonal::shared_ptr rnmodel = noiseModel::Diagonal::Sigmas(Vector1(rangenoise));
    noiseModel::Diagonal::shared_ptr enmodel = noiseModel::Diagonal::Sigmas(Vector1(elevnoise));
    noiseModel::Diagonal::shared_ptr ynmodel = noiseModel::Diagonal::Sigmas(Vector1(yawnoise));
    
    NonlinearFactorGraph graph;
    Pose2 priorMean(self_xyz_now[0], self_xyz_now[1], self_RPY_now[2]);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(prior_noise_xyY[0], prior_noise_xyY[1], prior_noise_xyY[2]));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

   // add factors (uavs)
    for(int i=0; i<uav_num; i++){
        if(i==self_id) continue;
        double other_xyz[3]={odoms_after[i].pose.pose.position.x,
        odoms_after[i].pose.pose.position.y,odoms_after[i].pose.pose.position.z};
        double uwb_range = uwb_measurements.uwb_r[i];
        double uwb_P = uwb_measurements.uwb_P[i];
        double uwb_Y = uwb_measurements.uwb_Y[i];
        graph.add(RangeFactor(1, uwb_range, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], rnmodel));
        graph.add(ElevFactor(1, uwb_P, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], enmodel));
        graph.add(YawFactor(1, uwb_Y, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], ynmodel));
    }
    // add factors (base stations)
    for(int i=0; i<bs_num; i++){
        double other_xyz[3]={bs_gt_poses[i].position.x,bs_gt_poses[i].position.y,bs_gt_poses[i].position.z};
        double uwb_range = uwb_measurements.uwb_r[i+uav_num];
        double uwb_P = uwb_measurements.uwb_P[i+uav_num];
        double uwb_Y = uwb_measurements.uwb_Y[i+uav_num];
        graph.add(RangeFactor(1, uwb_range, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], rnmodel));
        graph.add(ElevFactor(1, uwb_P, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], enmodel));
        graph.add(YawFactor(1, uwb_Y, other_xyz[0], other_xyz[1], other_xyz[2]-self_xyz_now[2], ynmodel));
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
    // low pass filter
    double delta_drift_x = newpose.x() - odom_before.pose.pose.position.x - driftcorrection.x;
    double delta_drift_y = newpose.y() - odom_before.pose.pose.position.y - driftcorrection.y;
    double delta_drift_Y = angle_protect(newpose.theta() - y - driftcorrection.theta);
    delta_drift_x = std::max(std::min(delta_drift_x, low_pass_alpha_x), -low_pass_alpha_x);
    delta_drift_y = std::max(std::min(delta_drift_y, low_pass_alpha_y), -low_pass_alpha_y);
    delta_drift_Y = std::max(std::min(delta_drift_Y, low_pass_alpha_Y), -low_pass_alpha_Y);
    driftcorrection.x += low_pass_beta * delta_drift_x;
    driftcorrection.y += low_pass_beta * delta_drift_y;
    driftcorrection.theta += angle_protect(low_pass_beta * delta_drift_Y);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_cali_node");
    UwbCaliNode node;
    ros::spin();
}