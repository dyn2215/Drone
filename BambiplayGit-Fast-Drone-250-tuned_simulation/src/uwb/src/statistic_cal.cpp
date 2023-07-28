// this ROS node calculates RMSE of uavs' positions and yaws
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>                           

#include <random>
#include <cmath>

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

class StatisticCalNode{
private:
    //nodehandle
    ros::NodeHandle nh, pnh{"~"};

    //configs
    int uav_num{3};
    std::vector<std::string> groundtruth_global_name;
    std::vector<std::string> cali_global_name;
    double timer_duration{0.01};    // 100 Hz

    //subscribers and publishers
    std::vector<ros::Subscriber> groundtruth_global_sub;
    std::vector<ros::Subscriber> cali_global_sub;
    ros::Subscriber start_subscriber;

    std::vector<std_msgs::Float64> SSE_x;
    std::vector<std_msgs::Float64> SSE_y;
    std::vector<std_msgs::Float64> SSE_Y;
    std::vector<ros::Publisher> RMSE_pub_x;
    std::vector<ros::Publisher> RMSE_pub_y;
    std::vector<ros::Publisher> RMSE_pub_Y;
    std::vector<std::string> RMSE_pub_x_name;
    std::vector<std::string> RMSE_pub_y_name;
    std::vector<std::string> RMSE_pub_Y_name;

    // timer
    ros::Timer timer; // timer, in order to publish the uwb measurement at a certain frequency
    long long timer_cnt{-1};

    // variables
    std::vector<geometry_msgs::Pose> uav_gt_pos;
    std::vector<geometry_msgs::Pose> uav_cali_pos;

    // callbacks
    void GroundtruthCb(const nav_msgs::OdometryConstPtr &msg, int uav_id);
    void CaliCb(const nav_msgs::OdometryConstPtr &msg, int uav_id);
    void SimTimerCb(const ros::TimerEvent &timerEvent); // Timer回调函数,更新测量值并输出
    void show_msg_Cb(const std_msgs::String::ConstPtr& msg);
public:
    StatisticCalNode();
};

StatisticCalNode::StatisticCalNode(){
    nh.param<int>("/uav_num", uav_num, 3);
    pnh.param<double>("timer_duration", timer_duration, 0.01);

    // initialize vectors
    groundtruth_global_name.resize(uav_num);
    cali_global_name.resize(uav_num);

    groundtruth_global_sub.resize(uav_num);
    cali_global_sub.resize(uav_num);

    RMSE_pub_x.resize(uav_num);
    RMSE_pub_y.resize(uav_num);
    RMSE_pub_Y.resize(uav_num);
    RMSE_pub_x_name.resize(uav_num);
    RMSE_pub_y_name.resize(uav_num);
    RMSE_pub_Y_name.resize(uav_num);
    SSE_x.resize(uav_num);
    SSE_y.resize(uav_num);
    SSE_Y.resize(uav_num);
    uav_gt_pos.resize(uav_num);
    uav_cali_pos.resize(uav_num);

    for(int i=0; i<uav_num; i++){
        groundtruth_global_name[i] = "/drone" + std::to_string(i) + "/odom_true";
        cali_global_name[i] = "/drone" + std::to_string(i) + "/global_odom_cali";
        RMSE_pub_x_name[i]= "/drone" + std::to_string(i) + "/RMSE_x";
        RMSE_pub_y_name[i]= "/drone" + std::to_string(i) + "/RMSE_y";
        RMSE_pub_Y_name[i]= "/drone" + std::to_string(i) + "/RMSE_Y";
        // subscriber
        groundtruth_global_sub[i] = nh.subscribe<nav_msgs::Odometry>(groundtruth_global_name[i], 5, boost::bind(&StatisticCalNode::GroundtruthCb, this, _1, i));
        cali_global_sub[i] = nh.subscribe<nav_msgs::Odometry>(cali_global_name[i], 5, boost::bind(&StatisticCalNode::CaliCb, this, _1, i));
        // publish uwb_sim_range and uwb_sim_Y, uwb_sim_P
        RMSE_pub_x[i] = nh.advertise<std_msgs::Float64>(RMSE_pub_x_name[i], 5);
        RMSE_pub_y[i] = nh.advertise<std_msgs::Float64>(RMSE_pub_y_name[i], 5);
        RMSE_pub_Y[i] = nh.advertise<std_msgs::Float64>(RMSE_pub_Y_name[i], 5);
    }
    start_subscriber = nh.subscribe<std_msgs::String>("/start",10,&StatisticCalNode::show_msg_Cb,this);
    timer = nh.createTimer(ros::Duration(timer_duration), &StatisticCalNode::SimTimerCb, this);
}

void StatisticCalNode::SimTimerCb(const ros::TimerEvent &timerEvent) {
    // std::cout<<"SimTimerCb init"<<std::endl;
    if(timer_cnt<0)
        return;
    timer_cnt++;
    for(int i=0;i<uav_num;i++){
        SSE_x[i].data+=pow(uav_gt_pos[i].position.x-uav_cali_pos[i].position.x,2);
        SSE_y[i].data+=pow(uav_gt_pos[i].position.y-uav_cali_pos[i].position.y,2);
        
        double gt_R,gt_P,gt_Y, cali_R,cali_P,cali_Y;
        tf2::Quaternion gtquat;
        tf2::convert(uav_gt_pos[i].orientation,gtquat);
        tf2::Matrix3x3 m(gtquat);
        m.getRPY(gt_R, gt_P, gt_Y);//进行转换
        tf2::Quaternion caliquat;
        tf2::convert(uav_cali_pos[i].orientation,caliquat);
        tf2::Matrix3x3 n(caliquat);
        n.getRPY(cali_R, cali_P, cali_Y);//进行转换
        
        SSE_Y[i].data+=pow(angle_protect(gt_Y-cali_Y),2);
        std_msgs::Float64 RMSE_x,RMSE_y,RMSE_Y;
        RMSE_x.data=sqrt(SSE_x[i].data/timer_cnt);
        RMSE_y.data=sqrt(SSE_y[i].data/timer_cnt);
        RMSE_Y.data=sqrt(SSE_Y[i].data/timer_cnt);
        RMSE_pub_x[i].publish(RMSE_x);
        RMSE_pub_y[i].publish(RMSE_y);
        RMSE_pub_Y[i].publish(RMSE_Y);
    }
    return;
}

void StatisticCalNode::show_msg_Cb(const std_msgs::String::ConstPtr& msg)
{
    std::cout<<"Statistic results start!\n";
    timer_cnt=0;
    return;
}

void StatisticCalNode::GroundtruthCb(const nav_msgs::OdometryConstPtr &msg, int uav_id){
    uav_gt_pos[uav_id].position=msg->pose.pose.position;
    uav_gt_pos[uav_id].orientation=msg->pose.pose.orientation;
    return;
}

void StatisticCalNode::CaliCb(const nav_msgs::OdometryConstPtr &msg, int uav_id){
    uav_cali_pos[uav_id].position=msg->pose.pose.position;
    uav_cali_pos[uav_id].orientation=msg->pose.pose.orientation;
    return;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_sim_node");
    StatisticCalNode node;
    // std::cout << "uwb_sim_node started!" << std::endl;
    ros::spin();
    return 0;
}