// this ROS node aims to simulate the VIO draft using White Gaussian Noise
// input is from rostopic dronei/mavros, output is the topic dronei/local_position and dronei/draft
// only x,y and yaw are considered in the draft, z, roll and pitch are ignored

// #include <fmt/format.h>
// #include <ceres/ceres.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtsam/geometry/Pose3.h>

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

class DraftNode {
private:
    //nodehandle
    ros::NodeHandle nh, pnh{"~"};

    //configs
    int self_id{0};
    std::string groundtruth_name;
    std::string local_postion_name;
    std::string global_postion_name;
    std::string draft_name;
    double timer_duration{0.02};
    // delta_x = N(0,sigma_x) * sqrt(delta_t)
    double sigma_x{0.1};
    double sigma_y{0.1};
    double sigma_Y{0.2}; // in degree
    double start_time{50}; // the time when the draft starts
    double init_Y{0};   // the initial yaws
    double init_x{0};
    double init_y{0};

    //subscribers and publishers
    ros::Subscriber groundtruth_sub;
    ros::Publisher local_position_pub;
    ros::Publisher global_position_pub;
    ros::Publisher draft_sim_pub;

    // timer
    ros::Timer timer; // timer, in order to publish the draft at a fixed rate
    int timer_cnt = 0;

    // variables
    nav_msgs::Odometry draft_sim; // 漂移的模拟值
    nav_msgs::Odometry local_position_sim; // 模拟（真实值+漂移）
    nav_msgs::Odometry global_position_sim; // 模拟（真实值+漂移）
    double draft_x, draft_y, draft_Y; // 漂移的模拟值的x,y,yaw,引入此变量是由于geometry_msgs::Point中不采用欧拉角，而是采用四元数
    std::vector<float> uavInitPos_; //初始无人机位姿,为[0x,0y,1x,1y,2x,2y,0Y,1Y,2Y]

    // callbacks
    void SimDraftCb(const ros::TimerEvent &timerEvent); // 漂移的模拟值的回调函数
    void GroundtruthCb(const nav_msgs::OdometryConstPtr &msg); // 无漂移的真实值的回调函数

public:
    DraftNode();    
};

DraftNode::DraftNode(){
    pnh.param<int>("self_id", self_id, 0);
    pnh.param<double>("sigma_x", sigma_x, 0);
    pnh.param<double>("sigma_y", sigma_y, 0);
    pnh.param<double>("sigma_Y", sigma_Y, 0);
    pnh.param<double>("start_time", start_time, 50);
    pnh.param<double>("timer_duration", timer_duration, 0.02);
    nh.param("/uav_init_pos", uavInitPos_, std::vector<float>(0));
    init_x=uavInitPos_[2*self_id];
    init_y=uavInitPos_[1+2*self_id];
    init_Y=uavInitPos_[6+self_id];

    groundtruth_name="/drone"+std::to_string(self_id)+"/mavros/local_position/odom";
    local_postion_name="/drone"+std::to_string(self_id)+"/local_position_sim";
    global_postion_name="/drone"+std::to_string(self_id)+"/global_position_sim";
    draft_name="/drone"+std::to_string(self_id)+"/draft_sim";

    groundtruth_sub=nh.subscribe<nav_msgs::Odometry>(groundtruth_name, 5, &DraftNode::GroundtruthCb, this);

    local_position_pub=nh.advertise<nav_msgs::Odometry>(local_postion_name, 5);
    global_position_pub=nh.advertise<nav_msgs::Odometry>(global_postion_name, 5);
    draft_sim_pub=nh.advertise<nav_msgs::Odometry>(draft_name, 5);

    timer = nh.createTimer(ros::Duration(timer_duration), &DraftNode::SimDraftCb, this);
}

 void DraftNode::SimDraftCb(const ros::TimerEvent &timerEvent) {
    timer_cnt += 1;
    if (timer_cnt < start_time/timer_duration)
        return;

    draft_x += randn(0, sigma_x) * sqrt(timer_duration);
    draft_y += randn(0, sigma_y) * sqrt(timer_duration);
    draft_Y += randn(0, sigma_Y) * sqrt(timer_duration) * M_PI / 180.0;
}

void DraftNode::GroundtruthCb(const nav_msgs::OdometryConstPtr &msg) {  
    // add the draft in x,y,yaw to the groundtruth and publish local_position_sim and draft_sim
    // add the draft in x,y
    local_position_sim=*msg;
    local_position_sim.pose.pose.position.x += (draft_x);
    local_position_sim.pose.pose.position.y += (draft_y);
    
    // add the draft in yaw
    // first, convert the groundtruth quaternion to euler
    double groundtruth_yaw, groundtruth_pitch, groundtruth_roll;

    //quaternion2euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, groundtruth_yaw, groundtruth_pitch, groundtruth_roll);
    tf2::Quaternion gtquat;
    tf2::convert(msg->pose.pose.orientation,gtquat);
    tf2::Matrix3x3 m(gtquat);

    m.getRPY(groundtruth_roll, groundtruth_pitch, groundtruth_yaw);//进行转换
    // then, add the draft in yaw
    groundtruth_yaw += draft_Y-init_Y;
    // finally, convert the euler to quaternion
    // euler2quaternion(groundtruth_yaw, groundtruth_pitch, groundtruth_roll, local_position_sim.pose.pose.orientation.x, local_position_sim.pose.pose.orientation.y, local_position_sim.pose.pose.orientation.z, local_position_sim.pose.pose.orientation.w);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(groundtruth_roll, groundtruth_pitch, groundtruth_yaw);
    local_position_sim.pose.pose.orientation=tf2::toMsg(myQuaternion);

    // publish local_position_sim
    local_position_pub.publish(local_position_sim);

    //publish global_position_sim
    global_position_sim=local_position_sim;
    global_position_sim.header.frame_id="world";
    global_position_sim.child_frame_id="";
    global_position_sim.pose.pose.position.y+=self_id;
    global_position_pub.publish(global_position_sim);

    // get draft_sim
    draft_sim.header=msg->header;
    draft_sim.child_frame_id=msg->child_frame_id;
    draft_sim.pose.pose.position.x = draft_x;
    draft_sim.pose.pose.position.y = draft_y;    
    // euler2quaternion(draft_Y, 0, 0, draft_sim.pose.pose.orientation.x, draft_sim.pose.pose.orientation.y, draft_sim.pose.pose.orientation.z, draft_sim.pose.pose.orientation.w);
    myQuaternion.setRPY(0, 0, draft_Y);
    draft_sim.pose.pose.orientation=tf2::toMsg(myQuaternion);

    // publish draft_sim
    draft_sim_pub.publish(draft_sim);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "draft_node");
    DraftNode node;
    ros::spin();
}