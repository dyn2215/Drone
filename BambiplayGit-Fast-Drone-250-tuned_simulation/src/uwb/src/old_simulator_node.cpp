#include <random>

#include <fmt/format.h>

#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PointStamped.h>
#include <uwb_msgs/UwbResultStamped.h>


class UwbSimulatorNode {

private:
    // ros node handle
    ros::NodeHandle nh, pnh{"~"};

    // configs
    int uav_num{3}; // number of UAVs
    std::string gazebo_model_type{"px4vision"}; // UAV model name in Gazebo should be e.g. "px4vision_0", "px4vision_1"

    float max_range{40.0}; // max communication range of UWB, unit: m
    float noise_std{0.05}; // 噪声的标准差，单位：m
    float uwb_freq{30.0}; // UWB测量频率，单位：Hz

    // 发布UWB结果的topic名称
    std::string uwb_result_topic{"/drone{:d}/uwb_result"};

    // 订阅和发布
    ros::Subscriber model_states_sub; // 订阅真值

    std::vector <ros::Publisher> uwb_result_pubs; // 发布uwb的估计值

    //// UWB测量模拟： UWB以固定频率$uwb_freq进行测量，每次测量能够给出一台无人机到其他所有无人机的距离
    ros::Timer timer; // 定时器，用于定时发布uwb测量结果
    int pub_id{0}; // 记录发布uwb测量结果的无人机id

    // 记录无人机的位置
    std::vector <geometry_msgs::PointStamped> uav_positions;

    // 随机数生成器，用于模拟UWB测量噪声
    std::default_random_engine generator;

    void modelStateCb(const gazebo_msgs::ModelStatesConstPtr &msg);
    void outputCb(const ros::TimerEvent &timerEvent);

public:
    UwbSimulatorNode();
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_simulator_node");

    UwbSimulatorNode node;

    ros::spin();
}

UwbSimulatorNode::UwbSimulatorNode() {

    // 获取ros参数
    nh.param<int>("/uav_num", uav_num, 3);

    uav_positions.resize(uav_num);

    for (int uav_id = 0; uav_id < uav_num; uav_id++) {
        uwb_result_pubs.emplace_back(nh.advertise<uwb_msgs::UwbResultStamped>(
                fmt::format(uwb_result_topic, uav_id), 5));
    }

    model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 1,
            boost::bind(&UwbSimulatorNode::modelStateCb, this, _1));

    timer = nh.createTimer(ros::Duration(1.0 / uwb_freq), &UwbSimulatorNode::outputCb, this);
}

void UwbSimulatorNode::modelStateCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int id = 0; id < uav_num; id++) {
        std::string name = gazebo_model_type + "_" + std::to_string(id);

        // 获取无人机模型对应的索引
        int idx;
        for (idx = 0; idx < msg->name.size() && msg->name[idx] != name; idx++);

        // 没有找到对应的索引
        if (idx == msg->name.size()) {
            continue;
        }

        // 记录无人机的位置
        uav_positions[id].header.stamp = ros::Time::now();
        uav_positions[id].point = msg->pose[idx].position;
    }
}

bool isRecent(ros::Time time) {
    return (ros::Time::now() - time).toSec() < 0.2;
}

#define sqr(x) (x) * (x)

void UwbSimulatorNode::outputCb(const ros::TimerEvent &timerEvent) {
    if (isRecent(uav_positions[pub_id].header.stamp)) {
        auto msg = uwb_msgs::UwbResultStamped();
        msg.header.stamp = ros::Time::now();
        msg.self_id = pub_id;

        // 模拟UWB测距的噪声
        std::normal_distribution<double> dist(0., noise_std);

        for (int id = 0; id < uav_num; id++) {
            if (id == pub_id) continue;
            if (!isRecent(uav_positions[id].header.stamp)) continue;

            // 计算无人机pub_id到无人机id之间的距离
            double distance = sqrt(
                    sqr(uav_positions[pub_id].point.x - uav_positions[id].point.x) +
                    sqr(uav_positions[pub_id].point.y - uav_positions[id].point.y) +
                    sqr(uav_positions[pub_id].point.z - uav_positions[id].point.z));

            if (distance < max_range) {
                double x = dist(generator);
                msg.others_id.push_back(id);
                msg.others_distance.push_back(distance + x);
            }
        }

        uwb_result_pubs[pub_id].publish(msg);
    }

    pub_id++;
    if (pub_id >= uav_num) pub_id -= uav_num;
}