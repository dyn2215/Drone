#include <vector>
#include <map>
#include <random>

#include <fmt/format.h>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <uwb_msgs/UwbResultStamped.h>


class UwbDriverNode {

private:
    // ros node handle
    ros::NodeHandle nh, pnh{"~"};

    // configs
    int uav_num{6}; // number of UAVs
    int self_id{0};
    std::vector<int> uwb_id;
    std::map<int, int> uwb_to_uav_id;
    std::string input_topic{"/smnr2/calc_tof"}; // 接收UWB数据的topic名称
    std::string output_topic{"uwb_result"}; // 发布UWB结果的topic名称

    // 订阅和发布
    ros::Subscriber input_sub; // 订阅UWB数据
    ros::Publisher output_pub; // 发布UWB数据

    std::vector <geometry_msgs::Vector3Stamped> msgs;

    void inputCb(const geometry_msgs::Vector3StampedConstPtr &msg);

    void publish();

public:
    UwbDriverNode();

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_bridge_node");

    UwbDriverNode node;

    ros::spin();
}

UwbDriverNode::UwbDriverNode() {
    nh.param<int>("/uav_num", uav_num, 6);
    nh.param < std::vector < int >> ("/uwb_id", uwb_id, std::vector<int>(0));
    pnh.param<int>("self_id", self_id, 0);

    for (int i = 0; i < uwb_id.size(); i++) {
        uwb_to_uav_id.insert(std::pair<int, int>(uwb_id[i], i));
    }

    output_pub = nh.advertise<uwb_msgs::UwbResultStamped>(
            fmt::format(output_topic, self_id), 5);

    input_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
            input_topic, 1, &UwbDriverNode::inputCb, this);
}

void UwbDriverNode::publish() {
    uwb_msgs::UwbResultStamped output_msg;
    output_msg.header = msgs[0].header;
    output_msg.self_id = self_id;

//    ROS_INFO("publish: %d", int(msgs.size()));

    for (auto msg: msgs) {
        if (uwb_to_uav_id.find(int(msg.vector.x))->second == self_id) {
            output_msg.others_id.push_back(uwb_to_uav_id.find(int(msg.vector.y))->second);
        } else {
            output_msg.others_id.push_back(uwb_to_uav_id.find(int(msg.vector.x))->second);
        }
        output_msg.others_distance.push_back(msg.vector.z / 100.0);

//        ROS_INFO("    %d: %lf", output_msg.others_id[output_msg.others_id.size() - 1], msg.vector.z);

    }
    output_pub.publish(output_msg);

    msgs.clear();
}

void UwbDriverNode::inputCb(const geometry_msgs::Vector3StampedConstPtr &msg) {
//    if (msgs.size() > 0) {
//        ros::Time last = msgs[msgs.size() - 1].header.stamp;
//        if ((msg->header.stamp - last).toSec() > 0.005) {
//            publish();
//        }
//    }

    if (uwb_to_uav_id.find(int(msg->vector.x))->second == self_id ||
        uwb_to_uav_id.find(int(msg->vector.y))->second == self_id) {
        msgs.push_back(*msg);
    }

    if (msgs.size() >= uav_num - 1) {
        publish();
    }
}