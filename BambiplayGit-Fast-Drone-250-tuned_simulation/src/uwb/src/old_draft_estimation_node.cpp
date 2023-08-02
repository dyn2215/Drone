#include <fmt/format.h>

#include <ceres/ceres.h>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <uwb_msgs/UwbResultStamped.h>
#include <nav_msgs/Odometry.h>

struct UwbError {
    UwbError(double x, double y, double z, double d)
            : x(x), y(y), z(z), d(d) {}

    template<typename T>
    bool operator()(const T *const point,
                    T *residuals) const {
        T d_ = ceres::sqrt((point[0] - x) * (point[0] - x) +
                           (point[1] - y) * (point[1] - y) +
                           (point[2] - z) * (point[2] - z));
        residuals[0] = d_ - d;
        return true;
    }

    static ceres::CostFunction *Create(
            const double x, const double y, const double z, const double d) {
        return (new ceres::AutoDiffCostFunction<UwbError, 1, 3>(new UwbError(x, y, z, d)));
    }

    double x, y, z, d;

};

struct PriorError {
    PriorError(double x, double y, double z)
            : x(x), y(y), z(z) {}

    template<typename T>
    bool operator()(const T *const point,
                    T *residuals) const {
        residuals[0] = point[0] - x;
        residuals[1] = point[1] - y;
        residuals[2] = point[2] - z;
        return true;
    }

    static ceres::CostFunction *Create(
            const double x, const double y, const double z) {
        return (new ceres::AutoDiffCostFunction<PriorError, 3, 3>(new PriorError(x, y, z)));
    }

    double x, y, z;
};

class DraftEstimationNode {

private:
    // ros node handle
    ros::NodeHandle nh, pnh{"~"};

    // configs
    int uav_num{3}; // 无人机数量
    int self_id{0}; // 自己的id
    double low_pass_alpha = 10.0; // 低通滤波参数, draft_delta = clamp(draft_new - draft_old, -alpha, alpha)
    double low_pass_beta = 0.1; // 低通滤波参数，draft = draft_old + beta * draft_delta
    double init_x, init_y, init_z;

    std::string uwb_result_topic{"/drone0/uwb_result"};
    std::string uav_global_odom_topic{"/drone{:d}/odom_true"};
    std::string self_local_odom_topic{"/drone0/mavros/local_position/odom"};
    std::string draft_topic{"/draft"};

    // 发布漂移校准前后的无人机位置，用于可视化，验证漂移校正模块
    std::string self_odom_before_est_topic{"/drone{:d}/odom_before_est"};
    std::string self_odom_after_est_topic{"/drone{:d}/odom_after_est"};
    ros::Publisher self_odom_before_est_pub, self_odom_after_est_pub;

    std::vector <geometry_msgs::PointStamped> uav_global_positions; // 记录所有无人机世界坐标下的位置（校准后的位置）
    geometry_msgs::PointStamped self_local_position; // 记录无人机自身在局部坐标下的位置（校准前的位置）

    geometry_msgs::Point draft, draft_sim; // 漂移的估计值

    ros::Subscriber uwb_result_sub; // 订阅UWB信息
    std::vector <ros::Subscriber> uav_global_odom_subs; // 订阅所有无人机世界坐标下的位置（校准后的位置）
    ros::Subscriber self_local_odom_sub;
    // ros::Publisher self_global_odom_pub;
    ros::Publisher draft_pub;

    ros::Timer timer; // 定时器，用于模拟VIO漂移
    int timer_cnt = 0;

    void uwbResultCb(const uwb_msgs::UwbResultStampedConstPtr &msg);

    void uavGlobalOdomCb(const nav_msgs::OdometryConstPtr &msg, int uav_id);

    void selfLocalOdomCb(const nav_msgs::OdometryConstPtr &msg);

    void simDraftCb(const ros::TimerEvent &timerEvent);

public:
    DraftEstimationNode();
};


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "draft_estimation_node");
    DraftEstimationNode node;
    ros::spin();
}

DraftEstimationNode::DraftEstimationNode() {

    // 获取ros参数
    nh.param<int>("/uav_num", uav_num, 3);
    pnh.param<int>("self_id", self_id, 0);
    pnh.param<std::string>(
            "self_local_odom_topic", self_local_odom_topic, "/drone0/mavros/local_position/odom");
    pnh.param<double>("low_pass_beta", low_pass_beta, low_pass_beta);

    init_y = self_id * 1.5 - 0.5;
    init_x = 0.7 - self_id * 0.5;
    
    // 初始化初始漂移估计值
    pnh.param<double>("init_x", init_x, init_x);
    pnh.param<double>("init_y", init_y, init_y);
    pnh.param<double>("init_z", init_z, init_z);
    draft.x = init_x;
    draft.y = init_y;
    draft.z = init_z;

    uav_global_positions.resize(uav_num);

    // self_global_odom_pub = nh.advertise<nav_msgs::Odometry>(
    //         fmt::format(uav_global_odom_topic, self_id), 3);
    self_odom_after_est_pub = nh.advertise<geometry_msgs::PointStamped>(
            fmt::format(self_odom_after_est_topic, self_id), 3);
    self_odom_before_est_pub = nh.advertise<geometry_msgs::PointStamped>(
            fmt::format(self_odom_before_est_topic, self_id), 3);

    draft_pub = nh.advertise<geometry_msgs::PointStamped>(
            draft_topic, 3);

    uwb_result_sub = nh.subscribe<uwb_msgs::UwbResultStamped>(
            uwb_result_topic, 1,
            &DraftEstimationNode::uwbResultCb, this);

    for (int uav_id = 0; uav_id < uav_num; uav_id++) {
        uav_global_odom_subs.emplace_back(nh.subscribe<nav_msgs::Odometry>(
                fmt::format(uav_global_odom_topic, uav_id), 1,
                boost::bind(&DraftEstimationNode::uavGlobalOdomCb, this, _1, uav_id)));
    }
    self_local_odom_sub = nh.subscribe<nav_msgs::Odometry>(
            self_local_odom_topic, 5,
            &DraftEstimationNode::selfLocalOdomCb, this);

     timer = nh.createTimer(ros::Duration(0.1), &DraftEstimationNode::simDraftCb, this);
}

void DraftEstimationNode::simDraftCb(const ros::TimerEvent &timerEvent) {
    timer_cnt += 1;
    if (timer_cnt < 500)
        return;
    if (self_id == 0) {
        draft_sim.x += 0.003;
        draft_sim.y -= 0.003;
    } else if (self_id == 1) {
        draft_sim.x += 0.003;
    } else if (self_id == 2) {
        draft_sim.x -= 0.003;
        draft_sim.y += 0.003;
    }
}

void DraftEstimationNode::uavGlobalOdomCb(const nav_msgs::OdometryConstPtr &msg, int uav_id) {
    uav_global_positions[uav_id].header = msg->header;
    uav_global_positions[uav_id].point = msg->pose.pose.position;
}

void DraftEstimationNode::selfLocalOdomCb(const nav_msgs::OdometryConstPtr &msg) {
    self_local_position.header = msg->header;
    self_local_position.point = msg->pose.pose.position;
    self_local_position.point.x += draft_sim.x;
    self_local_position.point.y += draft_sim.y;
    self_local_position.point.z += draft_sim.z;

    geometry_msgs::PointStamped point;
    point.point = msg->pose.pose.position;
    point.header = msg->header;

    point.point.x += init_x + draft_sim.x;
    point.point.y += init_y + draft_sim.y;
    point.point.z += init_z + draft_sim.z;

    self_odom_before_est_pub.publish(point);

    point.point.x += draft.x - init_x;
    point.point.y += draft.y - init_y;
    point.point.z += draft.z - init_z;

    self_odom_after_est_pub.publish(point);
}

bool isRecent(ros::Time time) {
    auto now = ros::Time::now();
    // ROS_DEBUG("%d, %d, %d, %d", now.sec, time.sec, now.nsec, time.sec);
    return (now - time).toSec() < 0.2;
}

void DraftEstimationNode::uwbResultCb(const uwb_msgs::UwbResultStampedConstPtr &msg) {
    if (!isRecent(self_local_position.header.stamp)) {
        ROS_ERROR("UAV_%d odom info loss!", self_id);
        return;
    }

    // 记录可以用于优化的无人机id
    std::string info = "";
    std::vector<int> available_ids;
    for (int idx = 0; idx < msg->others_id.size(); idx++) {
        const int uav_id = msg->others_id[idx];
        if (isRecent(uav_global_positions[uav_id].header.stamp)) {
            available_ids.push_back(idx);
            info += std::to_string(uav_id) + " ";
        } else {
            ROS_WARN("On UAV %d, UAV %d global odom is out of date.", self_id, uav_id);
        }
    }
    if (available_ids.size() == 0) {
        ROS_WARN("No info can be used for self draft estimation.");
        return;
    }
    ROS_INFO("Use the following UAVs' info for self draft estimation: %s", info.c_str());

    /*** 构建优化问题 ***/
    ceres::Problem problem;

    double point[3]; // 优化变量
    point[0] = self_local_position.point.x + draft.x;
    point[1] = self_local_position.point.y + draft.y;
    point[2] = self_local_position.point.z + draft.z;

    ceres::CostFunction *prior_error = PriorError::Create(
            point[0],
            point[1],
            point[2]);
    problem.AddResidualBlock(prior_error, nullptr, point);

    for (const int idx: available_ids) {
        const int uav_id = msg->others_id[idx];
        const double distance = msg->others_distance[idx];
        ceres::CostFunction *uwb_error = UwbError::Create(
                uav_global_positions[uav_id].point.x,
                uav_global_positions[uav_id].point.y,
                uav_global_positions[uav_id].point.z,
                distance);
        problem.AddResidualBlock(uwb_error, nullptr, point);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double draft_delta[3];
    draft_delta[0] = point[0] - self_local_position.point.x - draft.x;
    draft_delta[1] = point[1] - self_local_position.point.y - draft.y;
    draft_delta[2] = point[2] - self_local_position.point.z - draft.z;
    draft_delta[0] = std::max(std::min(draft_delta[0], low_pass_alpha), -low_pass_alpha);
    draft_delta[1] = std::max(std::min(draft_delta[1], low_pass_alpha), -low_pass_alpha);
    draft_delta[2] = std::max(std::min(draft_delta[2], low_pass_alpha), -low_pass_alpha);

    draft.x += low_pass_beta * draft_delta[0];
    draft.y += low_pass_beta * draft_delta[1];
    draft.z += low_pass_beta * draft_delta[2];
    draft.z = 0;

    geometry_msgs::PointStamped draft_msg;
    draft_msg.header = msg->header;
    draft_msg.point = draft;
    draft_pub.publish(draft_msg);

//    std::cout << summary.FullReport() << std::endl;
//    std::cout << uav_global_positions[self_id].point << std::endl;
    printf("initial cost: %lf, final cost: %lf, iterations: %d\n",
           summary.initial_cost, summary.final_cost, (int) summary.iterations.size());
    printf("draft: %0.2lf, %0.2lf, %0.2lf\n",
           draft.x, draft.y, draft.z);
//    printf("draft_sim: %0.2lf, %0.2lf, %0.2lf\n",
//           draft_sim.x, draft_sim.y, draft_sim.z);
//    printf("error: %0.2lf, %0.2lf, %0.2lf\n",
//           draft_sim.x + draft.x, draft_sim.y + draft.y, draft_sim.z + draft.z);
    printf("global position: %0.2lf, %0.2lf, %0.2lf\n",
           self_local_position.point.x + draft.x,
           self_local_position.point.y + draft.y,
           self_local_position.point.z + draft.z);

    printf("global position before: %0.2lf, %0.2lf, %0.2lf\n",
           self_local_position.point.x + init_x,
           self_local_position.point.y + init_y,
           self_local_position.point.z + init_z);

    printf("===============================\n");


}
