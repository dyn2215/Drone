# UWB package 说明文档
## 一、概述
本package针对多无人机路径规划中的定位误差（来源于两部分误差：初始无人机姿态摆放误差和VIO里程计偏移误差）问题，通过无人机之间基于UWB天线的``三维``测距测角，使用基于GTSAM的因子图方法进行位姿修正，包含以下功能：
* 分别仿真无人机姿态摆放误差和VIO里程计偏移误差，考虑x,y,yaw
* 仿真无人机之间基于UWB天线的测距测角结，使用AWGN
* 通过GTSAM进行位姿计算，通过自定义Factor完成
* 自动发布EGO-SWARM算法的目标位置,输出累积RMSE误差


值得注意的是，目前位姿优化为2维平面上的Pose2优化（不考虑z轴误差，不考虑Roll,Pitch误差），为分布式优化（每台无人机运行一个估计节点），且仅仅考虑了当前时刻的测量信息；除此之外，真实世界不存在的节点（如uwb测量模拟，位姿漂移模拟）均只存在于命名空间\simulations下。无人机位姿漂移模型如下：
> Global_odom(𝐱)为全局坐标，local_odom(𝐲)为无人机vio得到的局部坐标，为两个固定坐标系，通过线性变换联系(𝐱=𝐀𝐲+𝐛)
> 
> **无人机定位偏差：** 
初始位置姿态偏差->影响Global_odom与local_odom的变换关系,位置偏差影响𝐛,姿态偏差影响𝐀。 
采取Gaussian建模，考虑x,y与yaw（其余三个自由度在起飞前定死），这一步在gazebo配置中完成。 
> 
> **飞行过程中VIO偏差：**
误差影响在local_odom中，通过线性变换（初始姿态偏差决定）影响global_odom。 
由于local_odom仿真环境中为mavros真值，编写ROS节点添加噪声，使用WGN，考虑x，y和yaw上的漂移（以后可能需要考虑z）。 
> 
> ``Ego-swarm仅仅关注global_dom ``

UWB测量机理如下：

> 测距周期10ms,每一个time stamp uwb节点i发信号，其他M-1个uwb节点得到对i的三维测角，同时通过算法更新所有测距，也就是说测角结果更新周期为M*10 ms，而测距发布周期为10ms

GTSAM相关资料见：

> 1. 官方文档：https://gtsam.org/ ，有简单cpp+matlab例程，包含常见问题解答
> 
> 2. 入门视频：https://www.bilibili.com/video/BV1C4411772G/?spm_id_from=333.337.search-card.all.click&vd_source=bdd100a752aa0d614bf6d33ff5cf420d 强烈推荐，手把手把基本概念过了一遍，建议配合github仓库 https://github.com/dongjing3309/gtsam-examples 学习
> 
> 3. 一个wiki：https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/index.html

## 二、组成与结构

节点话题关系总览：

<img src=".\graphs\overview.png" alt="overview" style="zoom: 50%;" />

其中，与仿真相关节点（真实不存在）均在命名空间/simulations下：

<img src=".\graphs\simulations.png" alt="simulations" style="zoom:50%;" />

真实环境中仅仅包含uwb校准节点，在命名空间/droneX下（X为无人机编号，下同）

<img src=".\graphs\droneX.png" alt="droneX" style="zoom:50%;" />

本package目前包括5个.cpp，一个.hpp：


1. **drift_node.cpp**：模拟飞机在相对坐标下，x,y,yaw上的偏移，使用高斯随机游走模型。在/simulations/droneX（即每架飞机对应一个节点）命名空间下，订阅/droneX/mavros/local_position/odom（仿真中为真实值），输出/droneX/drift_sim（nav_msgs::Odometry）为模拟的偏移，/droneX/local_odom_sim（nav_msgs::Odometry）为模拟的相对坐标系内的odom，/droneX/global_odom_sim（nav_msgs::Odometry）为模拟的绝对坐标系中的odom，实际将/droneX/global_odom_sim接入至ego planner。

   <img src=".\graphs\driftnode.png" alt="image-20230801172159455" style="zoom:50%;" />

1. **GT_publisher_node**：发布GroundTruth绝对坐标至/droneX/odom_true(nav_msgs::Odometry)，此GT odometry用于统计指标计算节点statistic_cal.cpp和uwb仿真节点uwb_sim.cpp。

   <img src=".\graphs\gtpublisher.png" alt="gtpublisher" style="zoom:50%;" />

1. **uwb_sim.cpp**：模拟飞机各个飞机和固定基站的uwb测量，分别对无人机和固定基站输出话题/droneX/uwb_sim_result（UwbResultStamped）和/bsX/uwb_sim_result（UwbResultStamped）。消息类型UwbResultStamped定义如下：

   > std_msgs/Header header
   > int32 self_id
   > int32[] uwb_ids
   > float32[] uwb_r
   > float32[] uwb_Y
   > float32[] uwb_P

   其中，self_id为该uwb天线编号（默认0 ~ N-1为无人机，N ~ M-1为固定基站），uwb_ids存储所有uwb天线的编号（默认为0-M-1的一个固定数组），uwb_r,uwb_Y和uwb_P数组长度与uwb_ids相同分别发布``在self_id对应三维坐标系``中，uwb_ids中对应各天线的range，Yaw和Pitch测量结果（三维测角），仿真利用odom_true提供的真值。注意``在self_id对应三维坐标系``

   <img src=".\graphs\uwb_sim.png" alt="uwb_sim" style="zoom:50%;" />

1. **2DRYEFactors.hpp & uwb_cali.cpp**：在2DRYEFactors.hpp中，使用GTSAM库引入自定义Factor: RangeFactor, YawFactor和ElevFactor。uwb_cali.cpp对应/droneX/uwb_cali_node节点，采用分布式方式，每个无人机运行一个节点，使用因子图方法结合2DRYEFactors.hpp中自定义的Factor，输出校准结果/droneX/global_odom_cali (nav_msgs::Odometry)，其订阅其余无人机的校准结果，和自身模拟得到的绝对坐标（/droneX/global_odom_sim）与uwb测量（/droneX/uwb_sim_result）。

   <img src=".\graphs\uwbcali.png" alt="uwbcali" style="zoom:50%;" />

1. **target_pub.cpp**：一个简单的节点，订阅/start(std_msgs::String)，只要/start中发布消息，像/drone_X_ego_planner_node/move_base_simple/goal中发布目标姿态，开始任务。

   <img src=".\graphs\targetpub.png" alt="targetpub" style="zoom:50%;" />

1. **statistic_cal.cpp**：一个简单的节点，订阅三架飞机分别的GT全局坐标，校准后的全局坐标，/start，当接收到/start后开始计算并输出RMSE。

   <img src=".\graphs\statistic_cal.png" alt="statistic_cal" style="zoom:50%;" />

## 三、 使用方法与可调参数

使用shfiles目录下的"autoswarm.sh"启动仿真，在此shell中决定了无人机实际初始姿态RMSE与实际姿态，并写入同目录下的init_pos.txt中:

<img src=".\graphs\inittxt.png" alt="inittxt" style="zoom:50%;" />

在autoswarm.sh中使用roslaunch uwb_launch_all.launch启动全部uwb节点，其中列出了所有可以传入节点的参数。

## 四、下一步修改方向

/simulations下的各节点基本完善（可能uwb_msgs最终形式并不是分布式的，这一点需要后续明确），主要下一步改uwb_cali节点，目前优化方法十分trivial，且目前是分布式的，后面可能根据uwb_msgs的形式改成集中式的。