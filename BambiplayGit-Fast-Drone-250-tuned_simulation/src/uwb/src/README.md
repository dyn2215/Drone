<!-- 联系方式dyn2215@163.com -->
# UWB package 说明文档
## 一、概述
本package针对多无人机路径规划中的定位误差（来源于两部分误差：初始无人机姿态摆放误差和VIO里程计偏移误差）问题，通过无人机之间基于UWB天线的``三维``测距测角，使用基于GTSAM的因子图方法进行位姿修正，包含以下功能：
* 分别仿真无人机姿态摆放误差和VIO里程计偏移误差，考虑x,y,yaw
* 仿真无人机之间基于UWB天线的测距测角结，使用AWGN
* 通过GTSAM进行位姿计算，通过自定义Factor完成
* 自动发布EGO-SWARM算法的目标位置,输出累积RMSE误差

值得注意的是，目前位姿优化为2维平面上的Pose2优化（不考虑z轴误差，不考虑Roll,Pitch误差），为分布式优化（每台无人机运行一个估计节点），且仅仅考虑了当前时刻的测量信息。无人机位姿漂移模型如下：
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

GTSAM相关资料见：
> 1. 官方文档：https://gtsam.org/ ，有简单cpp+matlab例程，包含常见问题解答
> 
> 2. 入门视频：https://www.bilibili.com/video/BV1C4411772G/?spm_id_from=333.337.search-card.all.click&vd_source=bdd100a752aa0d614bf6d33ff5cf420d 强烈推荐，手把手把基本概念过了一遍，建议配合github仓库 https://github.com/dongjing3309/gtsam-examples 学习
> 
> 3. 一个wiki：https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/index.html

## 二、组成与结构
本package目前包括5个.cpp，一个.hpp，作用分别如下

1. drift_node.cpp


## 三、下一步修改方向