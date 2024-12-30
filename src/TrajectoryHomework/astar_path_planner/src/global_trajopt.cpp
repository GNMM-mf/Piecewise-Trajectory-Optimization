/*
  Author： Baining Tu
  Date: 2024-12-30
  Reference: https://github.com/ZJU-FAST-Lab/GCOPTER
*/ 


#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "visualizer.hpp"
#include "trajectory.hpp"

#include <std_msgs/Float64.h>

/*
// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

// Step 4: 自行实现轨迹生成类
class TrajectoryGenerator {
    // 轨迹规划的目标是根据A*算法给出的无碰撞路径，计算轨迹航点（控制点），从而生成一条以时间参数化的平滑轨迹，可以用于控制移动机器人跟踪
    // 本次作业中，我们要求生成一条分段多项式轨迹，即每段轨迹均为一个多项式函数
    // 你可以选择使用多项式、B样条、贝塞尔曲线、MINCO等多种轨迹基函数实现
    // 每段轨迹的连接处需要满足一定的连续性条件，如位置连续、速度连续、加速度连续等，这将构成轨迹优化的主要约束条件
    // 轨迹的初始和终止状态为到达指定位置，速度、加速度等状态均为0
    // 优化目标可以是最小化轨迹的加加速度（jerk）或加加加速度（snap），请自行选择合适的优化目标及多项式阶数
    // 本次作业对轨迹段的时间选择不做进一步要求，可以自行选择固定时间或自定义策略生成时间分配
    // 可以任意选用求解器，如qpOASaES、OSQP-Eigen等，也可以自行实现闭式解法
public:
    TrajectoryGenerator() = default;

        // your code
};
*/


struct Config
{ 
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum; // 最大轨迹段数量。

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
        nh_priv.getParam("MaxPieceNum", maxPieceNum);
    }
};

// 这个函数计算一个简单的梯形速度剖面下到达某个距离所需的时间，考虑了给定的速度和加速度。
// 如果目标距离足够短，直接计算加速阶段所需时间；否则计算整个加速、匀速和减速的过程。
double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // computed coefficientMatrix of the minimum-jerk trajectory in this function

    int s = 3;                                     // minimum-jerk
    int m = 3;                                     // 轨迹维数， m=3表示只考察xyz三维的轨迹
    int dim = timeAllocationVector.size() * 2 * s; // dimension of matrix M， M表示全局轨迹的分段数， M = timeAllocationVector.size()
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(dim, m); 

    // get F0 and D0
    Eigen::MatrixXd F0(s, 2 * s);
    Eigen::MatrixXd D0(s, 3);

    F0 << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0;

    D0 << initialPos(0), initialPos(1), initialPos(2),
        initialVel(0), initialVel(1), initialVel(2),
        initialAcc(0), initialAcc(1), initialAcc(2);

    M.block(0, 0, s, 2 * s) << F0;
    b.block(0, 0, s, 3) << D0;

    // get Fi, Ei, and Di, use for loop to fill them into M and b
    for (int i = 0; i < dim - 2 * s; i += 2 * s)
    {
        int index = i / (2 * s);
        double t = timeAllocationVector(index);
        Eigen::MatrixXd Fi(2 * s, 2 * s);
        Eigen::MatrixXd Ei(2 * s, 2 * s);
        Eigen::MatrixXd Di(1, 3);

        Fi << 0, 0, 0, 0, 0, 0,
            -1, 0, 0, 0, 0, 0,
            0, -1, 0, 0, 0, 0,
            0, 0, -2, 0, 0, 0,
            0, 0, 0, -6, 0, 0,
            0, 0, 0, 0, -24, 0;

        Ei << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
            1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
            0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
            0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3),
            0, 0, 0, 6, 24 * t, 60 * pow(t, 2),
            0, 0, 0, 0, 24, 120 * t;

        Di << intermediatePositions.col(index).transpose();

        M.block(s + i, 2 * s + i, 2 * s, 2 * s) << Fi;
        M.block(s + i, i, 2 * s, 2 * s) << Ei;
        b.block(s + i, 0, 1, 3) << Di;
    }

    // get E_M and D_M
    Eigen::MatrixXd E_M(s, 2 * s);
    Eigen::MatrixXd D_M(s, 3);

    double t = timeAllocationVector(timeAllocationVector.size() - 1);

    E_M << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
        0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
        0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

    D_M << terminalPos(0), terminalPos(1), terminalPos(2),
        terminalVel(0), terminalVel(1), terminalVel(2),
        terminalAcc(0), terminalAcc(1), terminalAcc(2);

    M.block(dim - s, dim - 2 * s, s, 2 * s) << E_M;
    b.block(dim - s, 0, s, 3) << D_M;

    // Solve Mc = b, using QR solver
    clock_t time_stt = clock();
    for (int i = 0; i < 3; i++)
    {
        coefficientMatrix.col(i) = M.colPivHouseholderQr().solve(b.col(i));
        // coefficientMatrix.col(i) = M.lu().solve(b.col(i));
    }

    // std::cout << "C is " << coefficientMatrix << std::endl;
    std::cout << "Trajopt Algorithm Time Cost = " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;

}



class GlobalTrajOpt // GlobalTrajOpt 类处理A*全局路径的接收、轨迹分段、轨迹优化计算和可视化。
{
private: // 成员变量
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber astarpathSub; // 订阅A*路径的订阅器
    Visualizer visualizer; // 可视化器，用于显示轨迹。
    Eigen::Matrix3Xd positions; // 保存目标点的矩阵（3 行，最多 maxPieceNum + 1 列）
    Eigen::VectorXd times; // 保存每个轨迹段的时间分配
    int positionNum; // 目标点的数量
    Trajectory<5> traj; // 存储轨迹的容器

    ros::Publisher velPub; // 发布速度时间图像
    ros::Publisher accPub; // 发布加速度时间图像
    ros::Publisher jerPub; // 发布加加速度时间图像

public: 
    GlobalTrajOpt(const Config &conf, ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh),
          positions(3, config.maxPieceNum + 1),
          times(config.maxPieceNum),
          positionNum(20)  // 初始化所有成员变量，订阅 targetTopic 主题并设定回调函数。
    {
        astarpathSub = nh.subscribe<nav_msgs::Path>("astar_path", 1,
                                 &GlobalTrajOpt::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay()); //订阅初始全局路径，执行回调函数targetCallBack
    
        velPub = nh.advertise<std_msgs::Float64>("vel_time_plot", 1); // Initialize velocity publisher
        accPub = nh.advertise<std_msgs::Float64>("acc_time_plot", 1); // Initialize acceleration publisher
        jerPub = nh.advertise<std_msgs::Float64>("jer_time_plot", 1); // Initialize jerk publisher
    }

    void targetCallBack(const nav_msgs::Path::ConstPtr &msg)
    /*
    当接受到A*规划的路径时，将轨迹分段，获得分段的点
    对于每一段轨迹，计算从上一个分段节点到当前分段节点所需的时间，生成最小间隔轨迹。
    最后，用visualization_msgs::Marker 来发布， 对生成的轨迹进行可视化
    参数说明：
    maxPieceNum: 最多分段节点数量
    positionNum: 实际分段节点数量
    */
    {
        // 获取路径中的所有位姿
        const std::vector<geometry_msgs::PoseStamped>& poses = msg->poses;

        // 获得所有在轨迹上的分段节点坐标，实际分段节点数量为positionNum,实际分段数为pieceNum
        // 选取节点, 第一个节点和最后一个节点不变，中间的均匀采样，每小段长度为piecelength
        int piecelength = (poses.size() - 2)/(positionNum - 1);
        //std::cout<< piecelength <<std::endl;
        //std::cout<< positionNum <<std::endl;
        for (int i = 0; i < poses.size(); ++i) {
            const geometry_msgs::PoseStamped& pose = poses[i];
            if( i == 0 ){
                positions(0, i) = pose.pose.position.x;
                positions(1, i) = pose.pose.position.y;
                positions(2, i) = pose.pose.position.z;  // 如果时空间曲线则用 std::fabs(msg->pose.orientation.z) * 0; 
            }
            if( i == poses.size() - 1 ){
                positions(0, positionNum-1) = pose.pose.position.x;
                positions(1, positionNum-1) = pose.pose.position.y;
                positions(2, positionNum-1) = pose.pose.position.z;  // 如果时空间曲线则用 std::fabs(msg->pose.orientation.z) * 0;                 
            }
            if(( i !=0 ) && ( i % piecelength == 0 )){
                positions(0, i/piecelength) = pose.pose.position.x;
                positions(1, i/piecelength) = pose.pose.position.y;
                positions(2, i/piecelength) = pose.pose.position.z;  // 如果时空间曲线则用 std::fabs(msg->pose.orientation.z) * 0;  
            }
        }

        /*
        // 路径长度
        std::cout << "path_size: " << poses.size() << std::endl;
        // 测试所有路径节点
        for (int i = 0; i < poses.size(); i=i+10) {
            const geometry_msgs::PoseStamped& pose = poses[i];

            // 打印位置信息 (position)
            ROS_INFO("Pose %zu - Position: [x: %f, y: %f, z: %f]", 
                    i, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

            // 打印姿态信息 (orientation, 四元数)
            ROS_INFO("Pose %zu - Orientation: [x: %f, y: %f, z: %f, w: %f]", 
                    i, pose.pose.orientation.x, pose.pose.orientation.y, 
                    pose.pose.orientation.z, pose.pose.orientation.w);
        }
        // 测试选取的节点
        for(int i=0; i < positionNum; ++i){
            ROS_INFO("Pose %zu - Position: [x: %f, y: %f, z: %f]", 
                    i, positions(0,i), positions(1,i), positions(2,i));
        }
        }
        */

        if (positionNum > config.maxPieceNum)
        {
            positionNum = 0;
            traj.clear();
        }
        // 计算每段分段轨迹的时间
        for(int i=0; i < positionNum; ++i){
            const double dist = (positions.col(i+1) - positions.col(i)).norm();
            times(i) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
        }

        if (positionNum > 1)
        {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        // std::cout<<"show trajopt result"<<std::endl;
        visualizer.visualize(traj, positions.leftCols(positionNum)); //可视化所选节点（红色）和优化之后的轨迹（蓝色）

        // Draw velocity-time, acceleration-time, jerk-time plot
        for (double t = 0; t < traj.getTotalDuration(); t += 0.01) // Loop through trajectory time
        {
            Eigen::Vector3d velocity = traj.getVel(t); // Get velocity at time t
            Eigen::Vector3d acceleration = traj.getAcc(t); // Get acceleration at time t
            Eigen::Vector3d jerk = traj.getJer(t); // Get jerk at time t

            // Publish velocity (magnitude or individual components)
            std_msgs::Float64 velMsg;
            velMsg.data = velocity.norm(); // You can publish the magnitude of the velocity
            velPub.publish(velMsg);

            // Publish acceleration (magnitude or individual components)
            std_msgs::Float64 accMsg;
            accMsg.data = acceleration.norm(); // You can publish the magnitude of the acceleration
            accPub.publish(accMsg);

            // Publish jerk (magnitude or individual components)
            std_msgs::Float64 jerMsg;
            jerMsg.data = jerk.norm(); // You can publish the magnitude of the jerk
            jerPub.publish(jerMsg);

            ros::Duration(0.01).sleep(); // Pause between each publish
        }

        return;
    }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_trajopt_node");
    ros::NodeHandle nh_;
    
    GlobalTrajOpt globaltajopt(Config(ros::NodeHandle("~")),nh_);
    
    ros::spin();
    return 0;
}
