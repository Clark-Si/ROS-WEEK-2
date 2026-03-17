#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

#define _USE_MATH_DEFINES

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;

// 全局变量
double current_yaw = 0.0;
double initial_yaw = 0.0; // 初始方位角（只记录一次）
double target_yaw = 0.0;  // 目标角度（基于初始角度计算，保持不变）
bool is_initialized = false; // 初始方位角是否已记录
bool is_rotating = false;
bool rotation_completed = false; // 旋转完成标志
const double ANGULAR_VELOCITY = M_PI / 10.0; // 3秒转完180度的角速度 (π/3 rad/s)

// IMU回调函数
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 从四元数提取偏航角（yaw）
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    
    // 计算偏航角（弧度）
    current_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // 只在首次回调时记录初始方位角
    if (!is_initialized)
    {
        initial_yaw = current_yaw;
        // 计算目标角度（180度 = π弧度）
        target_yaw = initial_yaw + M_PI;
        
        // 调整角度范围到[-π, π]
        if (target_yaw > M_PI)
            target_yaw -= 2*M_PI;
        else if (target_yaw < -M_PI)
            target_yaw += 2*M_PI;
        
        is_initialized = true;
    }
}

// 主循环控制
void controlLoop()
{
    if (is_initialized && !is_rotating && !rotation_completed)
    {
        // 开始旋转
        is_rotating = true;
    }
    else if (is_rotating)
    {
        // 计算当前角度与目标角度的差值
        double yaw_diff = target_yaw - current_yaw;
        
        // 调整角度差值到[-π, π]
        if (yaw_diff > M_PI)
            yaw_diff -= 2*M_PI;
        else if (yaw_diff < -M_PI)
            yaw_diff += 2*M_PI;
        
        // 检查是否达到旋转目标，缩小容耐范围提高精度
        if (fabs(yaw_diff) < 0.02) // 约3度的误差
        {
            // 停止机器人
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
            is_rotating = false;
            rotation_completed = true; // 标记旋转完成
        }
        else
        {
            // 使用固定角速度旋转
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = (yaw_diff > 0) ? ANGULAR_VELOCITY : -ANGULAR_VELOCITY;
        }
        
        // 发布速度指令
        vel_pub.publish(vel_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotation");
    ros::NodeHandle nh;

    // 创建速度发布器
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 检查IMU话题是否存在
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    bool imu_topic_found = false;
    std::string imu_topic = "/imu";
    
    // 尝试不同的IMU话题名称
    if (!imu_topic_found) {
        for (const auto& topic : topics) {
            if (topic.name == "/imu") {
                imu_topic_found = true;
                break;
            }
        }
    }
    
    if (!imu_topic_found) {
        imu_topic = "/imu/data";
        for (const auto& topic : topics) {
            if (topic.name == "/imu/data") {
                imu_topic_found = true;
                break;
            }
        }
    }
    
    if (!imu_topic_found) {
        imu_topic = "/imu/data_raw";
        for (const auto& topic : topics) {
            if (topic.name == "/imu/data_raw") {
                imu_topic_found = true;
                break;
            }
        }
    }
    
    // 订阅IMU话题
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);

    // 设置循环频率：10Hz
    ros::Rate rate(10);
    
    // 等待IMU数据初始化
    int init_count = 0;
    while (ros::ok() && !is_initialized && init_count < 30) // 最多等待3秒
    {
        ros::spinOnce();
        rate.sleep();
        init_count++;
    }
    
    if (!is_initialized)
    {
        return 1;
    }
    
    while (ros::ok() && !rotation_completed)
    {
        ros::spinOnce(); // 处理回调函数
        controlLoop();   // 执行控制逻辑
        rate.sleep();    // 按频率休眠
    }

    return 0;
}
