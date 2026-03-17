#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;

// 状态定义
enum State {
    STATE_FORWARD,     // 前进
    STATE_BACKWARD,    // 后退
    STATE_TURN,        // 转向
    STATE_STOP         // 停止
};

// 全局变量
State current_state = STATE_FORWARD;
double current_x = 0.0;
double current_y = 0.0;
double start_x = 0.0;
double target_x = 0.0;
double current_yaw = 0.0;
double start_yaw = 0.0;
double target_yaw = 0.0;
bool collision_detected = false;
int collision_type = 0; // 0: 无碰撞, 1: 正前/左前, 2: 右前

// 里程计回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    
    // 从四元数提取偏航角（yaw）
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    
    // 计算偏航角（弧度）
    current_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// 碰撞传感器回调函数
void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3)
    {
        ROS_WARN("Sensor data error! Skip obstacle avoidance.");
        return;
    }

    bool left_bump = msg->data[0];
    bool front_bump = msg->data[1];
    bool right_bump = msg->data[2];

    if (front_bump || left_bump)
    {
        collision_detected = true;
        collision_type = 1; // 正前/左前碰撞
    }
    else if (right_bump)
    {
        collision_detected = true;
        collision_type = 2; // 右前碰撞
    }
    else
    {
        collision_detected = false;
        collision_type = 0; // 无碰撞
    }
}

// 全局变量：后退开始位置
double start_backward_x = 0.0;
double start_backward_y = 0.0;

// 主循环控制
void controlLoop()
{
    switch (current_state)
    {
    case STATE_FORWARD: {
        if (collision_detected)
        {
            ROS_INFO("Collision detected! Starting backward movement.");
            // 记录后退开始位置
            start_backward_x = current_x;
            start_backward_y = current_y;
            current_state = STATE_BACKWARD;
            // 重置碰撞检测标志，避免后退过程中受影响
            collision_detected = false;
        }
        else
        {
            // 无碰撞时以0.1m/s速度前进
            vel_msg.linear.x = 0.1;
            vel_msg.angular.z = 0.0;
        }
        break;
    }
        
    case STATE_BACKWARD: {
        // 计算实际后退距离
        double backward_distance = sqrt(pow(current_x - start_backward_x, 2) + pow(current_y - start_backward_y, 2));
        
        ROS_INFO("Backward: current_x=%.3f, current_y=%.3f, start_x=%.3f, start_y=%.3f, distance=%.3f", 
                 current_x, current_y, start_backward_x, start_backward_y, backward_distance);
        
        // 检查是否达到后退目标（20cm）
        if (backward_distance >= 0.2) // 严格按照要求后退20cm
        {
            ROS_INFO("Backward movement completed! Starting turn.");
            start_yaw = current_yaw;
            
            ROS_INFO("Calculating turn target: start_yaw=%.3f, collision_type=%d", start_yaw, collision_type);
            if (collision_type == 1) // 正前/左前碰撞，右转60度
                target_yaw = start_yaw - M_PI/3; // 右转60度 (π/3弧度)
            else if (collision_type == 2) // 右前碰撞，左转60度
                target_yaw = start_yaw + M_PI/3; // 左转60度 (π/3弧度)
            else
            {
                // 默认右转60度，防止碰撞类型未设置
                target_yaw = start_yaw - M_PI/3;
                ROS_WARN("Collision type not set, using default right turn");
            }
            
            // 调整角度范围到[-π, π]
            if (target_yaw > M_PI)
                target_yaw -= 2*M_PI;
            else if (target_yaw < -M_PI)
                target_yaw += 2*M_PI;
            
            ROS_INFO("Turn target calculated: target_yaw=%.3f", target_yaw);
            
            // 停止机器人并停顿0.3秒
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
            ros::Duration(0.3).sleep(); // 严格按照要求停顿0.3秒
            
            current_state = STATE_TURN;
        }
        else
        {
            // 后退速度0.1m/s
            vel_msg.linear.x = -0.1;
            vel_msg.angular.z = 0.0;
        }
        break;
    }
        
    case STATE_TURN: {
        // 计算当前角度与目标角度的差值
        double yaw_diff = target_yaw - current_yaw;
        
        // 调整角度差值到[-π, π]，确保取最短路径
        if (yaw_diff > M_PI)
            yaw_diff -= 2*M_PI;
        else if (yaw_diff < -M_PI)
            yaw_diff += 2*M_PI;
        
        ROS_INFO("Turning: current_yaw=%.3f, target_yaw=%.3f, yaw_diff=%.3f", current_yaw, target_yaw, yaw_diff);
        
        // 检查是否达到转向目标
        if (fabs(yaw_diff) < 0.15) // 进一步增大误差阈值到约8.5度，提高鲁棒性
        {
            ROS_INFO("Turn completed! Resuming forward movement.");
            current_state = STATE_FORWARD;
            // 停止机器人
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
            ros::Duration(0.3).sleep(); // 停顿0.3秒
            // 确保碰撞检测标志为false，以便重新开始前进
            collision_detected = false;
        }
        else
        {
            // 基于角度差值调整转向速度，提高精度
            double turn_speed = std::min(1.0, fabs(yaw_diff) * 3.0); // 进一步增加转向速度，避免卡住
            // 确保转向方向正确
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = (yaw_diff > 0) ? turn_speed : -turn_speed;
            ROS_INFO("Turn speed: %.3f", vel_msg.angular.z);
        }
        break;
    }
        
    case STATE_STOP: {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        break;
    }
    }
    
    // 发布速度指令
    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_obstacle_avoidance");
    ros::NodeHandle n;

    // 创建速度发布器
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 订阅碰撞传感器和里程计话题
    ros::Subscriber bump_sub = n.subscribe("/robot/bump_sensor", 1000, bumpCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);

    // 设置循环频率：10Hz
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        ros::spinOnce(); // 处理回调函数
        controlLoop();   // 执行控制逻辑
        rate.sleep();    // 按频率休眠
    }

    return 0;
}
