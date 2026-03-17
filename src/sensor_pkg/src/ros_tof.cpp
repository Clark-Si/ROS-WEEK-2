#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>

#define _USE_MATH_DEFINES

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
double current_yaw = 0.0;
double start_backward_x = 0.0;
double start_backward_y = 0.0;
double start_yaw = 0.0;
double target_yaw = 0.0;
double left_distance = 10.0; // 初始化为较大值，避免初始误检测
double front_distance = 10.0; // 初始化为较大值，避免初始误检测
double right_distance = 10.0; // 初始化为较大值，避免初始误检测
const double OBSTACLE_THRESHOLD = 0.3; // 障碍物阈值，单位：米
const double BACKWARD_DISTANCE = 0.2; // 后退距离，单位：米
int obstacle_direction = 0; // 0: 无障碍物, 1: 左侧, 2: 前方, 3: 右侧

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

// 左侧TOF传感器回调函数
void rangeCallback1(const sensor_msgs::Range::ConstPtr& msg)
{
    left_distance = msg->range;
    ROS_INFO("DistanceLeft: %f", left_distance);
}

// 前方TOF传感器回调函数
void rangeCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
    front_distance = msg->range;
    ROS_INFO("DistanceFront: %f", front_distance);
}

// 右侧TOF传感器回调函数
void rangeCallback3(const sensor_msgs::Range::ConstPtr& msg)
{
    right_distance = msg->range;
    ROS_INFO("DistanceRight: %f", right_distance);
}

// 主循环控制
void controlLoop()
{
    switch (current_state)
    {
    case STATE_FORWARD: {
        // 检查是否检测到障碍物
        if (front_distance < OBSTACLE_THRESHOLD || left_distance < OBSTACLE_THRESHOLD || right_distance < OBSTACLE_THRESHOLD)
        {
            ROS_INFO("Obstacle detected! Starting backward movement.");
            ROS_INFO("Obstacle details: left=%.3f, front=%.3f, right=%.3f", 
                     left_distance, front_distance, right_distance);
            
            // 记录障碍物方向，避免后退过程中传感器数据变化影响转向判断
            if (right_distance < OBSTACLE_THRESHOLD)
            {
                obstacle_direction = 3; // 右侧
                ROS_INFO("Obstacle direction: RIGHT");
            }
            else if (front_distance < OBSTACLE_THRESHOLD)
            {
                obstacle_direction = 2; // 前方
                ROS_INFO("Obstacle direction: FRONT");
            }
            else if (left_distance < OBSTACLE_THRESHOLD)
            {
                obstacle_direction = 1; // 左侧
                ROS_INFO("Obstacle direction: LEFT");
            }
            
            // 记录后退开始位置
            start_backward_x = current_x;
            start_backward_y = current_y;
            current_state = STATE_BACKWARD;
        }
        else
        {
            // 无障碍物时以0.1m/s速度前进
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
        if (backward_distance >= BACKWARD_DISTANCE)
        {
            ROS_INFO("Backward movement completed! Starting turn.");
            start_yaw = current_yaw;
            
            // 根据记录的障碍物方向确定转向方向
            if (obstacle_direction == 3) // 右侧有障碍物，向左转60度
            {
                target_yaw = start_yaw + M_PI/3; // 左转60度 (π/3弧度)
                ROS_INFO("Right obstacle detected, turning left 60 degrees");
            }
            else if (obstacle_direction == 1 || obstacle_direction == 2) // 左侧或前方有障碍物，向右转60度
            {
                target_yaw = start_yaw - M_PI/3; // 右转60度 (π/3弧度)
                ROS_INFO("Left or front obstacle detected, turning right 60 degrees");
            }
            else
            {
                // 默认向右转60度
                target_yaw = start_yaw - M_PI/3;
                ROS_WARN("Obstacle direction not set, using default right turn");
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
            ros::Duration(0.3).sleep(); // 停顿0.3秒
            
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
        
        // 调整角度差值到[-π, π]
        if (yaw_diff > M_PI)
            yaw_diff -= 2*M_PI;
        else if (yaw_diff < -M_PI)
            yaw_diff += 2*M_PI;
        
        ROS_INFO("Turning: current_yaw=%.3f, target_yaw=%.3f, yaw_diff=%.3f", current_yaw, target_yaw, yaw_diff);
        
        // 检查是否达到转向目标
        if (fabs(yaw_diff) < 0.15) // 误差小于约8.5度
        {
            ROS_INFO("Turn completed! Resuming forward movement.");
            current_state = STATE_FORWARD;
            // 停止机器人
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
            ros::Duration(0.3).sleep(); // 停顿0.3秒
        }
        else
        {
            // 基于角度差值调整转向速度
            double turn_speed = std::min(1.0, fabs(yaw_diff) * 3.0);
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
    ros::init(argc, argv, "tof_obstacle_avoidance");
    ros::NodeHandle nh;

    // 创建速度发布器
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 订阅TOF传感器和里程计话题
    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::Range>("/ul/sensor1", 10, rangeCallback1); // 左侧TOF
    ros::Subscriber sub_2 = nh.subscribe<sensor_msgs::Range>("/ul/sensor2", 10, rangeCallback2); // 前方TOF
    ros::Subscriber sub_3 = nh.subscribe<sensor_msgs::Range>("/ul/sensor3", 10, rangeCallback3); // 右侧TOF
    ros::Subscriber odom_sub = nh.subscribe("/odom", 100, odomCallback);

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
