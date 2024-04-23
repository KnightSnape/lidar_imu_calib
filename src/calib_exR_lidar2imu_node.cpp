#include<rclcpp/rclcpp.hpp>
#include<iostream>
#include<calibExRLidar2Imu.h>
#include<pcl/conversions.h>
#include<deque>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

deque<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

void LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    lidar_buffer.push_back(msg);
}

void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_buffer.push_back(msg);
}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("lidar_imu_calib");
    
    std::string lidar_topic;
    std::string imu_topic;

    n->declare_parameter("lidar_topic","/scan");
    n->declare_parameter("imu_topic","/imu");

    n->get_parameter("lidar_topic",lidar_topic);
    n->get_parameter("imu_topic",imu_topic);

    CalibExRLidarImu caliber;

    lidar_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud",1000,LidarCallback);





    return 0;
}