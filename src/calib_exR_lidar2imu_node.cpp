#include<rclcpp/rclcpp.hpp>
#include<iostream>
#include<calibExRLidar2Imu.h>
#include<pcl_conversions/pcl_conversions.h>
#include<deque>
#include<queue>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>


using namespace std;
using namespace std::chrono_literals;

class CalibNode : public rclcpp::Node
{
    public:
        CalibNode():Node("lidar_imu_calib")
        {
    
            std::string lidar_topic;
            std::string imu_topic;

            this->declare_parameter("lidar_topic","/cloud");
            this->declare_parameter("imu_topic","/imu");

            this->get_parameter("lidar_topic",lidar_topic);
            this->get_parameter("imu_topic",imu_topic);

            lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud",1000,std::bind(&CalibNode::LidarCallback,this,std::placeholders::_1));
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu",1000,std::bind(&CalibNode::ImuCallback,this,std::placeholders::_1));
            __timer__ = this->create_wall_timer(0.01s,std::bind(&CalibNode::timer_callback,this));
        }

    private:
        void LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            lidar_buffer.push(msg);
        }

        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            imu_buffer.push(msg);
        }

        void timer_callback()
        {
            while(lidar_buffer.size() != 0)
            {
                CloudT::Ptr cloud(new CloudT);
                pcl::fromROSMsg(*(lidar_buffer.front()), *cloud);
                LidarData data;
                data.cloud = cloud;
                data.stamp = lidar_buffer.front()->header.stamp.sec;
                caliber.addLidarData(data);
                lidar_cnt++;
                lidar_buffer.pop();
            }
            while(imu_buffer.size()!=0)
            {
                ImuData data;
                data.acc = Eigen::Vector3d(imu_buffer.front()->linear_acceleration.x,
                                        imu_buffer.front()->linear_acceleration.y,
                                        imu_buffer.front()->linear_acceleration.z);
                data.gyr = Eigen::Vector3d(imu_buffer.front()->angular_velocity.x,
                                        imu_buffer.front()->angular_velocity.y,
                                        imu_buffer.front()->angular_velocity.z);
                data.rot = Eigen::Quaterniond(imu_buffer.front()->orientation.w,
                                            imu_buffer.front()->orientation.x,
                                            imu_buffer.front()->orientation.y,
                                            imu_buffer.front()->orientation.z);
                data.stamp = imu_buffer.front()->header.stamp.sec;
                std::cout<<data.stamp<<std::endl;
                caliber.addImuData(data);  
                imu_cnt++;
                imu_buffer.pop();
            }
            RCLCPP_INFO(this->get_logger(),"%d,%d",lidar_cnt,imu_cnt);
            if(lidar_cnt >= 2500)
            {
                Eigen::Vector3d rpy = caliber.calib();
                Eigen::Matrix3d rot = Eigen::Matrix3d(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
                cout << "result euler angle(RPY) : " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;
                cout << "result extrinsic rotation matrix : " << endl;
                cout << rot << endl;
                exit(0);
            }
        }   
        queue<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_buffer;
        queue<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

        rclcpp::TimerBase::SharedPtr __timer__;

        int lidar_cnt{};
        int imu_cnt{};

        CalibExRLidarImu caliber;
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CalibNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}