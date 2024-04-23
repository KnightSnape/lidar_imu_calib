#pragma once

#include<iostream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/filters/voxel_grid.h>
#include<Eigen/Eigen>

#include"../../ndt_omp_ros2/include/pclomp/ndt_omp.h"

using namespace std;

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct LidarData
{
    double stamp;
    CloudT::Ptr cloud;
};

struct LidarFrame
{
    double stamp;
    Eigen::Matrix4d T;
    Eigen::Matrix4d gT;
    CloudT::Ptr cloud{nullptr};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    Eigen::Quaterniond rot;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CalibExRLidarImu
{
    public:
        CalibExRLidarImu();
        ~CalibExRLidarImu();

        void setInitExR(Eigen::Vector3d init_R);

        void addLidarData(const LidarData &data);

        void addImuData(const ImuData &data);

        Eigen::Vector3d calib(bool integration = false);


    private:
        Eigen::Quaterniond getInterpolatedAttitude(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale);

        void optimize();

        Eigen::Quaterniond solve(const vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> &corres);

        Eigen::Vector3d init_R_{0.0, 0.0, 0.0};
        CloudT::Ptr last_lidar_cloud_{nullptr};                                 
        vector<LidarFrame> lidar_buffer_;                                       
        vector<ImuData> imu_buffer_;                                            
        vector<pair<LidarFrame, Eigen::Quaterniond>> aligned_lidar_imu_buffer_; 
        Eigen::Quaterniond q_l_b_;   

        CloudT::Ptr local_map_{nullptr};                                              // local map
        pcl::VoxelGrid<PointT> downer_;                                               // downsample local map
        pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr register_{nullptr}; // register object

        vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres1_;
        vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres2_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW         
};