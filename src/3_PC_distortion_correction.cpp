/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-05-17 10:34:52
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-05-18 21:50:54
 * @FilePath: /Desktop/libs-learn/pcl/pcl_tutorials/src/3_PC_distortion_correction.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "1_read_write.h"
#include <Eigen/Dense>

#include <iostream>

template<typename PointT>
bool show_point_cloud(typename pcl::PointCloud<PointT>::Ptr cloud, std::string window_name){
    if (!cloud) {
        std::cerr << "Error: cloud is null" << std::endl;
        return false;
    }
    pcl::visualization::CloudViewer viewer(window_name);
    viewer.showCloud(cloud, window_name);
    while(!viewer.wasStopped()){
    }
    return true;
}

bool readPlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud){
    if(pcl::io::loadPLYFile<pcl::PointXYZI>(filename, *outCloud) == -1){
        std::cout << "Error: can not read the ply file" << std::endl;
        return false;
    }else{
        return true;
    }
}
int main(){
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_PC_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_PC_2(new pcl::PointCloud<pcl::PointXYZI>);
    std::string plyname_1 = "../resource/pose_data/1766476395.219892.ply";
    std::string plyname_2 = "../resource/pose_data/1766476395.319893.ply";
    if(readPlyFile(plyname_1, input_PC_1)){
        show_point_cloud<pcl::PointXYZI>(input_PC_1, "original cloud_1");
    }
    if(readPlyFile(plyname_2, input_PC_2)){
        show_point_cloud<pcl::PointXYZI>(input_PC_2, "original cloud_2");
    }
    /*
    1.颜色区分原始点云
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_PC_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto point : input_PC_1->points){
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 255;
        colored_point.g = 0;
        colored_point.b = 0;
        colored_PC_1->points.push_back(colored_point);
    }
    for(auto point : input_PC_2->points){
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 255;
        colored_point.g = 255;
        colored_point.b = 255;
        colored_PC_1->points.push_back(colored_point);
    }
    show_point_cloud<pcl::PointXYZRGB>(colored_PC_1, "Ori-colored-PC");
    // 读取每帧雷达姿态
    // 已知：Tcam2lidar、Timu2cam,求Timu2lidar==========》》》 求Timu2lidar = Tcam2lidar * Timu2cam
    // 1.读取到雷达时刻较近的rtk表示的机器姿态--转换矩阵
    Eigen::Matrix4f rtk_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rtk_2 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q1 = Eigen::AngleAxisf(2.386932, Eigen::Vector3f::UnitZ()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());
    Eigen::Vector3f t1 = Eigen::Vector3f(0.562757, 5.397068, -0.094988);
    Eigen::Quaternionf q2 = Eigen::AngleAxisf(2.434984, Eigen::Vector3f::UnitZ()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());
    Eigen::Vector3f t2 = Eigen::Vector3f(0.484102, 5.435631, -0.102211);
    rtk_1.block<3,3>(0,0) = q1.toRotationMatrix();
    rtk_1.block<3,1>(0,3) = t1;
    rtk_2.block<3,3>(0,0) = q2.toRotationMatrix();
    rtk_2.block<3,1>(0,3) = t2;
    // 2.rtk转换矩阵转换到雷达坐标系下
    Eigen::Matrix4f T_lidar2world_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_lidar2world_2 = Eigen::Matrix4f::Identity();
    // 3.rtk到相机的转换矩阵
    Eigen::Matrix4f T_imu2cam = Eigen::Matrix4f::Identity();
    T_imu2cam << 9.99989e-01, 2.3758e-03, 4.46592e-03, 0,
                -4.0748e-03, -1.44794e-01, 9.89453e-01, 1.55e-01,
                2.99742e-03, 9.8946e-01, -1.44783e-01, 4.6e-03,
                0, 0, 0, 1;
    // 4.相机到雷达的转换矩阵
    Eigen::Matrix4f T_cam2lidar = Eigen::Matrix4f::Identity();

    T_cam2lidar << 9.9999822968029939e-01,1.8313739926385064e-03,
    -4.3209439513847462e-04, 4.6691198985442011e-03,

    7.2998758449245506e-04, -1.6593114930176212e-01,
    9.8613706999053796e-01, 6.5879603400000000e-02,

    1.7342878635652325e-03, -9.8613563963619921e-01,
    -1.6593219243085677e-01, 8.2844918300000001e-02,
    0, 0, 0, 1;
    // 5.雷达相对世界坐标系的转换矩阵
    T_lidar2world_1 = T_cam2lidar * T_imu2cam * rtk_1;
    T_lidar2world_2 = T_cam2lidar * T_imu2cam * rtk_2; 
    // 基于转换矩阵，计算姿态变化
    Eigen::Matrix4f T_lidar2lidar1 = T_lidar2world_1.inverse() * T_lidar2world_2;
    // 基于姿态变化，计算每个点的矫正位置
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_PC_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*input_PC_2, *transformed_PC_2, T_lidar2lidar1);
    // *input_PC_1 += *transformed_PC_2;
    /*
    1.颜色区分原始点云
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr undistortion_colored_PC(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto point : input_PC_1->points){
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 255;
        colored_point.g = 0;
        colored_point.b = 0;
        undistortion_colored_PC->points.push_back(colored_point);
    }
    for(auto point : transformed_PC_2->points){
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 255;
        colored_point.g = 255;
        colored_point.b = 255;
        undistortion_colored_PC->points.push_back(colored_point);
    }
    show_point_cloud<pcl::PointXYZRGB>(undistortion_colored_PC, "distortion-correction-cloud");

    return 0;
}