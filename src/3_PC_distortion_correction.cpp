/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-05-17 10:34:52
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-05-17 11:43:00
 * @FilePath: /Desktop/libs-learn/pcl/pcl_tutorials/src/3_PC_distortion_correction.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "1_read_write.h"
#include <Eigen/Dense>

#include <iostream>
bool show_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string window_name){
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
    std::string plyname_1 = "../resource/1761118386.819890.ply";
    std::string plyname_2 = "../resource/1761118386.819890_undistortion.ply";
    if(readPlyFile(plyname_1, input_PC_1)){
        show_point_cloud(input_PC_1, "original cloud_1");
    }
    if(readPlyFile(plyname_2, input_PC_2)){
        show_point_cloud(input_PC_2, "original cloud_2");
    }
    // 读取每帧雷达姿态
    // 已知：Tcam2lidar、Timu2cam,求Timu2lidar==========》》》 求Timu2lidar = Tcam2lidar * Timu2cam
    // 1.读取到雷达时刻较近的rtk表示的机器姿态--转换矩阵
    Eigen::Matrix4f rtk_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rtk_2 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q1 = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());
    Eigen::Vector3f t1 = Eigen::Vector3f(1, 1, 0.0);
    Eigen::Quaternionf q2 = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * 
							Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());
    Eigen::Vector3f t2 = Eigen::Vector3f(1, 1, 0.0);
    rtk_1.block<3,3>(0,0) = q1.toRotationMatrix();
    rtk_1.block<3,1>(0,3) = t1;
    rtk_2.block<3,3>(0,0) = q2.toRotationMatrix();
    rtk_2.block<3,1>(0,3) = t2;
    // 2.rtk转换矩阵转换到雷达坐标系下
    Eigen::Matrix4f T_lidar2world_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_lidar2world_2 = Eigen::Matrix4f::Identity();
    // 3.rtk到相机的转换矩阵
    Eigen::Matrix4f T_imu2cam = Eigen::Matrix4f::Identity();
    // 4.相机到雷达的转换矩阵
    Eigen::Matrix4f T_cam2lidar = Eigen::Matrix4f::Identity();
    // 5.雷达相对世界坐标系的转换矩阵
    T_lidar2world_1 = T_cam2lidar * T_imu2cam * rtk_1;
    T_lidar2world_2 = T_cam2lidar * T_imu2cam * rtk_2; 
    // 基于转换矩阵，计算姿态变化
    Eigen::Matrix4f T_lidar2lidar1 = T_lidar2world_1.inverse() * T_lidar2world_2;
    // 基于姿态变化，计算每个点的矫正位置
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_PC_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*input_PC_2, *transformed_PC_2, T_lidar2lidar1);
    *input_PC_1 += *transformed_PC_2;
    show_point_cloud(input_PC_1, "distortion-correction-cloud");

    return 0;
}