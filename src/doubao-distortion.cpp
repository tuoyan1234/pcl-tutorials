/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-05-19 21:27:37
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-05-19 21:37:46
 * @FilePath: /pcl-tutorials/src/doubao-distortion.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

// 位姿结构体：时间戳 + 旋转四元数 + 平移
struct PoseData
{
    double timestamp;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
};
// 单点运动畸变矫正
// p_in:原始点 xyz
// s:时间比例 0~1
// pose_start:帧起始位姿
// pose_end:帧结束位姿
Eigen::Vector3d undistortSinglePoint(
    const Eigen::Vector3d& p_in,
    double s,
    const PoseData& pose_start,
    const PoseData& pose_end)
{
    // 1.四元数球面线性插值 旋转
    Eigen::Quaterniond q_interp = pose_start.q.slerp(s, pose_end.q);
    // 2.平移线性插值
    Eigen::Vector3d t_interp = pose_start.t + s * (pose_end.t - pose_start.t);
    // 3.坐标矫正
    return q_interp * p_in + t_interp;// 这个坐标变换不严谨---
}
/**
 * @brief 整帧点云去除运动畸变
 * @param cloud_raw 输入原始带畸变点云
 * @param cloud_correct 输出矫正后点云
 * @param pose_start 帧起始时刻位姿
 * @param pose_end 帧结束时刻位姿
 * @param point_time_offset 每个点相对帧起始的时间偏移(秒)
 * @param frame_total_time 整帧点云总时长(秒)
 */
void undistortLidarCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_raw,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_correct,
    const PoseData& pose_start,
    const PoseData& pose_end,
    const std::vector<double>& point_time_offset,
    double frame_total_time)
{
    cloud_correct->clear();
    cloud_correct->resize(cloud_raw->size());

    for (size_t i = 0; i < cloud_raw->size(); ++i)
    {
        // 计算时间权重 0~1
        double s = point_time_offset[i] / frame_total_time;
        if(s < 0) s = 0.0;
        if(s > 1) s = 1.0;

        Eigen::Vector3d raw_p(cloud_raw->points[i].x,
                              cloud_raw->points[i].y,
                              cloud_raw->points[i].z);
        
        // 逐点矫正
        Eigen::Vector3d correct_p = undistortSinglePoint(raw_p, s, pose_start, pose_end);

        cloud_correct->points[i].x = correct_p.x();
        cloud_correct->points[i].y = correct_p.y();
        cloud_correct->points[i].z = correct_p.z();
    }
}
int main()
{
    // 1.构造首尾位姿（实际从IMU/里程计获取）
    PoseData pose0, pose1;
    // 起始位姿
    pose0.timestamp = 1600000000.0;
    pose0.q = Eigen::Quaterniond::Identity();
    pose0.t << 0,0,0;
    // 结束位姿
    pose1.timestamp = 1600000000.1;
    pose1.q = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX());
    pose1.t << 0.2, 0.1, 0.05;

    // 2.模拟点云 + 每个点时间偏移
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr correct_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> time_off;
    double total_dt = pose1.timestamp - pose0.timestamp;

    // 3.执行去畸变
    undistortLidarCloud(raw_cloud, correct_cloud, pose0, pose1, time_off, total_dt);

    return 0;
}

