/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-05-13 23:08:49
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-05-17 11:36:59
 * @FilePath: /Desktop/libs-learn/pcl/pcl_tutorials/include/1_read_write.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
// downsample
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/search/search.h>
// 平面提取
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
// 点云坐标变换
#include <pcl/common/transforms.h>

#include <iostream>

bool readPlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud);
bool writePlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud);
