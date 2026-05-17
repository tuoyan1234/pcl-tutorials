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

#include <iostream>

bool readPlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud);
bool writePlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud);
