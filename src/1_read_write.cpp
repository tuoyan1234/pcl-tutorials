#include "1_read_write.h"

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

bool writePlyFile(std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud){
    if(inCloud->points.size() == 0){
        std::cout << "Error: the cloud is empty" << std::endl;
        return false;
    }else{
        pcl::PLYWriter writer;
        writer.write(filename, *inCloud);
        std::cout << "Write ply file successfully" << std::endl;
        return true;
    }
}

bool writePcdFile(std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud){
    if(inCloud->points.size() == 0){
        std::cout << "Error: the cloud is empty" << std::endl;
        return false;
    }else{
        pcl::PCDWriter writer;
        writer.write(filename, *inCloud);
        std::cout << "Write pcd file successfully" << std::endl;
        return true;
    }
}

// downsample
bool uniformdownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, float leafSize){
    pcl::UniformSampling<pcl::PointXYZI> filter;
    filter.setInputCloud(inCloud);
    filter.setRadiusSearch(leafSize);
    filter.filter(*outCloud);
    return true;
}
bool voxelgrid_downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, float leafSize){
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(inCloud);
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.filter(*outCloud);
    return true;
}

// 平面提取
bool ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr non_plane_cloud){
    if(cloud_in->points.size() < 3){
        std::cout << "点云数量小于3，无法进行平面拟合" << std::endl;
        return false;
    }
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);
    extract.setNegative(true);
    extract.filter(*non_plane_cloud);
    return true;
}

int main(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 读取点云数据
    std::string plyname = "../resource/1761118386.819890.ply";
    if(readPlyFile(plyname, cloud)){
        show_point_cloud(cloud, "read cloud");
    }

    // 保存pcd格式点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcd_cloud->width = cloud->width;
    pcd_cloud->height = cloud->height;
    // pcd_cloud->is_dense = cloud->is_dense;
    // pcd_cloud->points.resize(pcd_cloud->width * pcd_cloud->height);
    for(size_t i = 0; i < cloud->points.size(); i++){
        pcd_cloud->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
    std::string pcd_filename = "../res/1761118386.819890_write.pcd";
    writePcdFile(pcd_filename, pcd_cloud);

    // 体素降采样
    voxelgrid_downsample(cloud, cloud, 0.1);
    show_point_cloud(cloud, "voxelgrid downsampled cloud");

    // 平面分割
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ransacPlaneSegmentation(cloud, plane_cloud, non_plane_cloud);
    show_point_cloud(plane_cloud, "plane cloud");
    show_point_cloud(non_plane_cloud, "non plane cloud");

    // 写入点云数据
    std::string ply_write_path = "../res/1761118386.819890_write.ply";
    writePlyFile(ply_write_path, cloud);
    show_point_cloud(cloud, "save cloud");

    return 0;
}
