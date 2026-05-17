/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-05-14 20:41:48
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-05-14 20:51:56
 * @FilePath: /pcl_tutorials/src/2_point_normals.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

bool show_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string window_name){
    if (!cloud) {
        std::cerr << "Error: cloud is null" << std::endl;
        return false;
    }
    pcl::visualization::CloudViewer viewer(window_name);
    // 设置背景颜色（RGB:0~1）
    viewer.runOnVisualizationThreadOnce(
        [](pcl::visualization::PCLVisualizer& vis){
            vis.setBackgroundColor(0.0, 0.0, 0.0); // 黑色背景
        });
    viewer.showCloud(cloud, window_name);
    while(!viewer.wasStopped()){
    }
    return true;
}
bool readPcdFile(std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud){
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *outCloud) == -1){
        std::cout << "Error: can not read the pcd file" << std::endl;
        return false;
    }else{
        return true;
    }
}
int main ()
{
    //加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string point_filename = "../resource/table_scene_lms400.pcd";
    readPcdFile(point_filename, cloud);
    show_point_cloud(cloud, "original cloud");
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
    //估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    //创建一个空的kdtree对象，并把它传递给法线估计对象
    //基于给出的输入数据集，kdtree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    //输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //使用半径在查询点周围3厘米范围内的所有邻元素
    ne.setRadiusSearch (0.05);
    // ne.setRadiusSearch (0.4);
    //计算特征值
    ne.compute (*cloud_normals);
    // cloud_normals->points.size () 应该与input_cloud_downsampled->points.size () 有相同尺寸
    // 基于法线方向提取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud_normals->points.size(); i++){
        if(cloud_normals->points[i].curvature > 0.01){
            cloud_filtered->points.push_back(cloud->points[i]);
        }
    }
    show_point_cloud(cloud_filtered, "cloud_filtered");
    std::cout << "cloud_filtered->points.size() = " << cloud_filtered->points.size() << std::endl;
    /*
    for(int i = 0; i < cloud_normals->points.size(); i++){
        if(cloud_normals->points[i].normal_x > 0.05){
            cloud_filtered_x->points.push_back(cloud->points[i]);
        }
        if(cloud_normals->points[i].normal_y > 0.05){
            cloud_filtered_y->points.push_back(cloud->points[i]);
        }
        if(cloud_normals->points[i].normal_z > 0.05){
            cloud_filtered_z->points.push_back(cloud->points[i]);
        }
    }
    show_point_cloud(cloud_filtered_x, "cloud_filtered_x");
    show_point_cloud(cloud_filtered_y, "cloud_filtered_y");
    show_point_cloud(cloud_filtered_z, "cloud_filtered_z");
    */
    //法线可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}
