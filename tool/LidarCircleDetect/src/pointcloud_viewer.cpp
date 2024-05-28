#include <QApplication>
#include <QMainWindow>
#include "PointCloudViewer.h"

// #define SINGLE_CLOUD

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow mainWindow;

    #ifdef SINGLE_CLOUD
    PointCloudViewer<pcl::PointXYZ> viewer("target_mask.pcd", &mainWindow);
    #else
    std::string pcd_file_map = "data/2022-01-18-15-25-03-449.pcd";
    std::string pcd_file_target = "target_mask.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

    std::string map_cloud_id = "map";
    std::string target_cloud_id = "cloud";

    // Load map cloud
    if (pcl::io::loadPCDFile(pcd_file_map, *cloud_map) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_file_map.c_str());
        return -1;
    }

    // Load target cloud
    if (pcl::io::loadPCDFile(pcd_file_target, *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_file_target.c_str());
        return -1;
    }

    pcl::visualization::PCLVisualizer::Ptr pcl_viewer (new pcl::visualization::PCLVisualizer ("PointCloudViewer"));

    pcl_viewer->addPointCloud<pcl::PointXYZ>(cloud_map, map_cloud_id);
    pcl_viewer->addPointCloud<pcl::PointXYZ>(cloud_target, target_cloud_id);
    
    std::cout << "pointcloud_viewer: BEFORE INSTANTIATING OBJECT" << std::endl;

    PointCloudViewer<pcl::PointXYZ> viewer(pcl_viewer, target_cloud_id, &mainWindow);

    std::cout << "pointcloud_viewer: AFTER INSTANTIATING OBJECT" << std::endl;
    #endif

    mainWindow.setCentralWidget(&viewer);
    mainWindow.resize(800, 600);
    mainWindow.show();

    std::cout << "##### AFTER mainWindow.show() #####" << std::endl;

    return app.exec();
}
