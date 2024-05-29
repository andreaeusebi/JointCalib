#include <QApplication>
#include <QMainWindow>
#include "PointCloudViewer.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow mainWindow;

    std::string pcd_file_map = "data/2022-01-18-15-25-03-449.pcd";
    std::string pcd_file_target = "test_results/target_mask.pcd";   

    PointCloudViewer<pcl::PointXYZRGB> viewer(pcd_file_map, pcd_file_target, &mainWindow);

    mainWindow.setCentralWidget(&viewer);
    mainWindow.resize(800, 600);
    mainWindow.show();

    return app.exec();
}
