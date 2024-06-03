#include <QApplication>
#include <QMainWindow>
#include "CloudAligner.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow mainWindow;

    std::string pcd_file_map    = "data/2022-01-18-15-25-03-449.pcd";
    std::string pcd_file_mask   = "test_results/target_mask.pcd";   

    CloudAligner<pcl::PointXYZRGB> viewer(pcd_file_map, pcd_file_mask, &mainWindow);

    mainWindow.setCentralWidget(&viewer);
    mainWindow.resize(800, 600);
    mainWindow.show();

    return app.exec();
}
