#include <QApplication>
#include <QMainWindow>
#include "PointCloudViewer.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow mainWindow;
    PointCloudViewer<pcl::PointXYZ> viewer("target_mask.pcd", &mainWindow);
    mainWindow.setCentralWidget(&viewer);
    mainWindow.resize(800, 600);
    mainWindow.show();

    std::cout << "##### AFTER mainWindow.show() #####" << std::endl;

    return app.exec();
}
