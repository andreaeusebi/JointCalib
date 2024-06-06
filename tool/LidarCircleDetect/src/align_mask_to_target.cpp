#include <QApplication>
#include <QMainWindow>
#include "CloudAligner.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow main_window;

    std::string pcd_file_map    = "/home/andrea/pcd_files/151203000.pcd";
    std::string pcd_file_mask   = "test_results/target_mask.pcd";   

    CloudAligner<pcl::PointXYZRGB> cloud_aligner(pcd_file_map, pcd_file_mask, &main_window);

    main_window.setCentralWidget(&cloud_aligner);
    main_window.resize(800, 600);
    main_window.show();

    return app.exec();
}
