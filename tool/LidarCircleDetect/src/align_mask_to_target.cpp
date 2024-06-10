#include <QApplication>
#include <QMainWindow>
#include "CloudAligner.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow main_window;

    std::string pcd_file_map    = "/home/andrea/data/calibration_sensor_data/original/_laser_left_data_002.pcd";
    std::string pcd_file_mask   = "test_results/target_mask.pcd";
    std::string results_file    = "/home/andrea/data/calibration_sensor_data/lidar_circles_centers.csv";

    CloudAligner<pcl::PointXYZRGB> cloud_aligner(pcd_file_map,
                                                 pcd_file_mask,
                                                 results_file,
                                                 &main_window);

    main_window.setCentralWidget(&cloud_aligner);
    main_window.resize(800, 600);
    main_window.show();

    return app.exec();
}
