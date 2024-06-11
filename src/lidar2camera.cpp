/*
 * Copyright (C) 2022 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include "camera_calibrator.hpp"

int main(int argc, char **argv)
{
    /* ----- Check input arguments ----- */
    if (argc != 3)
    {
        std::cout << "Usage: ./lidar2camera camera_dir csv_file\n"
                     "example:\n\t"
                     "./bin/lidar2camera data/intrinsic/ data/circle.csv"
                  << std::endl;
        return 0;
    }

    /* ----- Read input arguments ----- */
    std::string images_dir  = argv[1];
    std::string csv_file    = argv[2];

    std::cout << "Given images directory: " << images_dir << std::endl;
    std::cout << "Given CSV file path: "    << csv_file << std::endl;

    /* ----- Load lidar 3D center points from CSV ----- */
    std::ifstream fin(csv_file);
    std::string line;
    bool is_first = true;
    std::vector<std::vector<std::string>> lidar_3d_pts;

    while (getline(fin, line))
    {
        if (is_first)
        {
            is_first = false;
            continue;
        }

        std::istringstream sin(line);
        std::vector<std::string> fields;
        std::string field;

        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }

        lidar_3d_pts.push_back(fields);
    }

    /* ----- Load images from given directory ----- */
    std::vector<cv::String> image_paths;
    cv::glob(images_dir, image_paths);

    std::vector<cv::Mat> images;
    std::vector<std::string> images_name;

    std::cout << "Found following images: " << std::endl;
    for (const auto &path : image_paths)
    {
        std::cout << path << std::endl;
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        images.push_back(img);
        images_name.push_back(path);
    }

    /* ----- Set inputs to Camera Calibrator object ----- */
    cv::Mat camera_matrix    = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat dist_coeffs      = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
    cv::Size chessboard_size (17, 7);
    cv::Size image_size      (1920, 1080);

    CameraCalibrator lidar_camera_calibrator;
    lidar_camera_calibrator.set_input(images_name, images, chessboard_size, lidar_3d_pts);

    /* ----- Perform calibration and get results ----- */
    std::vector<cv::Mat> rot_vectors;
    std::vector<cv::Mat> trans_vectors;

    lidar_camera_calibrator.get_result(camera_matrix,
                                       dist_coeffs,
                                       image_size,
                                       rot_vectors,
                                       trans_vectors);

    /* ----- Print results to terminal ----- */
    std::cout << "----- Calibration Completed! -----" << std::endl;

    std::cout << "Camera Matrix:"             << std::endl << camera_matrix        << std::endl;
    std::cout << "Dist coefficient:"          << std::endl << dist_coeffs          << std::endl;
    std::cout << "Rotations vector size:"     << std::endl << rot_vectors.size()   << std::endl;
    std::cout << "Translations vector size:"  << std::endl << trans_vectors.size() << std::endl;

    std::cout << "Translation vector 0:"      << std::endl << trans_vectors[0]     << std::endl;
    std::cout << "Rotation vector 0:"         << std::endl << rot_vectors[0]       << std::endl;
    
    std::cout << "Rotation vector 1:"         << std::endl << rot_vectors[1]       << std::endl;
    std::cout << "Translation vector 1:"      << std::endl << trans_vectors[1]     << std::endl;

    std::cout << "----------------------------------" << std::endl;

    return 0;
}
