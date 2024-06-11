/* OpenCV Headers */
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

std::string NODE_NAME       = "camera_calibration_test";
std::string COUT_PREFIX     = "[" + NODE_NAME + "]: ";

void makeObjectPoints(const cv::Size & chessboard_size_,
                      std::vector<cv::Vec3f> & object_points_)
{
    object_points_.clear();

    for (int r = 0; r < chessboard_size_.height; ++r)
    {
        for (int c = 0; c < chessboard_size_.width; ++c)
        {
            // std::cout << COUT_PREFIX << "Inserting point: (" << c << ", " << r << ", 0.0)"
            //           << std::endl;

            object_points_.push_back(cv::Vec3f(c * 0.05, r * 0.05, 0.0));
        }
    }
}

int main(int argc, char ** argv)
{
    std::cout << COUT_PREFIX << "Begin." << std::endl;

    cv::Size chessboard_size(17, 7);
    cv::Size image_size(1920, 1080);

    if (argc != 2)
    {
        std::cout << COUT_PREFIX << "Error! One argument expected!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string images_dir = argv[1];
    std::vector<cv::String> image_files;
    cv::glob(images_dir, image_files);

    std::vector<cv::Mat> images;
    std::vector<std::vector<cv::Point2f>> image_points;

    /* ---- Find chessbord corners in the images ----- */

    std::cout << COUT_PREFIX << "Found following images:" << std::endl;
    for (const auto &path : image_files)
    {
        std::cout << COUT_PREFIX << path << std::endl;
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);

        std::cout << COUT_PREFIX << "Looking for chessboard corners..." << std::endl;
        std::vector<cv::Point2f> corner_points;

        bool corners_found = cv::findChessboardCorners(img, chessboard_size, corner_points);

        if (corners_found)
        {
            std::cout << COUT_PREFIX << "Cheesboard corners found!" << std::endl;
        }
        else
        {
            std::cout << COUT_PREFIX << "Error! Cheesboard corners NOT found!" << std::endl;
            continue;
        }

        cv::cornerSubPix(img,
                         corner_points,
                         cv::Size(5, 5),
                         cv::Size(-1, -1),
                         cv::TermCriteria(2, 30, 0.001));

        cv::drawChessboardCorners(img, chessboard_size, corner_points, corners_found);

        // cv::imshow("Detected Cheesboard Corners", img);
        // cv::waitKey();

        images.push_back(img);
        image_points.push_back(corner_points);
    }

    /* ----- Get 3D corners coordinates in chessboard frame (z = 0) ----- */

    // size is n x m, where n is number of images and m number of corners
    std::vector<std::vector<cv::Vec3f>> object_points;

    for (auto idx = 0; idx < images.size(); idx++)
    {
        std::vector<cv::Vec3f> obj_points_ith;
        makeObjectPoints(chessboard_size, obj_points_ith);

        object_points.push_back(obj_points_ith);
    }

    /* ----- Perform Camera Calibration ----- */
    
    cv::Mat camera_matrix;
    cv::Mat dist_coefficients;
    std::vector<cv::Mat> rot_vectors;
    std::vector<cv::Mat> trans_vectors;

    std::cout << COUT_PREFIX << "Performin camera calibration..." << std::endl;

    double re_error = cv::calibrateCamera(object_points,
                                          image_points,
                                          image_size,
                                          camera_matrix,
                                          dist_coefficients,
                                          rot_vectors,
                                          trans_vectors);

    std::cout << COUT_PREFIX << "Calibration Completed! Overall RMS re-projection error: "
              << re_error << std::endl;

    std::cout << COUT_PREFIX << "Camera matrix:" << std::endl << camera_matrix << std::endl;
    std::cout << COUT_PREFIX << "Dist coeffs:" << std::endl << dist_coefficients << std::endl;

    std::cout << COUT_PREFIX << "Transformation vectors:" << std::endl;

    for (auto idx = 0; idx < rot_vectors.size(); idx++)
    {
        std::cout << COUT_PREFIX << "### " << idx << ":" << std::endl;

        std::cout << COUT_PREFIX << "Translation:" << std::endl << trans_vectors[idx] << std::endl;        
        std::cout << COUT_PREFIX << "Rotation:" << std::endl << rot_vectors[idx] << std::endl;        
    }

    return 0;
}
