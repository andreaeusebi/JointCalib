#pragma once

#ifndef CALIBRATION_TARGET_MASK
#define CALIBRATION_TARGET_MASK

/** System Includes */
#include <thread>
#include <chrono>

/* PCL Includes */
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/io/pcd_io.h"

using namespace std::chrono_literals;

class Point2D
{
    public:
        Point2D(double x_, double y_) :
            x {x_},
            y {y_}
        {}

        double x;
        double y;
};

template <typename PointT>
class CalibrationTargetMask
{
    public:

        CalibrationTargetMask();

        ~CalibrationTargetMask();

        void visualizeAndSaveTargetMaskCloud();

        /** Get a read-only reference to the target mask pointer */
        const typename pcl::PointCloud<PointT>::Ptr & getTargetMask() const
        {
            return m_target_mask;
        }

    private:
        /* ### ----- Class Methods ----- ### */

        bool isPointInsideCircles(const Point2D & point_) const;

        void fillPointCloudPoint(const Point2D & point_, PointT & cloud_point_);

        void visualizationThread();

        /* ### ----- Class Attributes ----- ### */

        typename pcl::PointCloud<PointT>::Ptr m_target_mask;

        /** Resolution of the mask in meters. */
        const double m_resolution {0.005};

        /** Size of the calibration target along x direction in meters. */
        const double m_x_size {1.200};
        /** Size of the calibration target along y direction in meters. */
        const double m_y_size {1.200};
        
        const std::vector<Point2D> m_circle_centers {
            Point2D(0.3, 0.3),  // top left circle
            Point2D(0.3, 0.9),  // top right circle
            Point2D(0.9, 0.3),  // bottom left circle
            Point2D(0.9, 0.9)   // bottom right circle
        };

        const double m_circle_radius {0.1};

        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        std::thread m_vis_thread;

        std::atomic<bool> m_stop { false };
};

template <typename PointT>
CalibrationTargetMask<PointT>::CalibrationTargetMask() :
    m_target_mask   {new pcl::PointCloud<PointT>},
    m_viewer        {new pcl::visualization::PCLVisualizer("CalibrationTargetMask Viewer")}
{
    // Compute number of points along x and y directions
    const int points_x = m_x_size / m_resolution;
    const int points_y = m_y_size / m_resolution;
  
    // Fill the base rectangle plane
    for (int i = 0; i <= points_x; i++)
    {
        for (int j = 0; j <= points_y; j++)
        {
            const Point2D point (i * m_resolution, j * m_resolution);

            // Check wheter the point belongs to one of the circular holes
            if (this->isPointInsideCircles(point) )
            {
                continue; 
            }

            PointT cloud_point;

            this->fillPointCloudPoint(point, cloud_point);

            m_target_mask->push_back(cloud_point);
        }
        
    }
}

template <typename PointT>
CalibrationTargetMask<PointT>::~CalibrationTargetMask()
{
    // Should I release pointers?
}

template <typename PointT>
void CalibrationTargetMask<PointT>::visualizeAndSaveTargetMaskCloud()
{
    // Set up the visualization
    m_viewer->addPointCloud<PointT>(m_target_mask, "Calibration Target Mask");
    
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               "Calibration Target Mask");
    m_viewer->addCoordinateSystem(1.0);
    m_viewer->initCameraParameters();

    // Start visualization thread
    m_vis_thread = std::thread(&CalibrationTargetMask::visualizationThread, this);

    std::cout << "Displayed is the Calibration Target Mask. Enter 's' if you are satisfied "
              << "and want to save it, 'q' if you want to quit the program without saving."
              << std::endl;

    // Main loop for user input
    while (!m_stop)
    {
        // Get user input       
        char choice;
        std::cin >> choice;

        switch (choice)
        {
            case 's':
            {
                std::cout << "Saving Calibration Target Mask pointcloud as pcd file..."
                          << std::endl;

                std::string pcd_file_name = "target_mask.pcd";

                pcl::io::savePCDFileBinary(pcd_file_name, *m_target_mask);

                std::cout << "Saved in file: " << pcd_file_name << std::endl
                          << "Exiting..." << std::endl;
                
                m_stop = true;

                break;
            }
            case 'q':
                std::cout << "Exiting without saving to file!" << std::endl;

                m_stop = true;

                break;
            default:
                std::cout << "Invalid choice. Please enter 's' or 'q'." << std::endl;
        }
    }

    // Join visualization thread
    m_vis_thread.join();
}

template <typename PointT>
bool CalibrationTargetMask<PointT>::isPointInsideCircles(const Point2D & point_) const
{
    for (auto circle_it = m_circle_centers.cbegin(); circle_it != m_circle_centers.cend(); circle_it++)
    {
        double distance {sqrt( pow(point_.x - circle_it->x, 2) + pow(point_.y - circle_it->y, 2) ) };

        if (distance < m_circle_radius)
        {
            return true;
        }
    }
    
    return false;
}

// Default implementation for the primary template
template <typename PointT>
void CalibrationTargetMask<PointT>::fillPointCloudPoint(const Point2D & point_,
                                                        PointT & cloud_point_)
{
    cloud_point_.x = point_.x;
    cloud_point_.y = point_.y;
    cloud_point_.z = 0.0;
}

// Template specialization for pcl::PointXYZRGB
template<>
void CalibrationTargetMask<pcl::PointXYZRGB>::fillPointCloudPoint(const Point2D & point_,
                                                                  pcl::PointXYZRGB & cloud_point_)
{
    cloud_point_.x = point_.x;
    cloud_point_.y = point_.y;
    cloud_point_.z = 0.0;
    cloud_point_.r = 255 * (point_.x / m_x_size);
    cloud_point_.g = 255 * (point_.y / m_y_size);
    cloud_point_.b = 0;
}

template <typename PointT>
void CalibrationTargetMask<PointT>::visualizationThread()
{
    while (!m_stop)
    {
        m_viewer->spinOnce(100); // Update viewer
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
    }
}

#endif
