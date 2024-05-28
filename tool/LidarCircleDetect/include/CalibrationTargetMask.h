#pragma once

#ifndef CALIBRATION_TARGET_MASK
#define CALIBRATION_TARGET_MASK

/** System Includes */
#include <thread>
#include <chrono>

// PCL Library
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

template <typename PointType>
class CalibrationTargetMask
{
    public:

        CalibrationTargetMask();

        ~CalibrationTargetMask();

        void visualizeTargetMaskCloud();

        /** Get a read-only reference to the target mask pointer */
        const typename pcl::PointCloud<PointType>::Ptr & getTargetMask() const
        {
            return m_target_mask;
        }

    private:
        bool isPointInsideCircles(const Point2D point_) const;

        typename pcl::PointCloud<PointType>::Ptr m_target_mask;

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
};

template <typename PointType>
CalibrationTargetMask<PointType>::CalibrationTargetMask() :
    m_target_mask {new pcl::PointCloud<PointType>}
{
    // Compute number of points along x and y directions
    const int points_x = m_x_size / m_resolution;
    const int points_y = m_y_size / m_resolution;
  

    std::cout << "[CalibrationTargetMask]: points_x: " << points_x
              << " - points_y: " << points_y << std::endl;

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

            PointType current_point;

            current_point.x = point.x;
            current_point.y = point.y;
            current_point.z = 0.0;

            m_target_mask->push_back(current_point);
        }
        
    }
}

template <typename PointType>
CalibrationTargetMask<PointType>::~CalibrationTargetMask()
{}

template <typename PointType>
void CalibrationTargetMask<PointType>::visualizeTargetMaskCloud()
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->addPointCloud<PointType>(m_target_mask, "Calibration Target Mask");
    
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             1,
                                             "Calibration Target Mask");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    while (! viewer->wasStopped() )
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    pcl::io::savePCDFileBinary("target_mask.pcd", *m_target_mask);
}

template <typename PointType>
bool CalibrationTargetMask<PointType>::isPointInsideCircles(const Point2D point_) const
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

#endif
