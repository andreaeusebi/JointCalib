#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

/** System Includes */
#include <thread>
#include <mutex>
#include <filesystem>
#include <iostream>
#include <fstream>

/* QT Includes */
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSlider>
#include <QLabel>

/* PCL Includes */
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

/* Eigen Includes */
#include <Eigen/Geometry> 

/* Local Includes */
#include "BaseCloudAligner.h"
#include "CalibrationTargetMask.h"  // to get access to Point2D class

// Template class derived from the base class
template<typename PointT>
class CloudAligner : public BaseCloudAligner
{
    public:
        CloudAligner(const std::string & map_pcd_file_,
                     const std::string & mask_pcd_file_,
                     const std::string & results_file_,
                     QWidget *parent_ = nullptr);

        ~CloudAligner();

    public slots:

        void filterMap() override;

        void transformPointCloud() override;

        void hideMask() override;

        void alignMask() override;

        void saveResults() override;

        void terminateProgram() override;

    private:
        /* ### ----- Class Methods ----- ### */

        void initUI();

        void visualizationThread();

        void appendHorizBoxLayoutToVertLayout(const std::string & horiz_layout_label_,
                                              QDoubleSpinBox * & spin_box_,
                                              const double & initial_value_,
                                              const double & min_value_,
                                              const double & max_value_,
                                              QVBoxLayout * const & vert_layout_);
        
        /* ### ----- Class Attributes ----- ### */

        /** Original map pointcloud, not displayed, keeped as source for filtering. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_cloud;

        /** Filtered map cloud on the target, displayed on the viewer. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_cloud_filtered;

        /** Pointcloud of the calibration target mask. */
        typename pcl::PointCloud<PointT>::Ptr m_mask_cloud;

        /** Pointcloud made of the calibration target circles centers.  */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_circles_centers_cloud;

        /** ID of the displayed map cloud, used to distinguish in the PCL visualizer. */
        std::string m_map_cloud_id;

        /** ID of the mask cloud, used to distinguish in the PCL visualizer. */
        std::string m_mask_cloud_id;

        /** PCL visualizer object. */
        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        /** Mutex for handling access to m_viewer from different threads. */
        std::mutex m_mutex_viewer;

        /**
         * Transform from Lidar to Mask frame. After alignement with target, this will hold the
         * transform from Lidar to estimated Target frame which is the output of this node.
        */
        Eigen::Affine3f m_transform_LM;

        /** Flag defining if mask-target alignement has been done. */
        bool m_alignement_done {false};

        /** Thread handling the PCL visualizer update. */
        std::thread m_vis_thread;

        /**
         * Atomic flag to control the visualization thread. It ensures that there are no race
         * conditions when checking or setting this flag from different threads.
        */
        std::atomic<bool> m_running {true};

        /** Flag defining if the mask is currently displayed in the visualizer. */
        std::atomic<bool> m_mask_shown {true};

        /** CSV file in which storing (appending if it already exists) the circle centers coordinates. */
        std::string m_results_file;

        /** Flag indicating wheter results have been saved or not. */
        bool m_results_saved {false};

        QDoubleSpinBox * m_filter_x_min_spin_box;

        QDoubleSpinBox * m_filter_x_max_spin_box;

        QDoubleSpinBox * m_filter_y_min_spin_box;

        QDoubleSpinBox * m_filter_y_max_spin_box;

        QDoubleSpinBox * m_filter_z_min_spin_box;

        QDoubleSpinBox * m_filter_z_max_spin_box;
        
        QDoubleSpinBox * m_trans_x_spin_box;

        QDoubleSpinBox * m_trans_y_spin_box;

        QDoubleSpinBox * m_trans_z_spin_box;

        QDoubleSpinBox * m_rot_x_spin_box;

        QDoubleSpinBox * m_rot_y_spin_box;

        QDoubleSpinBox * m_rot_z_spin_box;
};

template<typename PointT>
CloudAligner<PointT>::CloudAligner(const std::string & map_pcd_file_,
                                   const std::string & mask_pcd_file_,
                                   const std::string & results_file_,
                                   QWidget *parent_) :
    BaseCloudAligner        (parent_),
    m_map_cloud             {new pcl::PointCloud<pcl::PointXYZ>},
    m_map_cloud_filtered    {new pcl::PointCloud<pcl::PointXYZ>},
    m_mask_cloud            {new pcl::PointCloud<PointT>},
    m_circles_centers_cloud {new pcl::PointCloud<pcl::PointXYZRGB>},
    m_map_cloud_id          {"map"},
    m_mask_cloud_id         {"mask"},
    m_viewer                {new pcl::visualization::PCLVisualizer("CloudAligner")},
    m_transform_LM          {Eigen::Affine3f::Identity()},
    m_results_file          {results_file_}
{
    /* ----- Load Pointclouds from PCD files -----  */

    if (pcl::io::loadPCDFile(map_pcd_file_, *m_map_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", map_pcd_file_.c_str());
        return;
    }

    // Copy the original map to the filtered one (at the beginning they are identical)
    pcl::copyPointCloud(*m_map_cloud, *m_map_cloud_filtered);

    if (pcl::io::loadPCDFile(mask_pcd_file_, *m_mask_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", mask_pcd_file_.c_str());
        return;
    }

    /* ----- Load Visualizer with Map and Mask Clouds -----  */

    m_viewer->addPointCloud<pcl::PointXYZ>(m_map_cloud_filtered, m_map_cloud_id);
    m_viewer->addPointCloud<PointT>(m_mask_cloud, m_mask_cloud_id);

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               m_map_cloud_id);

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               m_mask_cloud_id);

    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->addCoordinateSystem(1.0);
    m_viewer->initCameraParameters();

    // Start visualization thread
    m_vis_thread = std::thread(&CloudAligner::visualizationThread, this);

    /* ----- Init User Interface -----  */

    this->initUI();
}

template<typename PointT>
CloudAligner<PointT>::~CloudAligner()
{
    std::cout << "~CloudAligner() BEGIN" << std::endl;

    // Set running flag to false and join the visualization thread
    m_running = false;

    if (m_vis_thread.joinable())
    {
        m_vis_thread.join();
    }

    std::cout << "~CloudAligner() END" << std::endl;
}

template<typename PointT>
void CloudAligner<PointT>::filterMap()
{
    std::cout << "[filterMap()]: BEGIN" << std::endl;

    pcl::PassThrough<pcl::PointXYZ> filter_x;
    filter_x.setInputCloud(m_map_cloud);
    filter_x.setFilterFieldName("x");
    filter_x.setFilterLimits(m_filter_x_min_spin_box->value(),
                             m_filter_x_max_spin_box->value());
                             
    filter_x.filter(*m_map_cloud_filtered);

    pcl::PassThrough<pcl::PointXYZ> filter_y;
    filter_y.setInputCloud(m_map_cloud_filtered);
    filter_y.setFilterFieldName("y");
    filter_y.setFilterLimits(m_filter_y_min_spin_box->value(),
                             m_filter_y_max_spin_box->value());
                             
    filter_y.filter(*m_map_cloud_filtered);

    pcl::PassThrough<pcl::PointXYZ> filter_z;
    filter_z.setInputCloud(m_map_cloud_filtered);
    filter_z.setFilterFieldName("z");
    filter_z.setFilterLimits(m_filter_z_min_spin_box->value(),
                             m_filter_z_max_spin_box->value());
                             
    filter_z.filter(*m_map_cloud_filtered);

    m_mutex_viewer.lock();
    m_viewer->updatePointCloud(m_map_cloud_filtered, m_map_cloud_id);
    m_mutex_viewer.unlock();
}

template<typename PointT>
void CloudAligner<PointT>::transformPointCloud()
{
    std::cout << "transformPointCloud()" << std::endl;

    // Check if the alignement has already been done, in that case avoid further transform
    if (m_alignement_done)
    {
        std::cout << "Alignement already done! Can't apply further transform!" << std::endl;
        return;
    }

    // Get translation values from the spin boxes
    float tx = m_trans_x_spin_box->value();
    float ty = m_trans_y_spin_box->value();
    float tz = m_trans_z_spin_box->value();

    // Get rotation angles from the spin boxes
    double angle_x = m_rot_x_spin_box->value() * M_PI / 180.0;
    double angle_y = m_rot_y_spin_box->value() * M_PI / 180.0;
    double angle_z = m_rot_z_spin_box->value() * M_PI / 180.0;

    // Create transformation matrices for each rotation
    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();

    // Apply rotations around each axis
    transform_x.rotate(Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX()));
    transform_y.rotate(Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY()));
    transform_z.rotate(Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ()));

    // Combine the rotations as XYZ sequence (wrt fixed axes)!!!
    Eigen::Affine3f combined_transform = transform_z * transform_y * transform_x;

    // Apply translation
    combined_transform.translation() << tx, ty, tz;

    // Apply the combined transformation to the point cloud visualization
    m_mutex_viewer.lock();
    m_viewer->updatePointCloudPose(m_mask_cloud_id, combined_transform);
    m_mutex_viewer.unlock();

    std::cout << "combined_transform:" << std::endl
              << combined_transform.matrix() << std::endl;

    // Keep the current transform saved
    m_transform_LM = combined_transform;
}

template<typename PointT>
void CloudAligner<PointT>::hideMask()
{
    std::cout << "hideMask() - BEGIN" << std::endl;

    if (m_mask_shown)
    {
        std::cout << "Removing pointcloud..." << std::endl;

        m_mutex_viewer.lock();
        m_viewer->removePointCloud(m_mask_cloud_id);
        m_mutex_viewer.unlock();

        m_mask_shown = false;
        std::cout << "Pointcloud removed" << std::endl;
    }
    else
    {
        std::cout << "Adding pointcloud..." << std::endl;
        
        m_mutex_viewer.lock();
        m_viewer->addPointCloud<PointT>(m_mask_cloud, m_mask_cloud_id);
        
        // Re-apply the latest visualization pose transform
        m_viewer->updatePointCloudPose(m_mask_cloud_id, m_transform_LM);
        m_mutex_viewer.unlock();

        m_mask_shown = true;

        std::cout << "Pointcloud added" << std::endl;
    }

    std::cout << "hideMask() - END" << std::endl;
}

template <typename PointT>
void CloudAligner<PointT>::alignMask()
{
    std::cout << "alignMask()" << std::endl;

    // Create a copy of mask cloud of type pcl::PointXYZ (in case mask is not of that type)
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_mask_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*m_mask_cloud, *temp_mask_cloud_xyz);

    // Apply transformation to temp mask to effectively send it into user defined pose
    pcl::transformPointCloud(*temp_mask_cloud_xyz, *temp_mask_cloud_xyz, m_transform_LM);

    /* ---- Run ICP between temp mask and filtered map cloud ----- */

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Map is input since it has lowest number of points belonging to the calibration target
    icp.setInputSource(m_map_cloud_filtered);

    // Mask is target since it has high point density, the idea is that there will exist one
    // corresponding point in the mask for each point in the real target cloud. 
    icp.setInputTarget(temp_mask_cloud_xyz);

    // Set parameters for ICP
    icp.setMaxCorrespondenceDistance(0.5);      // Maximum distance for correspondence
    icp.setMaximumIterations(2000);             // Maximum number of iterations
    // icp.setTransformationEpsilon(1e-8);         // Convergence criteria
    // icp.setEuclideanFitnessEpsilon(1);          // Convergence criteria

    std::cout << "Running ICP..." << std::endl;

    // Perform alignment (i.e. compute T_TM)
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    icp.align(*aligned_cloud_xyz);

    std::cout << "ICP Completed!" << std::endl;

    if (icp.hasConverged())
    {
        m_alignement_done = true;

        std::cout << "ICP has converged!!!" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
        
        // Transform from calibration target (source) to mask (target)
        Eigen::Affine3f T_target_mask;
        T_target_mask.matrix() = icp.getFinalTransformation();

        std::cout << "T_target_mask (from ICP):" << std::endl
                  << T_target_mask.matrix() << std::endl;

        // Get convergence criteria
        auto criteria = icp.getConvergeCriteria();

        std::cout << "Convergence reason: " << criteria->getConvergenceState() << std::endl;

        // ----- Compute total transform from lidar to estimated target position ----- //
        
        /* 
         * Total transform is the concatenation of the first transform from lidar
         * to mask provided by the user, and the (inverse) of the transform
         * computed by the ICP.
         * Remark that since both these transforms are performed wrt the fixed lidar frame
         * axes we have to premultiply the ICP transform!!
         * Thus:
         *  
         * T_LT = T_LM (updated) = T_ICP^-1 * T_LM (from user)  
         */

        std::cout << "T_target_mask inverse:" << std::endl
                  << T_target_mask.inverse().matrix() << std::endl;

        std::cout << "m_transform_LM PRE: " << std::endl
                  << m_transform_LM.matrix() << std::endl;

        m_transform_LM = T_target_mask.inverse() * m_transform_LM;

        std::cout << "Final estimated transform from lidar to target:" << std::endl
                  << m_transform_LM.matrix() << std::endl;

        // Update visualization
        m_mutex_viewer.lock();
        m_viewer->updatePointCloudPose(m_mask_cloud_id, m_transform_LM);
        m_mutex_viewer.unlock();

        // ----- Compute circle centers wrt estimated target pose ----- //

        // Coordinates (in meters) of the holes centers in the calibration target
        // REMARK: FOR NOW THEY ARE COPYED AND PASTED HERE FROM CALIBRATION TARGET
        // WE CAN'T USE CALIB TARGET OBJECT BECAUSE IT HAS A PCL VIEWER INSIDE!!
        // SOLVE THIS ISSUE!!

        const std::vector<Point2D> circles_centers {
            Point2D(0.3, 0.3),  // top left circle
            Point2D(0.3, 0.9),  // top right circle
            Point2D(0.9, 0.3),  // bottom left circle
            Point2D(0.9, 0.9)   // bottom right circle
        };

        for (auto i_cc_iter = circles_centers.cbegin();
             i_cc_iter != circles_centers.cend();
             ++i_cc_iter)
        {
            pcl::PointXYZRGB cc_cloud_point;

            cc_cloud_point.x = i_cc_iter->x;
            cc_cloud_point.y = i_cc_iter->y;
            cc_cloud_point.z = 0.0;
            cc_cloud_point.r = 255;
            cc_cloud_point.g = 255;
            cc_cloud_point.b = 0;

            m_circles_centers_cloud->push_back(cc_cloud_point);
        }

        // Transform into extimated target frame
        pcl::transformPointCloud(*m_circles_centers_cloud, *m_circles_centers_cloud, m_transform_LM);

        // Display circle centers into the visualizer
        m_mutex_viewer.lock();
        m_viewer->addPointCloud<pcl::PointXYZRGB>(m_circles_centers_cloud, "circles_centers");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   5,
                                                   "circles_centers");
        m_mutex_viewer.unlock();

        std::cout << "Coordinates of the circles center wrt lidar frame:" << std::endl;

        for (auto cc_point_iter = m_circles_centers_cloud->begin();
             cc_point_iter != m_circles_centers_cloud->end();
             ++cc_point_iter)
        {
            std::cout << "(" << cc_point_iter->x << ","
                             << cc_point_iter->y << ","
                             << cc_point_iter->z
                      << ")" << std::endl;
        }
    }
    else
    {
        std::cout << "G-ICP did not converge." << std::endl;

        this->terminateProgram();
    }
}

template <typename PointT>
void CloudAligner<PointT>::saveResults()
{
    std::cout << "saveResults()" << std::endl;

    if (m_circles_centers_cloud->size() != 4)
    {
        std::cout << "ERROR! m_circles_centers_cloud->size() != 4" << std::endl;
        exit(EXIT_FAILURE);
    }

    bool file_exists = std::filesystem::exists(m_results_file);

    std::ofstream file;

    // Open file in append mode
    file.open(m_results_file, std::ios_base::app);

    if (! file.is_open())
    {
        std::cout << "ERROR! File not opened!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // If file doesn't exist, write the header
    if (!file_exists)
    {
        std::string header = "left_top_x,left_top_y,left_top_z,"
                             "right_top_x,right_top_y,right_top_z,"
                             "right_bottom_x,right_bottom_y,right_bottom_z,"
                             "left_bottom_x,left_bottom_y,left_bottom_z";

        file << header << '\n';
    }

    // Collect points coodinates
    pcl::PointXYZRGB point_top_left     {m_circles_centers_cloud->points[0]};
    pcl::PointXYZRGB point_top_right    {m_circles_centers_cloud->points[1]};
    pcl::PointXYZRGB point_bottom_right {m_circles_centers_cloud->points[3]}; // bottom right is the fourth!
    pcl::PointXYZRGB point_bottom_left  {m_circles_centers_cloud->points[2]}; // bottom left is the third!

    // Append the content
    file << point_top_left.x     << "," << point_top_left.y     << "," << point_top_left.z     << ","
         << point_top_right.x    << "," << point_top_right.y    << "," << point_top_right.z    << ","
         << point_bottom_right.x << "," << point_bottom_right.y << "," << point_bottom_right.z << ","
         << point_bottom_left.x  << "," << point_bottom_left.y  << "," << point_bottom_left.z  << "\n";

    file.close();

    std::cout << "Results correctly saved!" << std::endl;

    m_results_saved = true;
}

template <typename PointT>
void CloudAligner<PointT>::terminateProgram()
{
    std::cout << "terminateProgram()" << std::endl;

    m_running = false;
    qApp->quit();
}

template <typename PointT>
void CloudAligner<PointT>::initUI()
{
    std::cout << "initUI() BEGIN" << std::endl;

    /* ----- Set up QT stuff ----- */

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);

    /* ----- X Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map X Min:",
                                           m_filter_x_min_spin_box,
                                           -100.0, -100.0, 100.0,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map X Max:",
                                           m_filter_x_max_spin_box,
                                           100.0, -100.0, 100.0,
                                           layout);

    /* ----- Y Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map Y Min:",
                                           m_filter_y_min_spin_box,
                                           -100.0, -100.0, 100.0,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map Y Max:",
                                           m_filter_y_max_spin_box,
                                           100.0, -100.0, 100.0,
                                           layout);

    /* ----- Z Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map Z Min:",
                                           m_filter_z_min_spin_box,
                                           -100.0, -100.0, 100.0,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map Z Max:",
                                           m_filter_z_max_spin_box,
                                           100.0, -100.0, 100.0,
                                           layout);

    /* ----- Map Cloud Filter button ----- */

    QPushButton *map_filter_button = new QPushButton("Filter Map");
    layout->addWidget(map_filter_button);
    connect(map_filter_button, &QPushButton::clicked, this, &CloudAligner::filterMap);

    /* ----- X Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans X:",
                                           m_trans_x_spin_box,
                                           0.0, -100.0, 100.0,
                                           layout);

    /* ----- Y Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans Y:",
                                           m_trans_y_spin_box,
                                           0.0, -100.0, 100.0,
                                           layout);

    /* ----- Z Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans Z:",
                                           m_trans_z_spin_box,
                                           0.0, -100.0, 100.0,
                                           layout);

    /* ----- X Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot X:",
                                           m_rot_x_spin_box,
                                           0.0, -180.0, 180.0,
                                           layout);

    /* ----- Y Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot Y:",
                                           m_rot_y_spin_box,
                                           0.0, -180.0, 180.0,
                                           layout);

    /* ----- Z Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot Z:",
                                           m_rot_z_spin_box,
                                           0.0, -180.0, 180.0,
                                           layout);

    /* ----- Transform button ----- */

    QPushButton *transform_button = new QPushButton("Transform");
    layout->addWidget(transform_button);
    connect(transform_button, &QPushButton::clicked, this, &CloudAligner::transformPointCloud);

    /* ----- Hide Mask button ----- */

    QPushButton *hide_mask_button = new QPushButton("Hide/Show Mask");
    layout->addWidget(hide_mask_button);
    connect(hide_mask_button, &QPushButton::clicked, this, &CloudAligner::hideMask);

    /* ----- Align Mask button ----- */

    QPushButton *align_mask_button = new QPushButton("Align Mask");
    layout->addWidget(align_mask_button);
    connect(align_mask_button, &QPushButton::clicked, this, &CloudAligner::alignMask);

    /* ----- Save Results button ----- */

    QPushButton *save_results_button = new QPushButton("Save Results");
    layout->addWidget(save_results_button);
    connect(save_results_button, &QPushButton::clicked, this, &CloudAligner::saveResults);

    /* ----- Terminate button ----- */

    QPushButton *terminateButton = new QPushButton("Terminate");
    layout->addWidget(terminateButton);
    connect(terminateButton, &QPushButton::clicked, this, &CloudAligner::terminateProgram);

    std::cout << "initUI() END" << std::endl;
}

template <typename PointT>
void CloudAligner<PointT>::visualizationThread()
{
    std::cout << "visualizationThread() BEGIN" << std::endl;

    while (m_running)
    {
        m_mutex_viewer.lock();
        m_viewer->spinOnce(100); // Update viewer
        m_mutex_viewer.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
    }

    std::cout << "visualizationThread() END" << std::endl;
}

template <typename PointT>
void CloudAligner<PointT>::appendHorizBoxLayoutToVertLayout(const std::string & horiz_layout_label_,
                                                            QDoubleSpinBox * & spin_box_,
                                                            const double & initial_value_,
                                                            const double & min_value_,
                                                            const double & max_value_,
                                                            QVBoxLayout * const & vert_layout_)
{
    // Create a QLabel to act as the label for the spin box
    QLabel * q_label = new QLabel(horiz_layout_label_.c_str());

    // Create Spin Box
    spin_box_ = new QDoubleSpinBox;
    spin_box_->setRange(min_value_, max_value_);
    spin_box_->setSingleStep(1.0);
    spin_box_->setValue(initial_value_);

    // Create the horizontal box layout object
    QHBoxLayout * hbox_layout = new QHBoxLayout;

    // Add the label and the spin box to the layout
    hbox_layout->addWidget(q_label);
    hbox_layout->addWidget(spin_box_);
 
    /*
     * Add the QHBoxLayout to the existing QVBoxLayout.
     * Don't worry about pointed memory region handling: in Qt the parent takes ownership
     * of the added layout. Specifically, vert_layout_ will be responsible for deleting
     * hbox_layout when vert_layout_ itself is deleted, ensuring that there are no memory leaks.
     */
    vert_layout_->addLayout(hbox_layout);
}

#endif // POINT_CLOUD_VIEWER_H
