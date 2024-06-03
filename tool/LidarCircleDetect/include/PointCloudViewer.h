#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

/** System Includes */
#include <thread>

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

// Template class derived from the base class
template<typename PointT>
class PointCloudViewer : public BaseCloudAligner
{
    public:
        PointCloudViewer(const std::string & map_pcd_file_,
                         const std::string & mask_pcd_file_,
                         QWidget *parent_ = nullptr);

        ~PointCloudViewer();

    public slots:

        void filterMap() override;

        void transformPointCloud() override;

        void hideMask() override;

        void alignMask() override;

        void terminateProgram() override;

    private:
        /* ### ----- Class Methods ----- ### */

        void initUI();

        void visualizationThread();

        void appendHorizBoxLayoutToVertLayout(const std::string & horiz_layout_label_,
                                              const double & spin_box_initial_value_,
                                              QDoubleSpinBox * & spin_box_,
                                              QVBoxLayout * const & vert_layout_);
        
        /* ### ----- Class Attributes ----- ### */

        /** Original map pointcloud, not displayed, keeped as source for filtering. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_cloud;

        /** Filtered map cloud on the target, displayed on the viewer. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_cloud_filtered;

        /** Pointcloud of the calibration target mask. */
        typename pcl::PointCloud<PointT>::Ptr m_mask_cloud;

        std::string m_map_cloud_id;

        std::string m_mask_cloud_id;

        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        /** Transform from Lidar to Target frame. (###### ----- THIS IS LIDAR TO MASK!! ---- #####) */
        Eigen::Affine3f m_transform_LT;

        std::thread m_vis_thread;

        // Atomic flag to control the visualization thread
        // It ensures that there are no race conditions when checking or setting this flag from different threads.
        std::atomic<bool> m_running {true};

        bool m_mask_shown {true}; 

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
PointCloudViewer<PointT>::PointCloudViewer(const std::string & map_pcd_file_,
                                           const std::string & mask_pcd_file_,
                                           QWidget *parent_) :
    BaseCloudAligner    (parent_),
    m_map_cloud             {new pcl::PointCloud<pcl::PointXYZ>},
    m_map_cloud_filtered    {new pcl::PointCloud<pcl::PointXYZ>},
    m_mask_cloud            {new pcl::PointCloud<PointT>},
    m_map_cloud_id          {"map"},
    m_mask_cloud_id         {"mask"},
    m_viewer                {new pcl::visualization::PCLVisualizer("PointCloudViewer")},
    m_transform_LT          {Eigen::Affine3f::Identity()}
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
    m_vis_thread = std::thread(&PointCloudViewer::visualizationThread, this);

    /* ----- Init User Interface -----  */

    this->initUI();
}

template<typename PointT>
PointCloudViewer<PointT>::~PointCloudViewer()
{
    std::cout << "~PointCloudViewer() BEGIN" << std::endl;

    // Set running flag to false and join the visualization thread
    m_running = false;

    if (m_vis_thread.joinable())
    {
        m_vis_thread.join();
    }

    std::cout << "~PointCloudViewer() END" << std::endl;
}

template<typename PointT>
void PointCloudViewer<PointT>::filterMap()
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

    m_viewer->updatePointCloud(m_map_cloud_filtered, m_map_cloud_id);
}

template<typename PointT>
void PointCloudViewer<PointT>::transformPointCloud()
{
    std::cout << "transformPointCloud()" << std::endl;

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

    // Combine the transformations
    Eigen::Affine3f combined_transform = transform_z * transform_y * transform_x;

    // Apply translation
    combined_transform.translation() << tx, ty, tz;

    // Apply the combined transformation to the point cloud
    m_viewer->updatePointCloudPose(m_mask_cloud_id, combined_transform);

    std::cout << "combined_transform:" << std::endl
              << combined_transform.matrix() << std::endl;

    m_transform_LT = combined_transform;
}

template<typename PointT>
void PointCloudViewer<PointT>::hideMask()
{
    if (m_mask_shown)
    {
        m_viewer->removePointCloud(m_mask_cloud_id);

        m_mask_shown = false;
    }
    else
    {
        m_viewer->addPointCloud<PointT>(m_mask_cloud, m_mask_cloud_id);
        this->transformPointCloud();

        m_mask_shown = true;
    }
}

template <typename PointT>
void PointCloudViewer<PointT>::alignMask()
{
    std::cout << "alignMask()" << std::endl;

    std::cout << "m_transform_LT:" << std::endl
              << m_transform_LT.matrix() << std::endl;
    std::cout << "m_transform_LT inverse:" << std::endl
              << m_transform_LT.inverse().matrix() << std::endl;

    // Create a new temporary pointcloud object identical to the mask
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_mask_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Copy the data from the original to the copy
    pcl::copyPointCloud(*m_mask_cloud, *temp_mask_cloud);

    // Change the color to blue to distinguish it from original mask
    for (auto it = temp_mask_cloud->points.begin(); it != temp_mask_cloud->points.end(); ++it)
    {
        it->r = 0;
        it->g = 0;
        it->b = 255;
    }

    // Transform temp mask using latest transform from user
    pcl::transformPointCloud(*temp_mask_cloud, *temp_mask_cloud, m_transform_LT);

    // Display as check
    m_viewer->addPointCloud<pcl::PointXYZRGB>(temp_mask_cloud, "temp_mask_cloud");

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               "temp_mask_cloud");

    // Convert temp point cloud from pcl::PointXYZRGB to pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_mask_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*temp_mask_cloud, *temp_mask_cloud_xyz);

    // Run ICP between temp mask and filtered map cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setInputSource(m_map_cloud_filtered);
    icp.setInputTarget(temp_mask_cloud_xyz);

    // Set parameters for G-ICP
    icp.setMaxCorrespondenceDistance(0.5);      // Maximum distance for correspondence
    icp.setMaximumIterations(2000);             // Maximum number of iterations
    // icp.setTransformationEpsilon(1e-8);         // Convergence criteria
    // icp.setEuclideanFitnessEpsilon(1);          // Convergence criteria

    std::cout << "Running ICP..." << std::endl;

    // Perform alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    icp.align(*aligned_cloud_xyz);

    std::cout << "ICP Completed!" << std::endl;

    if (icp.hasConverged())
    {
        std::cout << "ICP has converged." << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
        
        Eigen::Affine3f transform_target_mask;
        transform_target_mask.matrix() = icp.getFinalTransformation();

        std::cout << "Transformation matrix: " << std::endl
                  << transform_target_mask.matrix() << std::endl;

        // Get convergence criteria
        auto criteria = icp.getConvergeCriteria();

        std::cout << "Convergence reason: " << criteria->getConvergenceState() << std::endl;

        // Apply the inverse of the just computed transform to move the mask into target position
        pcl::transformPointCloud(*temp_mask_cloud,
                                 *temp_mask_cloud,
                                 transform_target_mask.matrix().inverse());

        // Update visualization
        m_viewer->updatePointCloud(temp_mask_cloud, "temp_mask_cloud");

        // Compute circle centers wrt new mask pose

        // Compute overall transform from Lidar to (estimated) Target
        // cc_T = T_TL * cc_L
        // T_TL = T_TM * T_ML (to understand better)

    }
    else
    {
        std::cout << "G-ICP did not converge." << std::endl;
    }
}

template <typename PointT>
void PointCloudViewer<PointT>::terminateProgram()
{
    std::cout << "terminateProgram()" << std::endl;

    m_running = false;
    qApp->quit();
}

template <typename PointT>
void PointCloudViewer<PointT>::initUI()
{
    std::cout << "initUI() BEGIN" << std::endl;

    /* ----- Set up QT stuff ----- */

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);

    /* ----- X Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map X Min:",
                                           -100.0,
                                           m_filter_x_min_spin_box,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map X Max:",
                                           100.0,
                                           m_filter_x_max_spin_box,
                                           layout);

    /* ----- Y Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map Y Min:",
                                           -100.0,
                                           m_filter_y_min_spin_box,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map Y Max:",
                                           100.0,
                                           m_filter_y_max_spin_box,
                                           layout);

    /* ----- Z Axis PassThrough Filter Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Map Z Min:",
                                           -100.0,
                                           m_filter_z_min_spin_box,
                                           layout);

    this->appendHorizBoxLayoutToVertLayout("Map Z Max:",
                                           100.0,
                                           m_filter_z_max_spin_box,
                                           layout);

    /* ----- Map Cloud Filter button ----- */

    QPushButton *map_filter_button = new QPushButton("Filter Map");
    layout->addWidget(map_filter_button);
    connect(map_filter_button, &QPushButton::clicked, this, &PointCloudViewer::filterMap);

    /* ----- X Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans X:",
                                           0.0,
                                           m_trans_x_spin_box,
                                           layout);

    /* ----- Y Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans Y:",
                                           0.0,
                                           m_trans_y_spin_box,
                                           layout);

    /* ----- Z Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Trans Z:",
                                           0.0,
                                           m_trans_z_spin_box,
                                           layout);

    /* ----- X Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot X:",
                                           0.0,
                                           m_rot_x_spin_box,
                                           layout);

    /* ----- Y Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot Y:",
                                           0.0,
                                           m_rot_y_spin_box,
                                           layout);

    /* ----- Z Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    this->appendHorizBoxLayoutToVertLayout("Rot Z:",
                                           0.0,
                                           m_rot_z_spin_box,
                                           layout);

    /* ----- Transform button ----- */

    QPushButton *transform_button = new QPushButton("Transform");
    layout->addWidget(transform_button);
    connect(transform_button, &QPushButton::clicked, this, &PointCloudViewer::transformPointCloud);

    /* ----- Hide Mask button ----- */

    QPushButton *hide_mask_button = new QPushButton("Hide/Show Mask");
    layout->addWidget(hide_mask_button);
    connect(hide_mask_button, &QPushButton::clicked, this, &PointCloudViewer::hideMask);

    /* ----- Align Mask button ----- */

    QPushButton *align_mask_button = new QPushButton("Align Mask");
    layout->addWidget(align_mask_button);
    connect(align_mask_button, &QPushButton::clicked, this, &PointCloudViewer::alignMask);

    /* ----- Terminate button ----- */

    QPushButton *terminateButton = new QPushButton("Terminate");
    layout->addWidget(terminateButton);
    connect(terminateButton, &QPushButton::clicked, this, &PointCloudViewer::terminateProgram);

    std::cout << "initUI() END" << std::endl;
}

template <typename PointT>
void PointCloudViewer<PointT>::visualizationThread()
{
    std::cout << "visualizationThread() BEGIN" << std::endl;

    while (m_running)
    {
        m_viewer->spinOnce(100); // Update viewer
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
    }

    std::cout << "visualizationThread() END" << std::endl;
}

template <typename PointT>
void PointCloudViewer<PointT>::appendHorizBoxLayoutToVertLayout(
    const std::string & horiz_layout_label_,
    const double & spin_box_initial_value_,
    QDoubleSpinBox * & spin_box_,
    QVBoxLayout * const & vert_layout_
)
{
    // Create a QLabel to act as the label for the spin box
    QLabel * q_label = new QLabel(horiz_layout_label_.c_str());

    // Create Spin Box
    spin_box_ = new QDoubleSpinBox;
    spin_box_->setRange(-100.0, 100.0);
    spin_box_->setSingleStep(1.0);
    spin_box_->setValue(spin_box_initial_value_);

    // Create the horizontal box layout object
    QHBoxLayout * hbox_layout = new QHBoxLayout;

    // Add the label and the spin box to the layout
    hbox_layout->addWidget(q_label);
    hbox_layout->addWidget(spin_box_);

    /* Add the QHBoxLayout to the existing QVBoxLayout
       Don't worry about pointed memory region handling: in Qt the parent takes ownership
       of the added layout. Specifically, vert_layout_ will be responsible for deleting
       hbox_layout when vert_layout_ itself is deleted, ensuring that there are no memory leaks.
    */
    vert_layout_->addLayout(hbox_layout);
}

#endif // POINT_CLOUD_VIEWER_H
