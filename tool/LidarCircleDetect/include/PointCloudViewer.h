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
#include <pcl/registration/gicp.h>

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
                         const std::string & target_pcd_file_,
                         QWidget *parent_ = nullptr);

        ~PointCloudViewer();

    public slots:

        void transformPointCloud() override;

        void hideTarget() override;

        void alignTarget() override;

        void terminateProgram() override;

    private:
        /* ### ----- Class Methods ----- ### */

        void initUI();

        void visualizationThread();

        /* ### ----- Class Attributes ----- ### */

        pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_cloud;

        typename pcl::PointCloud<PointT>::Ptr m_target_cloud;

        std::string m_map_cloud_id;

        std::string m_target_cloud_id;

        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        Eigen::Affine3f m_transform_LT;

        QDoubleSpinBox * m_trans_x_spin_box;

        QDoubleSpinBox * m_trans_y_spin_box;

        QDoubleSpinBox * m_trans_z_spin_box;

        QDoubleSpinBox * m_rot_x_spin_box;

        QDoubleSpinBox * m_rot_y_spin_box;

        QDoubleSpinBox * m_rot_z_spin_box;

        std::thread m_vis_thread;

        // Atomic flag to control the visualization thread
        // It ensures that there are no race conditions when checking or setting this flag from different threads.
        std::atomic<bool> m_running {true};

        bool m_target_shown {true}; 
};

template<typename PointT>
PointCloudViewer<PointT>::PointCloudViewer(const std::string & map_pcd_file_,
                                           const std::string & target_pcd_file_,
                                           QWidget *parent_) :
    BaseCloudAligner    (parent_),
    m_map_cloud         {new pcl::PointCloud<pcl::PointXYZ>},
    m_target_cloud      {new pcl::PointCloud<PointT>},
    m_map_cloud_id      {"map"},
    m_target_cloud_id   {"target"},
    m_viewer            {new pcl::visualization::PCLVisualizer("PointCloudViewer")},
    m_transform_LT      {Eigen::Affine3f::Identity()}
{
    /* ----- Load Pointclouds from PCD files -----  */

    if (pcl::io::loadPCDFile(map_pcd_file_, *m_map_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", map_pcd_file_.c_str());
        return;
    }

    if (pcl::io::loadPCDFile(target_pcd_file_, *m_target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", target_pcd_file_.c_str());
        return;
    }

    /* ----- Load Visualizer with Map and Target Clouds -----  */

    m_viewer->addPointCloud<pcl::PointXYZ>(m_map_cloud, m_map_cloud_id);
    m_viewer->addPointCloud<PointT>(m_target_cloud, m_target_cloud_id);

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               m_map_cloud_id);

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               m_target_cloud_id);

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
    m_viewer->updatePointCloudPose(m_target_cloud_id, combined_transform);

    std::cout << "combined_transform:" << std::endl
              << combined_transform.matrix() << std::endl;

    m_transform_LT = combined_transform;
}

template<typename PointT>
void PointCloudViewer<PointT>::hideTarget()
{
    if (m_target_shown)
    {
        m_viewer->removePointCloud(m_target_cloud_id);

        m_target_shown = false;
    }
    else
    {
        m_viewer->addPointCloud<PointT>(m_target_cloud, m_target_cloud_id);
        this->transformPointCloud();

        m_target_shown = true;
    }
}

template <typename PointT>
void PointCloudViewer<PointT>::alignTarget()
{
    std::cout << "alignTarget()" << std::endl;

    std::cout << "m_transform_LT:" << std::endl
              << m_transform_LT.matrix() << std::endl;
    std::cout << "m_transform_LT inverse:" << std::endl
              << m_transform_LT.inverse().matrix() << std::endl;

    // Create a new pointcloud object identical to target (temporary)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Copy data
    pcl::copyPointCloud(*m_target_cloud, *temp_cloud);

    // Change the color to blue to distinguish it from original target
    for (auto it = temp_cloud->points.begin(); it != temp_cloud->points.end(); ++it)
    {
        it->r = 255;
        it->g = 255;
        it->b = 0;
    }

    // Transform temp target using latest transform from user
    pcl::transformPointCloud(*temp_cloud, *temp_cloud, m_transform_LT);

    // Maybe display as check (display temp cloud)
    m_viewer->addPointCloud<pcl::PointXYZRGB>(temp_cloud, "temp_cloud");

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               "temp_cloud");

    // Convert temp point cloud from pcl::PointXYZRGB to pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*temp_cloud, *temp_cloud_xyz);

    // Run g-icp between temp target and map cloud
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> g_icp;

    g_icp.setInputSource(temp_cloud_xyz);
    g_icp.setInputTarget(m_map_cloud);

    // Set parameters for G-ICP
    g_icp.setMaxCorrespondenceDistance(0.5);   // Maximum distance for correspondence
    g_icp.setMaximumIterations(800);             // Maximum number of iterations
    // g_icp.setTransformationEpsilon(1e-8);       // Convergence criteria
    // g_icp.setEuclideanFitnessEpsilon(1);        // Convergence criteria

    std::cout << "Running ICP..." << std::endl;

    // Perform alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    g_icp.align(*aligned_cloud_xyz);

    std::cout << "ICP Completed!" << std::endl;

    if (g_icp.hasConverged())
    {
        std::cout << "G-ICP has converged." << std::endl;
        std::cout << "Fitness score: " << g_icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix: " << std::endl << g_icp.getFinalTransformation() << std::endl;
    }
    else
    {
        std::cout << "G-ICP did not converge." << std::endl;
    }

    // Convert aligned point cloud from pcl::PointXYZ to pcl::PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*aligned_cloud_xyz, *aligned_cloud);

    // Change the color to yellow to distinguish it from original target
    for (auto it = aligned_cloud->points.begin(); it != aligned_cloud->points.end(); ++it)
    {
        it->r = 0;
        it->g = 0;
        it->b = 255;
    }

    // Display result (apply updatePointCloudPose to original target cloud or create a new temp copy e apply updatepose to it)
    m_viewer->addPointCloud<pcl::PointXYZRGB>(aligned_cloud, "aligned_cloud");

    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               "aligned_cloud");

    // Remove temp cloud from visualization
    m_viewer->removePointCloud("temp_cloud");

    // Compute overall transform (which is pre transform * g-icp transform)
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
    std::cout << "init() BEGIN" << std::endl;

    /* ----- Set up QT stuff ----- */

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);

    /* ----- X Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */
    QHBoxLayout * trans_x_hbox_layout = new QHBoxLayout;

    m_trans_x_spin_box = new QDoubleSpinBox;
    m_trans_x_spin_box->setRange(-100.0, 100.0);
    m_trans_x_spin_box->setSingleStep(1.0);
    m_trans_x_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * trans_x_label = new QLabel("Trans X:");

    // Add the label and the spin box to the layout
    trans_x_hbox_layout->addWidget(trans_x_label);
    trans_x_hbox_layout->addWidget(m_trans_x_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(trans_x_hbox_layout);

    /* ----- Y Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */
    QHBoxLayout * trans_y_hbox_layout = new QHBoxLayout;

    m_trans_y_spin_box = new QDoubleSpinBox;
    m_trans_y_spin_box->setRange(-100.0, 100.0);
    m_trans_y_spin_box->setSingleStep(1.0);
    m_trans_y_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * trans_y_label = new QLabel("Trans Y:");

    // Add the label and the spin box to the layout
    trans_y_hbox_layout->addWidget(trans_y_label);
    trans_y_hbox_layout->addWidget(m_trans_y_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(trans_y_hbox_layout);

    /* ----- Z Axis Translation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */
    QHBoxLayout * trans_z_hbox_layout = new QHBoxLayout;

    m_trans_z_spin_box = new QDoubleSpinBox;
    m_trans_z_spin_box->setRange(-100.0, 100.0);
    m_trans_z_spin_box->setSingleStep(1.0);
    m_trans_z_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * trans_z_label = new QLabel("Trans Z:");

    // Add the label and the spin box to the layout
    trans_z_hbox_layout->addWidget(trans_z_label);
    trans_z_hbox_layout->addWidget(m_trans_z_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(trans_z_hbox_layout);

    /* ----- X Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */
    QHBoxLayout * rot_x_hbox_layout = new QHBoxLayout;

    m_rot_x_spin_box = new QDoubleSpinBox;
    m_rot_x_spin_box->setRange(-360.0, 360.0);
    m_rot_x_spin_box->setSingleStep(1.0);
    m_rot_x_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * rot_x_label = new QLabel("Rot X:");

    // Add the label and the spin box to the layout
    rot_x_hbox_layout->addWidget(rot_x_label);
    rot_x_hbox_layout->addWidget(m_rot_x_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(rot_x_hbox_layout);

    /* ----- Y Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    QHBoxLayout * rot_y_hbox_layout = new QHBoxLayout;

    m_rot_y_spin_box = new QDoubleSpinBox;
    m_rot_y_spin_box->setRange(-360.0, 360.0);
    m_rot_y_spin_box->setSingleStep(1.0);
    m_rot_y_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * rot_y_label = new QLabel("Rot Y:");

    // Add the label and the spin box to the layout
    rot_y_hbox_layout->addWidget(rot_y_label);
    rot_y_hbox_layout->addWidget(m_rot_y_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(rot_y_hbox_layout);

    /* ----- Z Axis Rotation Horizontal Layout (QLabel + QDoubleSpinBox) ----- */

    QHBoxLayout * rot_z_hbox_layout = new QHBoxLayout;

    m_rot_z_spin_box = new QDoubleSpinBox;
    m_rot_z_spin_box->setRange(-360.0, 360.0);
    m_rot_z_spin_box->setSingleStep(1.0);
    m_rot_z_spin_box->setValue(0.0);

    // Create a QLabel to act as the label for the spin box
    QLabel * rot_z_label = new QLabel("Rot Z:");

    // Add the label and the spin box to the layout
    rot_z_hbox_layout->addWidget(rot_z_label);
    rot_z_hbox_layout->addWidget(m_rot_z_spin_box);

    // Add the QHBoxLayout to the existing QVBoxLayout
    layout->addLayout(rot_z_hbox_layout);

    /* ----- Transform button ----- */
    QPushButton *transform_button = new QPushButton("Transform");
    layout->addWidget(transform_button);
    connect(transform_button, &QPushButton::clicked, this, &PointCloudViewer::transformPointCloud);

    /* ----- Hide Target button ----- */
    QPushButton *hide_target_button = new QPushButton("Hide/Show Target");
    layout->addWidget(hide_target_button);
    connect(hide_target_button, &QPushButton::clicked, this, &PointCloudViewer::hideTarget);

    /* ----- Align Target button ----- */
    QPushButton *align_target_button = new QPushButton("Align Target");
    layout->addWidget(align_target_button);
    connect(align_target_button, &QPushButton::clicked, this, &PointCloudViewer::alignTarget);

    /* ----- Terminate button ----- */
    QPushButton *terminateButton = new QPushButton("Terminate");
    layout->addWidget(terminateButton);
    connect(terminateButton, &QPushButton::clicked, this, &PointCloudViewer::terminateProgram);

    std::cout << "init() END" << std::endl;
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

#endif // POINT_CLOUD_VIEWER_H
