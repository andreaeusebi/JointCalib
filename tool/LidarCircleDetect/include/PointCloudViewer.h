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
    m_viewer            {new pcl::visualization::PCLVisualizer("PointCloudViewer")}
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
        std::cout << "visualizationThread() - while() - BEGIN" << std::endl;

        m_viewer->spinOnce(100); // Update viewer
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage

        std::cout << "visualizationThread() - while() - END" << std::endl;
    }

    std::cout << "visualizationThread() END" << std::endl;
}

#endif // POINT_CLOUD_VIEWER_H
