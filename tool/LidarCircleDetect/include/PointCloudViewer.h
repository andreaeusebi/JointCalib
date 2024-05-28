#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <thread>

#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSlider>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "BasePointCloudViewer.h"

// Template class derived from the base class
template<typename PointT>
class PointCloudViewer : public BasePointCloudViewer
{
    public:
        PointCloudViewer(const std::string& pcd_file, QWidget *parent = nullptr);

        PointCloudViewer(pcl::visualization::PCLVisualizer::Ptr & viewer,
                         const std::string & cloud_id, 
                         QWidget *parent = nullptr);

        ~PointCloudViewer();

    public slots:
        void rotatePointCloud() override;

        void translatePointCloud() override;

        void terminateProgram() override;

    private:
        void init();

        void visualizationThread();

        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        std::string m_cloud_id;

        QDoubleSpinBox * m_rotation_spin_box;
        
        QSlider * m_translation_x_slider;

        std::thread m_vis_thread;

        // Atomic flag to control the visualization thread
        // It ensures that there are no race conditions when checking or setting this flag from different threads.
        std::atomic<bool> m_running {true};
};

// Template class member function implementations
template<typename PointT>
PointCloudViewer<PointT>::PointCloudViewer(const std::string& pcd_file, QWidget *parent) :
    BasePointCloudViewer(parent),
    m_viewer            {new pcl::visualization::PCLVisualizer("PointCloudViewer")},
    m_cloud_id          {"cloud"}
{
    /* ----- Load PCD -----  */

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_file.c_str());
        return;
    }

    /* ----- Set up cloud visualization ----- */

    m_viewer->addPointCloud<PointT>(cloud, m_cloud_id);

    this->init();
}

template<typename PointT>
PointCloudViewer<PointT>::PointCloudViewer(pcl::visualization::PCLVisualizer::Ptr & viewer,
                                           const std::string & cloud_id,
                                           QWidget *parent) :
    BasePointCloudViewer(parent),
    m_viewer            (viewer),
    m_cloud_id          {cloud_id}
{
    std::cout << "PointCloudViewer() BEGIN" << std::endl;

    this->init();

    std::cout << "PointCloudViewer() END" << std::endl;
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
void PointCloudViewer<PointT>::rotatePointCloud()
{
    std::cout << "rotatePointCloud()" << std::endl;

    double angle = m_rotation_spin_box->value() * M_PI / 180.0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
    m_viewer->updatePointCloudPose(m_cloud_id, transform);
}

template<typename PointT>
void PointCloudViewer<PointT>::translatePointCloud()
{
    std::cout << "translatePointCloud()" << std::endl;

    float tx = m_translation_x_slider->value() / 10.0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tx, 0.0, 0.0;
    m_viewer->updatePointCloudPose(m_cloud_id, transform);
}

template <typename PointT>
void PointCloudViewer<PointT>::terminateProgram()
{
    std::cout << "terminateProgram()" << std::endl;

    m_running = false;
    qApp->quit();
}

template <typename PointT>
void PointCloudViewer<PointT>::init()
{
    std::cout << "init() BEGIN" << std::endl;

    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               1,
                                               m_cloud_id);
    m_viewer->addCoordinateSystem(1.0);
    m_viewer->initCameraParameters();

    // Start visualization thread
    m_vis_thread = std::thread(&PointCloudViewer::visualizationThread, this);

    /* ----- Set up QT stuff ----- */

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);

    m_rotation_spin_box = new QDoubleSpinBox;
    m_rotation_spin_box->setRange(-360.0, 360.0);
    m_rotation_spin_box->setSingleStep(1.0);
    m_rotation_spin_box->setValue(0.0);
    layout->addWidget(m_rotation_spin_box);

    QPushButton *rotateButton = new QPushButton("Rotate");
    layout->addWidget(rotateButton);
    connect(rotateButton, &QPushButton::clicked, this, &PointCloudViewer::rotatePointCloud);

    m_translation_x_slider = new QSlider(Qt::Horizontal);
    m_translation_x_slider->setRange(-100, 100);
    m_translation_x_slider->setValue(0);
    layout->addWidget(m_translation_x_slider);

    QPushButton *translateButton = new QPushButton("Translate");
    layout->addWidget(translateButton);
    connect(translateButton, &QPushButton::clicked, this, &PointCloudViewer::translatePointCloud);

    // Add the terminate button
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
