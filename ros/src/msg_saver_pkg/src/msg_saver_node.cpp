/* Standard Libraries */
#include <string>
#include <mutex>  // Include for mutex
#include <fstream> // Include for file operations
#include <iomanip> // Include for setw and setfill
#include <sys/stat.h> // Include for directory check
#include <algorithm> // Include for std::find_if

/* ROS Includes */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_srvs/Empty.h"
#include "cv_bridge/cv_bridge.h"

/* PCL Headers */
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"

/* OpenCV Headers */
#include "opencv2/opencv.hpp"

// Global variables to store the latest messages
sensor_msgs::PointCloud2::ConstPtr g_latest_pointcloud1;
sensor_msgs::PointCloud2::ConstPtr g_latest_pointcloud2;
sensor_msgs::Image::ConstPtr g_latest_image;
sensor_msgs::CameraInfo::ConstPtr g_latest_camera_info;

ros::Subscriber g_camera_info_sub;

// Topic names
std::string g_pointcloud1_topic;
std::string g_pointcloud2_topic;
std::string g_image_topic;
std::string g_camera_info_topic;

// Separate mutexes for each message type
std::mutex g_mtx_pointcloud1;
std::mutex g_mtx_pointcloud2;
std::mutex g_mtx_image;
std::mutex g_mtx_camera_info;

// Directory to save files
std::string g_save_directory;

// Counter for saved files
int g_save_counter = 0;

// Function to check if directory exists
bool directoryExists(const std::string& dir)
{
    struct stat info;
    if (stat(dir.c_str(), &info) != 0)
    {
        return false;
    }
    else if (info.st_mode & S_IFDIR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Function to sanitize topic names
std::string sanitizeTopicName(const std::string& topic) {
    std::string sanitized = topic;

    // Check if the first character is a special character
    if (!sanitized.empty() && !std::isalnum(sanitized[0]) && sanitized[0] != '_') {
        sanitized.erase(sanitized.begin()); // Remove the first character
    }

    // Replace the remaining special characters with underscores
    std::replace_if(sanitized.begin(), sanitized.end(),
        [](char c) { return !std::isalnum(c) && c != '_'; }, '_');

    return sanitized;
}

// Callback functions
void pointcloud1Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    g_mtx_pointcloud1.lock();
    g_latest_pointcloud1 = msg;
    g_mtx_pointcloud1.unlock();
}

void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    g_mtx_pointcloud2.lock();
    g_latest_pointcloud2 = msg;
    g_mtx_pointcloud2.unlock();
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    g_mtx_image.lock();
    g_latest_image = msg;
    g_mtx_image.unlock();
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    g_mtx_camera_info.lock();
    g_latest_camera_info = msg;
    g_mtx_camera_info.unlock();
}

// Service callback function
bool saveData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // Format the counter as a three-digit string
    std::ostringstream counter_str;
    counter_str << std::setw(3) << std::setfill('0') << g_save_counter;

    // Create file paths with counter as postfix
    std::string pointcloud1_path    = g_save_directory + "/" + sanitizeTopicName(g_pointcloud1_topic) +
                                      "_" + counter_str.str() + ".pcd";
    std::string pointcloud2_path    = g_save_directory + "/" + sanitizeTopicName(g_pointcloud2_topic) +
                                      "_" + counter_str.str() + ".pcd";
    std::string image_path          = g_save_directory + "/" + sanitizeTopicName(g_image_topic)       +
                                      "_" + counter_str.str() + ".png";
    std::string camera_info_path    = g_save_directory + "/" + sanitizeTopicName(g_camera_info_topic) +
                                      "_" + counter_str.str() + ".txt";

    g_mtx_pointcloud1.lock();
    if (g_latest_pointcloud1)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*g_latest_pointcloud1, cloud);
        pcl::io::savePCDFileASCII(pointcloud1_path, cloud);
        ROS_INFO("Saved %s", pointcloud1_path.c_str());
    }
    g_mtx_pointcloud1.unlock();
    
    g_mtx_pointcloud2.lock();
    if (g_latest_pointcloud2)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*g_latest_pointcloud2, cloud);
        pcl::io::savePCDFileASCII(pointcloud2_path, cloud);
        ROS_INFO("Saved %s", pointcloud2_path.c_str());
    }
    g_mtx_pointcloud2.unlock();
    
    g_mtx_image.lock();
    if (g_latest_image)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(g_latest_image, "bgr8");
        cv::imwrite(image_path, cv_ptr->image);
        ROS_INFO("Saved %s", image_path.c_str());
    }
    g_mtx_image.unlock();
    
    g_mtx_camera_info.lock();
    if (g_latest_camera_info)
    {
        std::ofstream file(camera_info_path);
        file << "Width: " << g_latest_camera_info->width << "\n";
        file << "Height: " << g_latest_camera_info->height << "\n";
        
        file << "Camera matrix: " << "[";
        for (auto it = g_latest_camera_info->K.cbegin();
             it != g_latest_camera_info->K.cend();
             ++it)
        {
            file << *it << " ";
        }
        file << "]" << "\n";

        file << "Distortion coeffs: " << "[";
        for (auto it = g_latest_camera_info->D.cbegin();
             it != g_latest_camera_info->D.cend();
             ++it)
        {
            file << *it << " ";
        }
        file << "]" << "\n";

        // You can add more camera info details here
        file.close();
        ROS_INFO("Saved %s", camera_info_path.c_str());

        // Shutdown the subscriber since it is useless to save its data multiple times
        g_camera_info_sub.shutdown();
        // Reset the pointer message
        g_latest_camera_info = nullptr;
    }
    g_mtx_camera_info.unlock();

    // Increment the save counter
    g_save_counter++;
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_saver_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // Collect parameters from ROS param server
    p_nh.param("pointcloud1_topic", g_pointcloud1_topic, std::string(""));
    p_nh.param("pointcloud2_topic", g_pointcloud2_topic, std::string(""));
    p_nh.param("image_topic", g_image_topic, std::string(""));
    p_nh.param("camera_info_topic", g_camera_info_topic, std::string(""));

    // Check if the parameter exists
    if (!p_nh.hasParam("save_directory"))
    {
        ROS_ERROR("Parameter 'save_directory' not found in the parameter server. Exiting...");
        return 1;
    }

    p_nh.param("save_directory", g_save_directory, std::string("")); // Get the save directory parameter

    // Check if the save directory exists
    if (!directoryExists(g_save_directory))
    {
        ROS_ERROR("The provided save directory does not exist: %s", g_save_directory.c_str());
        return 1; // Exit the node
    }

    // Subscribers
    ros::Subscriber pointcloud1_sub, pointcloud2_sub, image_sub;

    if (!g_pointcloud1_topic.empty())
    {
        pointcloud1_sub = nh.subscribe(g_pointcloud1_topic, 10, pointcloud1Callback);
    }
    if (!g_pointcloud2_topic.empty())
    {
        pointcloud2_sub = nh.subscribe(g_pointcloud2_topic, 10, pointcloud2Callback);
    }
    if (!g_image_topic.empty())
    {
        image_sub = nh.subscribe(g_image_topic, 10, imageCallback);
    }
    if (!g_camera_info_topic.empty())
    {
        g_camera_info_sub = nh.subscribe(g_camera_info_topic, 10, cameraInfoCallback);
    }

    // Service to save data
    ros::ServiceServer service = nh.advertiseService("save_data", saveData);

    ROS_INFO("Message saver node started.");
    ros::spin();

    return 0;
}
