#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <limits.h>
#include <unistd.h>

#define TEST_DATA_RELATIVE_PATH     "/../../../../development/test_data"
// TODO: Get lidar data
#define TEST_DATA_CLOUD_NAME        "/stereo_point_clouds/stereo_points_1.pcd"


/**
 * Get the path to the executable running this code, useful for finding other files.
 * @return  The path to the the executable
 */
std::string getexepath(void) {
    // Path to executable
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string path = std::string(result, (count > 0) ? count : 0);

    // Directory of executable
    return path.substr(0, path.find_last_of("\\/"));
}

 
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "lidar_points_dummy"); 
    ros::NodeHandle nh;
 
    // Create a ROS publisher for the output point cloud
    // TODOL Get right topic name
    ros::Publisher pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("multisense/lidar/points2", 1);

    // Load point cloud from test data
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH + TEST_DATA_CLOUD_NAME;

    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(test_data_path, *test_cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

 
    ros::Rate r(1); 
    while (ros::ok()) {
        // Publish the message .
        // Note that it is automatically converted to sensor_msgs::PointCloud2
        pub.publish(*test_cloud);

        r.sleep(); 
    }
}