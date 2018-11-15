#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <limits.h>
#include <unistd.h>

#define TEST_DATA_RELATIVE_PATH     "/../../test_data/stereo_point_clouds/stereo_points_1.pcd"

// convenience
typedef pcl::PointXYZ   p_xyz;
typedef pcl::PointCloud<pcl::PointXYZ>  pcloud_xyz; 


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

/**
 * Adds a point cloud to the given viewer, in the given colour.
 * @param viewer   The viewer to add the cloud to
 * @param cloud    The point cloud
 * @param name     A name for the cloud
 * @param r,g,b    RGB values to draw the points with
 * @param p_size   The size to draw the points
 */
void vis_cloud(pcl::visualization::PCLVisualizer *viewer, 
               pcloud_xyz::Ptr cloud,
               std::string name, 
               uint r, uint g, uint b, uint p_size) {
    pcl::visualization::PointCloudColorHandlerCustom<p_xyz> color_handler(cloud, r, g, b);
    (*viewer).addPointCloud(cloud, color_handler, name);
    (*viewer).setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, p_size, name);
}
    
    
int main(int argc, char **argv) {
    // Load data
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH;
    pcloud_xyz::Ptr cloud(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

    // Visualise result
    pcl::visualization::PCLVisualizer viewer("Transform Visualisation");
    viewer.addCoordinateSystem(1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    // Colour
    // pcl::visualization::PointCloudColorHandlerRGBField<p_xyz> rgb(cloud);
    // viewer.addPointCloud<p_xyz>(cloud, rgb, "cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // Gray
    vis_cloud(&viewer, cloud, "cloud", 80, 80, 80, 1);

    // Display
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}