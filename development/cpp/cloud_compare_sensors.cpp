/* pointclouds.org/documentation/tutorials/matrix_transform.php */

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <limits.h>
#include <unistd.h>

#define TEST_DATA_RELATIVE_PATH_S   "/../../test_data/stereo_point_clouds/stereo_points_1.pcd"
#define TEST_DATA_RELATIVE_PATH_L   "/../../test_data/lidar_point_clouds/lidar_points_1.pcd"

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
 * Create an Affine transformation for 3d points (rotation and translation).
 * @param tx,ty,tz      Translation x, y and z
 * @param rx,ry,rz      Rotation around x, y and z axes
 * @return  3d Affine transformation 
 */
Eigen::Affine3f create_transform_matrix(float tx, float ty, float tz,
                                        float rx, float ry, float rz) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Rotation (radians)
    transform.rotate(Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));

    // Translation
    transform.translation() << tx, ty, tz;

    return transform;
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
    float tx = 0.035;
    float ty = -0.002;
    float tz = 0.0;
    float rx = -2.356;
    float ry = -1.571;
    float rz = -2.356;

    if (argc == 7) {
        // Try convert the args
        tx = atof(argv[1]);
        ty = atof(argv[2]);
        tz = atof(argv[3]);
        rx = atof(argv[4]);
        ry = atof(argv[5]);
        rz = atof(argv[6]);
    }
    else if (argc != 1) {
        std::cout << "Expected 0 or 6 params." << std::endl;
        return -1;
    }

    /************/
    /* Get data */
    /************/
    // Load test data - stereo
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH_S;
    pcloud_xyz::Ptr stereo_cloud(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *stereo_cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

    // Load test data - lidar
    test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH_L;
    pcloud_xyz::Ptr lidar_cloud(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *lidar_cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }


    /*******************/
    /* Transformations */
    /*******************/
    // Apply transformations to stereo
    Eigen::Matrix4f st_transform = Eigen::Matrix4f::Identity();
    st_transform(0,0) = -3.67321e-06; st_transform(0,1) = -3.67321e-06; st_transform(0,2) = 1;            st_transform(0,3) = 0;
    st_transform(1,0) = -1;           st_transform(1,1) = 1.34925e-11;  st_transform(1,2) = -3.67321e-06; st_transform(1,3) = 0.035;
    st_transform(2,0) = 0;            st_transform(2,1) = -1;           st_transform(2,2) = -3.67321e-06; st_transform(2,3) = -0.002;

    std::cout << "Applying stereo transform: " << std::endl << st_transform.matrix() << std::endl;
    pcloud_xyz::Ptr transformed_stereo_cloud(new pcloud_xyz);
    pcl::transformPointCloud(*stereo_cloud, *transformed_stereo_cloud, st_transform);

    // Apply transformations to lidar
    Eigen::Affine3f li_transform = create_transform_matrix(0., 0., 0.,
                                                           0., 0., 0.);

    std::cout << "Applying lidar transform: " << std::endl << li_transform.matrix() << std::endl;
    pcloud_xyz::Ptr transformed_lidar_cloud(new pcloud_xyz);
    pcl::transformPointCloud(*lidar_cloud, *transformed_lidar_cloud, li_transform);


    /*********************/
    /* Visualise results */
    /*********************/
    pcl::visualization::PCLVisualizer viewer("Transform Visualisation");
    viewer.addCoordinateSystem(1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    vis_cloud(&viewer, transformed_lidar_cloud, "lidar_cloud", 80, 80, 80, 1);
    vis_cloud(&viewer, transformed_stereo_cloud, "stereo_cloud", 150, 150, 220, 1);

    // Display
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}