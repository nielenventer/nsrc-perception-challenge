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

#define TEST_DATA_RELATIVE_PATH     "/../../test_data/stereo_point_clouds/stereo_points_1.pcd"
#define POINTS_N                    10

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
 * Get some random points from the given point cloud.
 * @param cloud         The cloud to get points from
 * @param out_points    Array to be populated with points
 * @param size          Number of points to get
 */
void get_random_points(pcloud_xyz::Ptr cloud, p_xyz *out_points, uint size) {
    // Get rid of nans
    pcloud_xyz::Ptr cloud_no_nan(new pcloud_xyz);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

    // Get random points and populate array
    std::srand(std::time(0));

    for (int i = 0; i < POINTS_N; i++) {
        uint r = (std::rand() % (int)(cloud_no_nan->points.size() + 1));
        out_points[i] = cloud_no_nan->points[r];
    }
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

    // Translation
    transform.translation() << tx, ty, tz;

    // Rotation (radians)
    transform.rotate(Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));

    return transform;
}

/**
 * Transform a 3d point with the given Affine transformation.
 * @param point         The point to transform
 * @param transform     The Affine transformation
 * @return  The transformed point
 */
p_xyz transform_point(p_xyz point, Eigen::Affine3f transform) {
    // Apply transform to point
    Eigen::Vector4f v(point.x, point.y, point.z, 1.);
    Eigen::Vector4f vt = transform.matrix() * v;

    // Convert point to correct type
    p_xyz transformed_point(vt[0], vt[1], vt[2]);
    return transformed_point;
}

/**
 * Build a point cloud with the given points.
 * @param points    Array of points
 * @param size      Number of points in array
 * @return  The constructed point cloud
 */
pcloud_xyz::Ptr cloud_with_points(pcl::PointXYZ *points, int size) {
    pcloud_xyz::Ptr cloud(new pcloud_xyz);

    for (int i = 0; i < size; i++) {
        cloud->push_back(points[i]);
    }
    return cloud;
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
    float tx = 2.5;
    float ty = 0.0;
    float tz = 0.0;
    float rx = 0.0;
    float ry = 0.0;
    float rz = M_PI/4.;

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
    // Load test data
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH;
    pcloud_xyz::Ptr source_cloud(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *source_cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

    // Get random points from cloud
    p_xyz source_points[POINTS_N];
    get_random_points(source_cloud, source_points, POINTS_N);

    /*******************/
    /* Transformations */
    /*******************/
    // Apply transformations to cloud
    Eigen::Affine3f transform = create_transform_matrix(tx, ty, tz,
                                                        rx, ry, rz);

    std::cout << "Applying transform: " << std::endl << transform.matrix() << std::endl;
    pcloud_xyz::Ptr transformed_cloud(new pcloud_xyz);
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

    // Apply transformations to individual points
    p_xyz transformed_points[POINTS_N];
    for (int i = 0; i < POINTS_N; i++) {
        transformed_points[i] = transform_point(source_points[i], transform);
    }

    /*********************/
    /* Visualise results */
    /*********************/
    pcl::visualization::PCLVisualizer viewer("Transform Visualisation");
    viewer.addCoordinateSystem(1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    vis_cloud(&viewer, source_cloud, "original_cloud", 80, 80, 80, 1);
    vis_cloud(&viewer, transformed_cloud, "transformed_cloud", 5, 5, 120, 1);

    pcloud_xyz::Ptr source_points_cloud      = cloud_with_points(source_points, POINTS_N);
    pcloud_xyz::Ptr transformed_points_cloud = cloud_with_points(transformed_points, POINTS_N);
    vis_cloud(&viewer, source_points_cloud, "original_points", 250, 250, 250, 5);
    vis_cloud(&viewer, transformed_points_cloud, "transformed_points", 50, 50, 250, 5);

    // Display
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}