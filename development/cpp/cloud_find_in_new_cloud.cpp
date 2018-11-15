#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <string>
#include <limits.h>
#include <unistd.h>

#define TEST_DATA_RELATIVE_PATH     "/../../test_data/stereo_point_clouds/stereo_points_1.pcd"
#define POINTS_N                    50

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

/**
 * Adds a point to the given viewer, in the given colour.
 * @param viewer   The viewer to add the cloud to
 * @param point    The point
 * @param name     A name for the cloud
 * @param r,g,b    RGB values to draw the points with
 * @param p_size   The size to draw the points
 */
void vis_point(pcl::visualization::PCLVisualizer *viewer, 
               p_xyz point,
               std::string name, 
               uint r, uint g, uint b, uint p_size) {
    pcloud_xyz::Ptr cloud(new pcloud_xyz);
    cloud->push_back(point);
    vis_cloud(viewer, cloud, name, r, g, b, p_size);
}
   
    
int main(int argc, char **argv) {
    float px = -0.522822;
    float py = -0.243983;
    float pz =  2.83589;

    if (argc == 4) {
        // Try convert the args
        px = atof(argv[1]);
        py = atof(argv[2]);
        pz = atof(argv[3]);
    }
    else if (argc != 1) {
        std::cout << "Expected 0 or 3 params." << std::endl;
        return -1;
    }

    /************/
    /* Get data */
    /************/
    // Load test data
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH;
    pcloud_xyz::Ptr cloud_to_search(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *cloud_to_search) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

    // Point to find
    p_xyz point_to_find(px, py, pz);

    // Apply transformation to point (TODO: find appropriate transform)
    Eigen::Affine3f transform = create_transform_matrix(0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0);

    std::cout << "Applying transform: " << std::endl << transform.matrix() << std::endl;
    p_xyz transformed_point = transform_point(point_to_find, transform);

    /********************/
    /* Search the cloud */
    /********************/
    pcl::KdTreeFLANN<p_xyz> kdtree;
    kdtree.setInputCloud(cloud_to_search);
    pcloud_xyz::Ptr found_points(new pcloud_xyz);
    p_xyz smoothed_point(0.0, 0.0, 0.0);

    std::vector<int> pointIdxNKNSearch(POINTS_N);
    std::vector<float> pointNKNSquaredDistance(POINTS_N);

    if (kdtree.nearestKSearch(point_to_find, POINTS_N, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
            p_xyz found_point = cloud_to_search->points[ pointIdxNKNSearch[i] ];
            // float dist = pointNKNSquaredDistance[i];
            found_points->push_back(found_point);

            smoothed_point.x += found_point.x;
            smoothed_point.y += found_point.y;
            smoothed_point.z += found_point.z;
        }
    }
    smoothed_point.x /= POINTS_N;
    smoothed_point.y /= POINTS_N;
    smoothed_point.z /= POINTS_N;

    /*********************/
    /* Visualise results */
    /*********************/
    pcl::visualization::PCLVisualizer viewer("Transform Visualisation");
    viewer.addCoordinateSystem(1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    vis_cloud(&viewer, cloud_to_search, "cloud_to_search", 80, 80, 80, 1);
    vis_cloud(&viewer, found_points, "points_found", 120, 120, 120, 2);
    vis_point(&viewer, point_to_find, "point_to_find", 220, 220, 220, 3);
    vis_point(&viewer, smoothed_point, "smoothed_point", 255, 220, 220, 5);

    // Display
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}