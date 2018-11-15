#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <limits.h>
#include <unistd.h>

#define POINT_VALID(p)  (pcl_isfinite((p).x))

#define TEST_DATA_RELATIVE_PATH     "/../../test_data/stereo_point_clouds/stereo_points_1.pcd"
#define SMOOTH_POINTS_XY_N          5
#define SMOOTH_POINTS_Z_N           10

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
 * Remap/linearly interpolate the given point from the input range to the output range.
 * @param inp       The point to remap
 * @param inp_min   The input range start
 * @param inp_max   The input range end
 * @param out_min   The output range start
 * @param out_max   The output range end
 * @return  The remapped input
 */
inline float interp_lin(float inp, float inp_min, float inp_max, float out_min, float out_max) {
    return (out_max - out_min) * ((inp - inp_min) / (inp_max - inp_min)) + out_min;
}

/**
 * Search for nearest points in given direction and return average position and index.
 * @param real              Position (real) coordinate to be updated
 * @param index             Index to be updated
 * @param vertical          Vertical search (else horizontal)
 * @param delta             Index search delta
 * @param cloud             Cloud containing object
 * @param constr_cloud      Cloud for placing construction points (useful for debugging)
 * @param xi                X index to search around
 * @param yi                Y index to search around
 */
void search_around_point(float *real, float *real_perp, float *index, 
                         bool vertical, int delta,
                         pcloud_xyz::Ptr cloud, pcloud_xyz::Ptr constr_cloud, 
                         uint x_init, uint y_init) {
    int sx = x_init; // search index
    int sy = y_init;

    int dx = vertical ? 0 : delta;
    int dy = vertical ? delta : 0;

    float ave_real = 0.0;
    float ave_perp = 0.0;
    float ave_index = 0.0;

    uint pts_found = 0;

    while (pts_found < SMOOTH_POINTS_XY_N) {
        // Update index
        sx += dx; sy += dy;
        if ((sx < 0) || (sx >= cloud->width) ||
            (sy < 0) || (sy >= cloud->height)) {
            break; // Out of bounds
        }

        // Check if valid point
        p_xyz p = (*cloud)(sx, sy);
        if (POINT_VALID(p)) {
            constr_cloud->push_back(p);
            pts_found++;

            ave_real += vertical ? p.y : p.x;
            ave_perp += vertical ? p.x : p.y;
            ave_index += vertical ? sy : sx;
        }
    }

    // Average valid points
    if (pts_found) {
        *real  = ave_real / pts_found;
        *real_perp = ave_perp / pts_found;
        *index = ave_index / pts_found;
    }
    
}

/**
 * Linearly interpolate real world position using points along x or y axis.
 * @param cloud             Cloud containing object
 * @param constr_cloud      Cloud for placing construction points (useful for debugging)
 * @param xi                X index to search around
 * @param yi                Y index to search around
 * @param vertical          Interpolate vertically (else horizontally)
 * @return      Interpolated position
 */
p_xyz interpolate_along_axis(pcloud_xyz::Ptr cloud, pcloud_xyz::Ptr constr_cloud, 
                             uint xi, uint yi, bool vertical) {
    // Look high
    float real_high = 0;
    float real_perp_high = 0;
    float index_high = 0;
    search_around_point(&real_high, &real_perp_high, &index_high, 
                        vertical, 1, cloud, constr_cloud, xi, yi);

    // Look low
    float real_low = 0;
    float real_perp_low = 0;
    float index_low = 0;
    search_around_point(&real_low, &real_perp_low, &index_low, 
                        vertical, -1, cloud, constr_cloud, xi, yi);

    // Interpolate
    p_xyz out;
    float interp  = 0;
    float intperp = 0; // perpindicular interpolate

    if (index_low && index_high) {
        interp  = interp_lin((vertical ? yi: xi), index_low, index_high, real_low, real_high);
        intperp = (real_perp_low + real_perp_high) / 2.;
    }
    else {
        if (index_low)  intperp = real_perp_low;
        if (index_high) intperp = real_perp_high;
    }

    out.y = vertical ? interp : intperp;
    out.x = vertical ? intperp : interp;
    return out;
}

/**
 * Linearly interpolate real world position using points along x and y axis.
 * @param point             Point to put interpolated x and y coordinates
 * @param xi                X index to search around
 * @param yi                Y index to search around
 * @param cloud             Cloud containing object
 * @param constr_cloud      Cloud for placing construction points (useful for debugging)
 */
void interpolate_to_index(p_xyz *point, uint xi, uint yi, 
                           pcloud_xyz::Ptr cloud, pcloud_xyz::Ptr constr_cloud) {
    p_xyz interp_x = interpolate_along_axis(cloud, constr_cloud, xi, yi, false);
    p_xyz interp_y = interpolate_along_axis(cloud, constr_cloud, xi, yi, true);

    (*point).x = interp_x.x ?: interp_y.x;
    (*point).y = interp_y.y ?: interp_x.y;
}

/**
 * Estimate the depth of the given pixel by finding closest coordinates and averaging their
 * depthds. Closest points are found by spiraling outwards.
 * @param cloud             Cloud containing object to search
 * @param constr_cloud      Cloud for placing construction points (useful for debugging)
 * @param xi                X index to search around
 * @param yi                Y index to search around
 * @return      Estimated depth
 */
float est_depth_for_index(pcloud_xyz::Ptr cloud, pcloud_xyz::Ptr constr_cloud, uint xi, uint yi) {
    int x = 0;
    int y = 0;
    float z = 0.0;
    uint pfound = 0;

    while (pfound < SMOOTH_POINTS_Z_N) {
        uint px = xi + x;
        uint py = yi + y;
        p_xyz p = (*cloud)(px, py);

        // Check if point is valid
        if (POINT_VALID(p)) {
            constr_cloud->push_back(p);
            z += p.z;
            pfound++;
        }

        // Spiral out (keep going)
        if (abs(x) <= abs(y) && (x != y || x >= 0)) {
            x += ((y >= 0) ? 1 : -1);
        }
        else {
            y += ((x >= 0) ? -1 : 1);
        }

        if ((px < 0) || (px >= cloud->width) ||
            (py < 0) || (py >= cloud->height)) {
            break; // Out of bounds
        }
    }

    // Return average z of valid points
    return (pfound) ? z / pfound : 0.;
}

/**
 * Adds a point cloud to the given viewer, in the given colour.
 * @param viewer   The viewer to add the cloud to
 * @param cloud    The point cloud
 * @param name     A name for the cloud
 * @param r,g,b    RGB values to draw the points with
 * @param p_size   Size to draw the points
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
    // Default point
    uint x = 470;
    uint y = 220;

    if (argc == 3) {
        // Try convert the args
        x = atoi(argv[1]);
        y = atoi(argv[2]);
    }
    else if (argc != 1) {
        std::cout << "Expected 0 or 2 params." << std::endl;
        return -1;
    }

    /************/
    /* Get data */
    /************/
    std::string test_data_path = getexepath() + TEST_DATA_RELATIVE_PATH;
    pcloud_xyz::Ptr source_cloud(new pcloud_xyz);
    if (pcl::io::loadPCDFile(test_data_path, *source_cloud) < 0) {
        std::cout << "Couldn't load: " << test_data_path << std::endl;
        return -1;
    }

    /********************/
    /* Find coordinates */
    /********************/
    pcloud_xyz::Ptr cross_pcloud(new pcloud_xyz);
    pcloud_xyz::Ptr close_pcloud(new pcloud_xyz);
    p_xyz target_point = (*source_cloud)(x, y);
    
    if (!POINT_VALID(target_point)) {
        // If the point doesn't exist, interpolate
        interpolate_to_index(&target_point, x, y, source_cloud, cross_pcloud);
        target_point.z = est_depth_for_index(source_cloud, close_pcloud, x, y);
    }
    
    /*********************/
    /* Visualise results */
    /*********************/
    pcl::visualization::PCLVisualizer viewer("Transform Visualisation");
    viewer.addCoordinateSystem(1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    // Cloud with construction points
    vis_cloud(&viewer, source_cloud, "original_cloud", 80, 80, 80, 1);
    vis_cloud(&viewer, cross_pcloud, "construction_points_xy", 155, 155, 255, 3);
    vis_cloud(&viewer, close_pcloud, "construction_points_z", 155, 255, 155, 3);

    // Final (interpolated) point
    pcloud_xyz::Ptr final_point_cloud(new pcloud_xyz);
    final_point_cloud->push_back(target_point);
    vis_cloud(&viewer, final_point_cloud, "final_point", 255, 255, 255, 5);
    std::cout << "Final point: " << target_point << std::endl;

    // Display
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}