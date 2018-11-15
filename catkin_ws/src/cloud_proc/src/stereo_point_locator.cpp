#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cam_proc/pixel_loc.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#define POINT_VALID(p)  (pcl_isfinite((p).x))

#define SMOOTH_POINTS_XY_N          5
#define SMOOTH_POINTS_Z_N           10

// convenience
typedef pcl::PointXYZ   p_xyz;
typedef pcl::PointCloud<pcl::PointXYZ>  pcloud_xyz;

static cam_proc::pixel_loc pixel_location;
static bool location_received = false;
static ros::Publisher pub; 


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
 * @param xi                X index to search around
 * @param yi                Y index to search around
 */
void search_around_point(float *real, float *real_perp, float *index, 
                         bool vertical, int delta,
                         pcloud_xyz::Ptr cloud, uint x_init, uint y_init) {
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
 * @param xi                X index to search around
 * @param yi                Y index to search around
 * @param vertical          Interpolate vertically (else horizontally)
 * @return      Interpolated position
 */
p_xyz interpolate_along_axis(pcloud_xyz::Ptr cloud, uint xi, uint yi, bool vertical) {
    // Look high
    float real_high = 0;
    float real_perp_high = 0;
    float index_high = 0;
    search_around_point(&real_high, &real_perp_high, &index_high, 
                        vertical, 1, cloud, xi, yi);

    // Look low
    float real_low = 0;
    float real_perp_low = 0;
    float index_low = 0;
    search_around_point(&real_low, &real_perp_low, &index_low, 
                        vertical, -1, cloud, xi, yi);

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
 */
void interpolate_to_index(p_xyz *point, uint xi, uint yi, 
                           pcloud_xyz::Ptr cloud) {
    p_xyz interp_x = interpolate_along_axis(cloud, xi, yi, false);
    p_xyz interp_y = interpolate_along_axis(cloud, xi, yi, true);

    (*point).x = interp_x.x ?: interp_y.x;
    (*point).y = interp_y.y ?: interp_x.y;
}

/**
 * Estimate the depth of the given pixel by finding closest coordinates and averaging their
 * depthds. Closest points are found by spiraling outwards.
 * @param cloud             Cloud containing object to search
 * @param xi                X index to search around
 * @param yi                Y index to search around
 * @return      Estimated depth
 */
float est_depth_for_index(pcloud_xyz::Ptr cloud, uint xi, uint yi) {
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
 * Callback for when cloud data arrives
 */
void cloud_arrived(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // process
    if (location_received) {
        if (pixel_location.r || pixel_location.b || pixel_location.g) {
            // Convert cloud
            pcloud_xyz::Ptr cloud_ptr(new pcloud_xyz);
            pcl::fromROSMsg(*msg, *cloud_ptr);

            uint x = pixel_location.x;
            uint y = pixel_location.y;
            p_xyz target_point = (*cloud_ptr)(x, y);
        
            if (!POINT_VALID(target_point)) {
                // If the point doesn't exist, interpolate
                interpolate_to_index(&target_point, x, y, cloud_ptr);
                target_point.z = est_depth_for_index(cloud_ptr, x, y);
            }

            // Publish point
            geometry_msgs::Point out_msg;
            out_msg.x = target_point.x; 
            out_msg.y = target_point.y;
            out_msg.z = target_point.z;
            pub.publish(out_msg);
        }
    }
}

/**
 * Callback for when LED location (x,y) arrives
 */
void pixel_loc_arrived(const cam_proc::pixel_loc::ConstPtr &msg) {
    // Save the pixel
    location_received = true;
    pixel_location = *msg;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "stereo_point_locator");
    ros::NodeHandle nh;

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/multisense/camera/points2", 
                                                            1, cloud_arrived);
    ros::Subscriber sub_pixel = nh.subscribe<cam_proc::pixel_loc>("/LED_im_loc", 
                                                                    1, pixel_loc_arrived);
    pub = nh.advertise<geometry_msgs::Point>("/stereo_coordinate", 1);

    ros::spin();
}