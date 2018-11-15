#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Point.h>
#include <pcl/kdtree/kdtree_flann.h>

#define POINTS_N                    50

// convenience
typedef pcl::PointXYZ   p_xyz;
typedef pcl::PointCloud<pcl::PointXYZ>  pcloud_xyz;

static p_xyz pixel_location;
static bool location_received = false;
static ros::Publisher pub;
static Eigen::Affine3f transform;


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
 * Uses kd tree based nearest neighbour search to find the nearest
 * points in the given cloud to the given location. Returns the average 
 * of the found points.
 * @param cloud             Cloud to search
 * @param point_to_find     Point to find :P
 * @return  The averaged point
 */
p_xyz closest_point_in_cloud(pcloud_xyz::Ptr cloud, p_xyz point_to_find) {
    pcl::KdTreeFLANN<p_xyz> kdtree;
    kdtree.setInputCloud(cloud);
    p_xyz smoothed_point(0.0, 0.0, 0.0);

    std::vector<int> pointIdxNKNSearch(POINTS_N);
    std::vector<float> pointNKNSquaredDistance(POINTS_N);

    if (kdtree.nearestKSearch(point_to_find, POINTS_N, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
            p_xyz found_point = cloud->points[ pointIdxNKNSearch[i] ];

            smoothed_point.x += found_point.x;
            smoothed_point.y += found_point.y;
            smoothed_point.z += found_point.z;
        }
    }
    smoothed_point.x /= POINTS_N;
    smoothed_point.y /= POINTS_N;
    smoothed_point.z /= POINTS_N;

    return smoothed_point;
}

/**
 * Callback for when cloud data arrives
 */
void cloud_arrived(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // process
    if (location_received) {
        // Find best stimate of point in lidar cloud
        pcloud_xyz::Ptr cloud_ptr(new pcloud_xyz);
        pcl::fromROSMsg(*msg, *cloud_ptr); 
        p_xyz final_point = closest_point_in_cloud(cloud_ptr, pixel_location);

        // publish
        geometry_msgs::Point out_msg;
        out_msg.x = final_point.x; 
        out_msg.y = final_point.y;
        out_msg.z = final_point.z;
        pub.publish(out_msg);
    }
}

/**
 * Callback for when LED location (x,y) arrives
 */
void pixel_loc_arrived(const geometry_msgs::Point::ConstPtr &msg) {
    // Trsanform and save the pixel
    location_received = true;

    p_xyz location_received;
    location_received.x = (*msg).x;
    location_received.y = (*msg).y;
    location_received.z = (*msg).z;

    // Transform to lidar frame
    pixel_location = transform_point(pixel_location, transform);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "stereo_point_locator");
    ros::NodeHandle nh;

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/multisense/camera/points2", 
                                                            1, cloud_arrived);
    ros::Subscriber sub_coord = nh.subscribe<geometry_msgs::Point>("/stereo_coordinate", 
                                                                    1, pixel_loc_arrived);
    pub = nh.advertise<geometry_msgs::Point>("/lidar_coordinate", 1);

    // Transform matrix from stereo to lidar frame
    // (TODO: find appropriate transform)
    transform = create_transform_matrix(0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0);

    ros::spin();
}