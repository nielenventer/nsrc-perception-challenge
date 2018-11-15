#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cam_proc/pixel_loc.h>
#include <geometry_msgs/Point.h>
#include <srcsim/Console.h>

#define NEW_LED_DELTA           5
#define POINT_SMOOTH_FACTOR     0.2
#define EXP_SMOOTH(inp_prev, inp, alpha)     ((alpha)*((inp) - (inp_prev)) + (inp_prev))


static ros::Publisher pub;
static float point_x = 0.0;
static float point_y = 0.0;
static float point_z = 0.0;

static bool new_point_arrived = true;
static float point_r = 0.0;
static float point_g = 0.0;
static float point_b = 0.0;
static uint point_LED_x = 0;
static uint point_LED_y = 0;

static Eigen::Matrix4f st_transform = Eigen::Matrix4f::Identity();


/**
 * Transform a 3d point with the given transformation matrix.
 * @param point         The point to transform
 * @param transform     The transformation matrix
 * @return  The transformed point
 */
geometry_msgs::Point transform_point(geometry_msgs::Point point, Eigen::Matrix4f transform) {
    // Apply transform to point
    Eigen::Vector4f v(point.x, point.y, point.z, 1.);
    Eigen::Vector4f vt = transform * v;

    // Convert point to correct type
    geometry_msgs::Point transformed_point;
    transformed_point.x = vt[0];
    transformed_point.y = vt[1];
    transformed_point.z = vt[2];

    return transformed_point;
}

/**
 * Callback for when LED location (x,y) arrives
 */
void pixel_loc_arrived(const cam_proc::pixel_loc::ConstPtr &msg) {
    // Save the pixel colour
    point_r = (*msg).r;
    point_g = (*msg).g;
    point_b = (*msg).b;

    // Check if it's a new point
    if ((abs(point_LED_x - (*msg).x) > NEW_LED_DELTA) || 
        (abs(point_LED_y - (*msg).y) > NEW_LED_DELTA)) {
        ROS_INFO_STREAM("New target, resetting filters.");
        new_point_arrived = true;
        point_LED_x = (*msg).x;
        point_LED_y = (*msg).y;
    }
}

/**
 * Callback for when stereo point arrives
 */
void st_point_arrived(const geometry_msgs::Point::ConstPtr& msg) {
    // Get points
    geometry_msgs::Point transformed_point = transform_point(*msg, st_transform);

    if (new_point_arrived) {
        // Reset filters
        new_point_arrived = false;
        point_x = transformed_point.x;
        point_y = transformed_point.y;
        point_z = transformed_point.z;
    }
    else {
        // Filter
        point_x = EXP_SMOOTH(point_x, transformed_point.x, POINT_SMOOTH_FACTOR);
        point_y = EXP_SMOOTH(point_y, transformed_point.y, POINT_SMOOTH_FACTOR);
        point_z = EXP_SMOOTH(point_z, transformed_point.z, POINT_SMOOTH_FACTOR);
    }

    // Publish in answer format
    srcsim::Console out_msg;
    out_msg.x = point_x;
    out_msg.y = -point_y;
    out_msg.z = -point_z;
    out_msg.r = point_r;
    out_msg.g = point_g;
    out_msg.b = point_b;

    pub.publish(out_msg);
}

// /**
//  * Callback for when lidar point arrives
//  */
// void li_point_arrived(const geometry_msgs::Point::ConstPtr& msg) {
//     // Get points
//     // TODO: Kalman filter here
//     point_z = (*msg).z;
// }


int main(int argc, char **argv) {
    ros::init(argc, argv, "point_aggregator");
    ros::NodeHandle nh;

    // Stereo transform matrix 
    // (from tf_transcriber)
    st_transform(0,0) = -3.67321e-06; st_transform(0,1) = -3.67321e-06; st_transform(0,2) =  1;           st_transform(0,3) =  0;
    st_transform(1,0) = -1;           st_transform(1,1) =  1.34925e-11; st_transform(1,2) = -3.67321e-06; st_transform(1,3) =  0.035;
    st_transform(2,0) =  0;           st_transform(2,1) = -1;           st_transform(2,2) = -3.67321e-06; st_transform(2,3) = -0.002;

    // Subscribe to LED hunter
    ros::Subscriber sub_pixel = nh.subscribe<cam_proc::pixel_loc>("/LED_im_loc", 
                                                                1, pixel_loc_arrived);

    // Subscribe to stereo 3D point locator
    ros::Subscriber sub_cloud = nh.subscribe<geometry_msgs::Point>("/stereo_coordinate", 
                                                            1, st_point_arrived);

    //ros::Subscriber sub_pixel = nh.subscribe<geometry_msgs::Point>("/lidar_coordinate", 
    //                                                        1, li_point_arrived);

    pub = nh.advertise<srcsim::Console>("/srcsim/qual1/light", 1);
    ros::spin();
}