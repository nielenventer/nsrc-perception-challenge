#include <ros/ros.h>
// #include "ros/common.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
// #include <tf/transform_datatypes.h>
// #include <eigen_conversions/eigen_msg.h>

 
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "tf_transcriber"); 
    ros::NodeHandle nh;

    tf::TransformListener listener;

    ros::Rate r(1); 
    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/head",
                                     "/left_camera_optical_frame",   
                                     ros::Time(0), transform);

            // Convert to Eigen
            Eigen::Affine3d transform_eigen;
            tf::transformTFToEigen(transform, transform_eigen);

            // Log
            std::cout << "Found transform: " << std::endl << transform_eigen.matrix() << std::endl;
            std::cout << "Inverse: " << std::endl << transform_eigen.matrix().inverse() << std::endl;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
        }

        r.sleep(); 
    }
}