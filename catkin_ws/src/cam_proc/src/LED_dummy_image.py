#!/usr/bin/env python
import roslib
roslib.load_manifest('cam_proc')
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os

TEST_IMAGE_PATH = "/development/test_data/single_camera_images/red_409_380.png"


def dummy_LED_image():
    pub = rospy.Publisher("/multisense/camera/left/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    # Load image
    qual1_fold_url = '/'.join(os.path.dirname(os.path.realpath(__file__)).split('/')[:-4])
    test_image_url = qual1_fold_url + TEST_IMAGE_PATH
    im_from_disk   = cv2.imread(test_image_url)

    # Main loop.
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            img = bridge.cv2_to_imgmsg(im_from_disk, "bgr8")
            pub.publish(img)
        except CvBridgeError, e:
            print e
        r.sleep();


# Main function.
if __name__ == '__main__':
    rospy.init_node('LED_image_dummy', anonymous=True)
    try:
        dummy_LED_image()
    except rospy.ROSInterruptException: 
        pass