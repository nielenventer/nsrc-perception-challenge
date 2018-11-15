#!/usr/bin/env python
import roslib
roslib.load_manifest("cam_proc")
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cam_proc.msg import pixel_loc
from cv_bridge import CvBridge, CvBridgeError

CV_DEBUG_WINDOW_NAME = "display_im"


class LEDHunter(object):
    def __init__(self, debug=True):
        self.pub = rospy.Publisher("LED_im_loc", pixel_loc, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/multisense/camera/left/image_raw", 
                                            Image, self._image_arrived)

        self.debug = debug

    def get_LED_with_colour_filter(self, image):
        # Colour ranges
        c_ranges = {"blue":  ((250,   0,   0), (255,   10,   10)),
                    "green": ((0,   250,   0), (  10, 255,   10)),
                    "red":   ((0,     0, 250), (  10,   10, 255)),
                    }

        for i, col in enumerate(c_ranges):
            # create arrays from the boundaries
            lower = np.array(c_ranges[col][0], dtype="uint8")
            upper = np.array(c_ranges[col][1], dtype="uint8")
         
            # find the colors within the specified boundaries and apply the mask
            mask   = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask=mask)

            # Convert to binary image
            imgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
            imgray = cv2.medianBlur(imgray, 3) # Get rid of speckles
            _, thresh = cv2.threshold(imgray, 10, 255, cv2.THRESH_BINARY)

            # Get contours/objects
            try:
                _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            except:
                # output depends on opencv version
                contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cont in contours:
                # Get contour area, and if it is large enough
                # assume it is the target LED
                cont_size = cv2.contourArea(cont)
                if cont_size > 10:
                    M  = cv2.moments(cont)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    return (col, cx, cy)

        return ("none", 0, 0) # No LED found

    def _find_LED_location(self, image):
        # Find LED location and colour
        LED_loc = self.get_LED_with_colour_filter(image)

        # Put results into message
        msg = pixel_loc()
        msg.x = LED_loc[1]
        msg.y = LED_loc[2]
        msg.r, msg.g, msg.b = {"blue":  (0, 0, 1),
                               "green": (0, 1, 0),
                               "red":   (1, 0, 0),
                               "none":  (0, 0, 0)}[LED_loc[0]]
        return msg

    def _image_arrived(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # Get LED location
        msg = self._find_LED_location(cv_image)

        # Publish location of LED in image
        self.pub.publish(msg)

        if self.debug:
            cv2.namedWindow(CV_DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(CV_DEBUG_WINDOW_NAME, 600, 400)

            # Draw circle at LED location
            colour = (255*msg.b, 255*msg.g, 255*msg.r)
            if colour != (0, 0, 0):
                loc = (msg.x, msg.y)
                cv2.circle(cv_image, loc, 0, 0, 5) # Black border
                cv2.circle(cv_image, loc, 0, colour, 4)

            cv2.imshow(CV_DEBUG_WINDOW_NAME, cv_image)
            cv2.waitKey(1)


def main(args):
    rospy.init_node('LED_hunter', anonymous=True)
    lh = LEDHunter()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException: 
        print "Shutting down"
        cv2.destroyAllWindows()
     
if __name__ == '__main__':
    main(sys.argv)