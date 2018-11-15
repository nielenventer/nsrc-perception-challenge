#!/usr/bin/env python
import roslib
roslib.load_manifest("cam_proc")
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cam_proc.msg import pixel_loc
from cv_bridge import CvBridge, CvBridgeError

CV_DEBUG_WINDOW_NAME = "display_im"


class LEDHunter(object):
    def __init__(self, debug=True):
        self.pub = rospy.Publisher("LED_im_loc", pixel_loc, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/multisense/camera/left/image_raw", 
                                            Image, self._image_arrived)

        self.msg = pixel_loc()
        self.msg.x, self.msg.y = 400, 250
        self.msg.r, self.msg.g, self.msg.b = 1, 0, 0
        self.display_im = None

        self.debug = debug

    def _on_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Single click -> get x,y of mouse click
            self.msg.x, self.msg.y = x, y

        if event == cv2.EVENT_LBUTTONDBLCLK:
            # Double click -> change LED colour
            current = self.msg.r, self.msg.g, self.msg.b
            self.msg.r, self.msg.g, self.msg.b = \
                {(0, 0, 0): (1, 0, 0),
                 (1, 0, 0): (0, 1, 0),
                 (0, 1, 0): (0, 0, 1),
                 (0, 0, 1): (0, 0, 0),}[current]

    def _image_arrived(self, data):
        try:
            self.display_im = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # Publish location of LED in image
        self.pub.publish(self.msg)

        if self.debug:
            cv2.namedWindow(CV_DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(CV_DEBUG_WINDOW_NAME, 600, 400)
            cv2.setMouseCallback(CV_DEBUG_WINDOW_NAME, self._on_click)

            # Draw circle at LED location
            colour = (255*self.msg.b, 255*self.msg.g, 255*self.msg.r)
            if colour != (0, 0, 0):
                loc = (self.msg.x, self.msg.y)
                cv2.circle(self.display_im, loc, 0, 0, 5) # Black border
                cv2.circle(self.display_im, loc, 0, colour, 4)

            cv2.imshow(CV_DEBUG_WINDOW_NAME, self.display_im)
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