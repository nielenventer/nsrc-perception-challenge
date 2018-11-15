import cv2
import numpy as np
import os, sys
import matplotlib.pyplot as plt


def get_LED_with_colour_filter(image, debug=False):
    # Colour ranges
    c_ranges = {"blue":  ((250,   0,   0), (255,  10,  10)),
                "green": ((0,   250,   0), ( 10, 255,  10)),
                "red":   ((0,     0, 250), ( 10,  10, 255)),
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

        if debug:
            plt.subplot(231 + i)
            plt.title(col)
            plt.imshow(imgray, cmap="gray")

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
                # Get centroid
                M  = cv2.moments(cont)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                if debug:
                    plt.subplot(212)
                    cv2.circle(image, (cx, cy), 0, 0, 5) # Black marker
                    cv2.circle(image, (cx, cy), 0, (255, 255, 255), 4) # White marker
                    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    plt.show()

                return (col, cx, cy)

    if debug:
        plt.subplot(212)
        plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)) 
        plt.show()
    return ("none", 0, 0) # No LED found

if __name__ == "__main__":
    TEST_IMAGES_FOLDER = '/'.join(os.path.dirname(os.path.realpath(__file__)).split('/')[:-1]) \
                            + "/test_data/single_camera_images/" 
    image_url = TEST_IMAGES_FOLDER + "red_409_380_0.png"

    image = cv2.imread(image_url)
    if image is None:
        print "Couldn't load image at: %s" % image_url
        sys.exit(-1)

    get_LED_with_colour_filter(image, True)

