import unittest
import cv2
import LED_huntin
import os, glob

class LEDLocationTest(unittest.TestCase):
    # folder containing test data
    TEST_IMAGES_FOLDER = '/'.join(os.path.dirname(os.path.realpath(__file__)).split('/')[:-1]) \
                            + "/test_data/single_camera_images/" 

    # Max allowed deviation from target location
    LOC_EPSILON = 3

    def _load_image_at_URL(self, image_url):
        image = cv2.imread(image_url)
        
        if image is None:
            self.fail("Couldn't load image")
        else:
            return image

    def _expected_outputs_from_url(self, url):
        exp = url.split("/")[-1].split(".")[0].split("_")
        return (exp[0], int(exp[1]), int(exp[2]))

    def _compare_outputs(self, exp_outputs, alg_outputs):
        # First element (colour) must be exactly the same
        self.assertEqual(exp_outputs[0], alg_outputs[0]) 

        # Other elements (location) must be withint tolerance
        self.assertTrue(abs(exp_outputs[1] - alg_outputs[1]) < self.LOC_EPSILON)
        self.assertTrue(abs(exp_outputs[2] - alg_outputs[2]) < self.LOC_EPSILON)

    def _test_all_images_func(self, func):
        # Get all images in test_data folder
        for im_url in glob.glob(self.TEST_IMAGES_FOLDER + "*.png"):
            print "Testing: %s" % im_url

            # Load image
            image = self._load_image_at_URL(im_url)

            # Get expected outputs and actual outputs
            exp_outputs = self._expected_outputs_from_url(im_url)
            alg_outputs = func(image)

            # Compare
            self._compare_outputs(exp_outputs, alg_outputs)


    # Tests for specific algorithms go here
    def test_LED_with_colour_filter(self):
        self._test_all_images_func(LED_huntin.get_LED_with_colour_filter)


if __name__ == "__main__":
    unittest.main()