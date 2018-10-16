import time

import numpy as np
import rospy
import cv2
import sys
import constants
import utilities
from host.base_tracker import BaseTracker

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped

class SpheroTracker(BaseTracker):
    '''
    Updated Tracker for Spheros
    '''

    def __init__(self, tracked_ids = ['red_node', 'blue_node']):

        super(SpheroTracker, self).__init__(tracked_ids=tracked_ids)

        empty_point = PointStamped()

        # Available Images
        self.ref_image = None
        self.latest_img = None
        self.masked_image = None
        self.diff_image = None

        self.blur_level = 25

        # Filter Details
        self.running_average = {}
        self.last_good_value = {}
        self.reset_filter_count = {}

        # Whether reference frame was initalized
        self.initialized = False

        version = cv2.__version__.split('.')

        if(int(version[0]) < 3):
            print("Please upgrade your opencv version to 3.0, Current OpenCV Version: " + version[0] + "." + version[1])
            print("  sudo pip install opencv-python")
            exit(1)

        print("Good to go! OpenCV Version: " + version[0] + "." + version[1])


    def init_subscribers(self):

        super(SpheroTracker, self).init_subscribers()

        self.sub_image = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.process_frame,
                                          queue_size=1)

    def init_publishers(self):

        super(SpheroTracker, self).init_publishers()

        self.pub_masked_image   = rospy.Publisher('/arena/masked_image', Image, queue_size=1)
        self.pub_arena_image    = rospy.Publisher('/arena/game_image', Image, queue_size=1)
        self.pub_diff_image     = rospy.Publisher('/arena/diff_image', Image, queue_size=1)

        for color in constants.COLOR_THRESHOLDS:
            self.pub_mask[color] = rospy.Publisher('/arena/'+color+'_mask', Image, queue_size=1)

    def setup_reference_image(self, img):
        '''
        Create Reference Image to subtract from
        :return:
        '''

        # Check to confirm field is empty

        if sys.version_info[0] == 3:
            response = str(input("Is Field Clear of Spheros? (If not clear it now and type 'n') (Y/N):")).lower()
        else:
            response = raw_input("Is Field Clear of Spheros? (If not clear it now and type 'n') (Y/N):").lower()

        if(not(response == "y" or response == "yes")):
            return

        kernel = np.ones((15, 15), np.float32) / 225
        smoothed = cv2.filter2D(img, -1, kernel)

        cv2_img_blur = cv2.GaussianBlur(smoothed, (15,15),0)

        grey = cv2.cvtColor(cv2_img_blur, cv2.COLOR_BGR2GRAY)

        self.ref_image = grey
        self.initialized = True
        print("Tracker Initialized")

    def get_average_intensity(self, img, circle):

        new_img = img.copy()

        circle_img = np.zeros(new_img.shape, np.uint8)
        cv2.circle(circle_img, (circle[0], circle[1]), circle[2], 1, thickness=-1)

        masked_data = cv2.bitwise_and(new_img, new_img, mask=circle_img)

        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(masked_data)

        return maxVal

    def find_circles(self, mask):
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1 = 50, param2 = 8,
                                   minRadius = 0, maxRadius = int(90 * constants.COVERT_MM2PIXEL))

        if(circles is None):
            return None

        maxVal = 0

        pt = None

        for (x,y,r) in circles[0,:]:
            # Only accept point if within bounds of arena
            if (constants.ARENA_BOUNDS['left'] < x < constants.ARENA_BOUNDS['right']
              and constants.ARENA_BOUNDS['top'] < y < constants.ARENA_BOUNDS['bottom']):

                val = self.get_average_intensity(mask, (x,y,r))

                if(val > maxVal):
                    maxVal = val
                    pt = Point(x,y,0)
        return pt

    def get_spheros(self, cv2_image):

        masks = self.get_masks(cv2_image)

        centers = {}
        for mask in masks:
            self.pub_mask[mask].publish(self.bridge.cv2_to_imgmsg(masks[mask], encoding="mono8"))
            centers[mask] = self.find_circles(masks[mask])

        return centers

    def get_mask(self, cv2_img):
        '''
        Take in CV2 Image and return masked image
        :param cv2_img:
        :return:
        '''

        # Smooth and Filter
        kernel = np.ones((15, 15), np.float32) / 225
        smoothed = cv2.filter2D(cv2_img, -1, kernel)

        cv2_img_blur = cv2.GaussianBlur(smoothed, (15,15),0)

        grey = cv2.cvtColor(cv2_img_blur, cv2.COLOR_BGR2GRAY)

        img_diff = cv2.absdiff(grey, self.ref_image)
        self.diff_image = self.bridge.cv2_to_imgmsg(img_diff, encoding="mono8")

        ret, mask = cv2.threshold(img_diff, 10, 255, cv2.THRESH_BINARY)

        masked_img = cv2.bitwise_and(cv2_img, cv2_img, mask=mask)

        # Extract only arena portion (e.g. zero everything outside arena)
        # (technique from 'stackoverflow.com/questions/11492214')
        extracted = np.zeros(masked_img.shape,np.uint8)
        extracted[constants.ARENA_BOUNDS['top']:constants.ARENA_BOUNDS['bottom'],
        constants.ARENA_BOUNDS['left']:constants.ARENA_BOUNDS['right']] \
          = masked_img[constants.ARENA_BOUNDS['top']:constants.ARENA_BOUNDS['bottom'],
            constants.ARENA_BOUNDS['left']:constants.ARENA_BOUNDS['right']]
        return extracted

    def pub_images(self):

        super(SpheroTracker, self).pub_images()

        if (not self.masked_image is None):
            self.pub_masked_image.publish(self.masked_image)

        if (not self.diff_image is None):
            self.pub_diff_image.publish(self.diff_image)

    def process_frame(self, image_data):
        '''
        Take a specific image and identify sphero locations
        :return:
        '''
        stamp = image_data.header.stamp

        cv2_img = self.bridge.compressed_imgmsg_to_cv2(image_data, desired_encoding="passthrough")

        self.latest_img = cv2_img

        # Check if initialized, if not save image
        if(not self.initialized):
            self.setup_reference_image(cv2_img)
            return

        # Mask Field
        masked_img = self.get_mask(cv2_img)
        self.masked_image = self.bridge.cv2_to_imgmsg(masked_img, encoding="bgr8")

        # Do Processing
        spheros = self.get_spheros(masked_img)

        self.update_locations(spheros, stamp)

        super(SpheroTracker, self).process_frame(image_data)


if(__name__ == "__main__"):
    t = SpheroTracker()
    t.init_ros()
    t.start_tracking()
