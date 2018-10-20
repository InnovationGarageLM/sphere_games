import time

import numpy as np
import rospy
import cv2
import sys
import constants
import utilities
from base_tracker import BaseTracker

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped

class SpheroTracker(BaseTracker):
    '''
    Updated Tracker for Spheros
    '''

    def __init__(self, tracked_ids = ['red_sphero', 'blue_sphero']):

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

        self.pub_mask = {}

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

    def find_center(self,who,img):
        dat = [ 2,1,6,250 ] if who == 'red_sphero' else [ 0,4,8,225 ]
        img = img[:,:,dat[0]] #red channel
        _,img = cv2.threshold(img,dat[3],255,cv2.THRESH_BINARY)
        img = cv2.erode(img,None,iterations=dat[1])
        img = cv2.dilate(img, None, iterations=dat[2])
        mom = cv2.moments(img)
        div = mom['m00']
        if div == 0: return None
        cx = int(mom['m10'] / div)
        cy = int(mom['m01'] / div)
        return Point(cx,cy,0)
        

    def get_spheros(self,img):
        img = img[constants.ARENA_BOUNDS['top']:constants.ARENA_BOUNDS['bottom'],
                  constants.ARENA_BOUNDS['left']:constants.ARENA_BOUNDS['right']]
        centers = {}
        for who in ['red_sphero','blue_sphero']:
            centers[who] = self.find_center(who,img)
            if centers[who] is not None:
                centers[who].y += constants.ARENA_BOUNDS['top']
                centers[who].x += constants.ARENA_BOUNDS['left']
        return centers

    def process_frame(self, image_data):
        '''
        Take a specific image and identify sphero locations
        :return:
        '''
        stamp = image_data.header.stamp

        cv2_img = self.bridge.compressed_imgmsg_to_cv2(image_data, desired_encoding="passthrough")

        self.latest_img = cv2_img

        spheros = self.get_spheros(cv2_img)

        self.update_locations(spheros, stamp)

        super(SpheroTracker, self).process_frame(image_data)


if(__name__ == "__main__"):
    t = SpheroTracker()
    t.init_ros()
    t.start_tracking()
