import time

import numpy as np
import rospy
import cv2
import sys
import constants
import utilities

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped

from cv_bridge import CvBridge

class SpheroTracker():
    '''
    Updated Tracker for Spheros
    '''

    def __init__(self):
        # Variables used by game
        self.base = {'red':constants.RED_BASE, 'blue': constants.BLUE_BASE}
        self.center = {'red':Point(0, 0, 0), 'blue':Point(0, 0, 0)}

        empty_point = PointStamped()

        self.red_center_mm = empty_point
        self.blue_center_mm = empty_point

        self.red_base_mm = utilities.pixels_2_mm(self.base['red'])

        self.blue_base_mm = utilities.pixels_2_mm(self.base['blue'])

        # Who has a flag
        self.flag = {'red':False, 'blue' : False}

        # Game State
        self.game_state = 0 # 0 = Waiting, 1 = Running, 2 = Finished
        self.time_elapsed = 0 # Seconds
        self.start = None

        # Current Score
        self.score = {'red':0, 'blue':0}

        self.initialized = False

        # Available Images
        self.ref_image = None
        self.latest_img = None
        self.masked_image = None
        self.arena_image = None
        self.diff_image = None

        self.blur_level = 25

        self.bridge = CvBridge()

        # Filter Details
        self.running_average = {}
        self.last_good_value = {}
        self.reset_filter_count = {}

        version = cv2.__version__.split('.')

        if(int(version[0]) < 3):
            print("Please upgrade your opencv version to 3.0, Current OpenCV Version: " + version[0] + "." + version[1])
            print("  sudo pip install opencv-python")
            exit(1)

        print("Good to go! OpenCV Version: " + version[0] + "." + version[1])

    def convert_pixels_mm(self, pt_pixels, time):
        # problem is this forces values to int
        #pt_mm = utilities.pixels_2_mm(pt_pixels)

        x = pt_pixels.x - constants.ORIGIN_PIXELS.x
        y = -(pt_pixels.y - constants.ORIGIN_PIXELS.y) # flip so negative is down
        # Scale
        x_mm = x * constants.COVERT_PIXEL2MM
        y_mm = y * constants.COVERT_PIXEL2MM

        p = PointStamped()
        p.header.stamp = time
        p.point.x = x_mm
        p.point.y = y_mm
        return p

    def update_time(self):
        self.time_elapsed = time.time() - self.start
        if(self.time_elapsed >= constants.TOTAL_ALLOWED_TIME):
            self.game_state = 2

    def update_game_state(self):

        if(self.game_state == 0): # Waiting State
            self.time_elapsed = 0
            pass
        elif(self.game_state == 1): # Running
            self.update_scoring()
            self.update_time()
        elif(self.game_state == 2): # Complete
            pass
        elif(self.game_state == 3): # Test Mode
            self.time_elapsed = 0
            pass
        else: # Invalid States
            pass

    def process_start(self, do_start):

        # Start Game if in waiting state
        if(self.game_state == 0 and do_start.data == True):
            self.game_state = 1
            self.start = time.time()


    def process_reset(self, do_reset):

        # Start Game if in waiting state
        if(do_reset.data == True):
            self.game_state = 0
            self.start = None
            self.time_elapsed = 0
            self.score['red'] = 0
            self.score['blue'] = 0

    def init_publishers(self):
        self.pub_red_center     = rospy.Publisher('/arena/red_sphero/center', Point, queue_size=1)
        self.pub_red_center_mm  = rospy.Publisher('/arena/red_sphero/center_mm', PointStamped, queue_size=1)
        self.pub_red_base       = rospy.Publisher('/arena/red_sphero/base', Point, queue_size=1)
        self.pub_red_base_mm    = rospy.Publisher('/arena/red_sphero/base_mm', Point, queue_size=1)
        self.pub_red_flag       = rospy.Publisher('/arena/red_sphero/flag', Bool, queue_size=1)
        self.pub_red_score      = rospy.Publisher('/arena/red_sphero/score', Int16, queue_size=1)

        self.pub_blue_center    = rospy.Publisher('/arena/blue_sphero/center', Point, queue_size=1)
        self.pub_blue_center_mm = rospy.Publisher('/arena/blue_sphero/center_mm', PointStamped, queue_size=1)
        self.pub_blue_base      = rospy.Publisher('/arena/blue_sphero/base', Point, queue_size=1)
        self.pub_blue_base_mm   = rospy.Publisher('/arena/blue_sphero/base_mm', Point, queue_size=1)
        self.pub_blue_flag      = rospy.Publisher('/arena/blue_sphero/flag', Bool, queue_size=1)
        self.pub_blue_score     = rospy.Publisher('/arena/blue_sphero/score', Int16, queue_size=1)

        self.pub_game_state     = rospy.Publisher('/arena/game_state', Int16, queue_size=1)
        self.pub_time_elapsed   = rospy.Publisher('/arena/time_elapsed', Int16, queue_size=1)

        self.pub_masked_image   = rospy.Publisher('/arena/masked_image', Image, queue_size=1)
        self.pub_blue_mask      = rospy.Publisher('/arena/blue_mask', Image, queue_size=1)
        self.pub_red_mask       = rospy.Publisher('/arena/red_mask', Image, queue_size=1)
        self.pub_arena_image    = rospy.Publisher('/arena/game_image', Image, queue_size=1)
        self.pub_diff_image     = rospy.Publisher('/arena/diff_image', Image, queue_size=1)

        self.sub_image = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.process_frame, queue_size=1)

        # Game Controls
        self.sub_image = rospy.Subscriber('/arena/start_game', Bool, self.process_start, queue_size=1)
        self.sub_image = rospy.Subscriber('/arena/reset_game', Bool, self.process_reset, queue_size=1)

        rospy.init_node('sphere_tracker', anonymous=True)

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

    def find_contours(self, mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None

        if len(contours) > 0:
            moments = cv2.moments(max(contours, key=cv2.contourArea))
            center = Point(int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']), 0)

        return center


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

        # Mask by hue and find center
        hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        red_lower_1 = np.array([0, 50, 50])
        red_upper_1 = np.array([10, 255, 255])
        red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)

        red_lower_2 = np.array([160, 50, 50])
        red_upper_2 = np.array([180, 255, 255])
        red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)

        red_mask = red_mask_1 + red_mask_2
        red_mask = cv2.erode(red_mask, None, iterations=2)
        red_mask = cv2.dilate(red_mask, None, iterations=2)

        blue_lower = np.array([110, 50, 50])
        blue_upper = np.array([120, 255, 255])
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        blue_mask = cv2.erode(blue_mask, None, iterations=2)
        blue_mask = cv2.dilate(blue_mask, None, iterations=2)

        self.pub_blue_mask.publish(self.bridge.cv2_to_imgmsg(blue_mask, encoding="mono8"))
        self.pub_red_mask.publish(self.bridge.cv2_to_imgmsg(red_mask, encoding="mono8"))

        #blue_center = self.find_contours(blue_mask)
        #red_center = self.find_contours(red_mask)

        blue_center = self.find_circles(blue_mask)
        red_center = self.find_circles(red_mask)

        return {'blue':blue_center, 'red':red_center}

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

    def update_locations(self, blue_pt, red_pt, stamp):

        # Return Sphero locations
        if(not red_pt is None):
            #red_pt = self.filter('red', red_pt)
            self.center['red'] = red_pt
            self.red_center_mm = self.convert_pixels_mm(red_pt, stamp)

        if (not blue_pt is None):
            #blue_pt = self.filter('blue', blue_pt)
            self.center['blue'] = blue_pt
            self.blue_center_mm = self.convert_pixels_mm(blue_pt, stamp)

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

        self.update_locations(spheros['blue'], spheros['red'], stamp)

    # Scoring logic
    def update_scoring(self):

        red_at_away = False
        red_at_home = False
        blue_at_away = False
        blue_at_home = False

        threshold = 100
        if self.flag['red'] != False:
            distance = utilities.calculate_distance(self.center['red'], self.base['red'])

            if distance < threshold:
                red_at_home = True
        else:
            distance = utilities.calculate_distance(self.center['red'], self.base['blue'])

            if distance < threshold:
                red_at_away = True

        if self.flag['blue'] != False:
            distance = utilities.calculate_distance(self.center['blue'], self.base['blue'])

            if distance < threshold:
                blue_at_home = True
        else:
            distance = utilities.calculate_distance(self.center['blue'], self.base['red'])

            if distance < threshold:
                blue_at_away = True

        # Scenarios
        if red_at_home and blue_at_home: # Tie
            self.score['red'] += 1
            self.score['blue'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        elif red_at_home: # Red Return
            self.score['red'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        elif blue_at_home: # Blue Return
            self.score['blue'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        else: # Picked up flag
            if red_at_away:
                self.flag['red'] = True
            if blue_at_away:
                self.flag['blue'] = True
        return

    def start_tracking(self):

        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():

            self.update_game_state()

            if(not self.latest_img is None):
                #self.arena_image = self.update_arena(self.latest_img)
                self.arena_image = utilities.update_arena(self.game_state, self.time_elapsed,
                                                          self.score, self.center, self.base,
                                                          self.flag, self.latest_img)
                arena_image = self.bridge.cv2_to_imgmsg(self.arena_image, encoding="bgr8")
                self.pub_arena_image.publish(arena_image)

            if(not self.masked_image is None):
                self.pub_masked_image.publish(self.masked_image)

            if (not self.diff_image is None):
                self.pub_diff_image.publish(self.diff_image)

            # Publish Centers
            if(self.center['red'] is not None):
                self.pub_red_center.publish(self.center['red'])
                self.pub_red_center_mm.publish(self.red_center_mm)

            if (self.center['blue'] is not None):
                self.pub_blue_center.publish(self.center['blue'])
                self.pub_blue_center_mm.publish(self.blue_center_mm)

            self.pub_red_base.publish(self.base['red'])
            self.pub_blue_base.publish(self.base['blue'])

            self.pub_red_base_mm.publish(self.red_base_mm)
            self.pub_blue_base_mm.publish(self.blue_base_mm)

            # Publish Game State
            self.pub_game_state.publish(self.game_state)
            self.pub_time_elapsed.publish(self.time_elapsed)
            self.pub_red_flag.publish(self.flag['red'])
            self.pub_blue_flag.publish(self.flag['blue'])

            self.pub_red_score.publish(self.score['red'])
            self.pub_blue_score.publish(self.score['blue'])

            rate.sleep()

        pass



if(__name__ == "__main__"):
    t = SpheroTracker()
    t.init_publishers()
    t.start_tracking()
