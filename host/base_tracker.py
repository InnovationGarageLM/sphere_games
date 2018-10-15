
from cv_bridge import CvBridge

import rospy

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped

import constants
import utilities as util

import time
import cv2

class BaseTracker(object):


    def __init__(self, tracked_ids = ['blue', 'red']):
        '''
        Initializes base tracker to track specified ids
        :param tracked_ids: ids to track, By default will track blue and red
        '''

        self.ids = tracked_ids

        self.base = {}
        self.base_mm = {}

        self.center = {}
        self.center_mm = {}

        self.flag = {}
        self.score = {}

        self.pub_center = {}
        self.pub_center_mm = {}
        self.pub_base = {}
        self.pub_base_mm = {}
        self.pub_flag = {}
        self.pub_score = {}

        zeroPoint = PointStamped()
        zeroPoint.header.stamp = rospy.Time()
        zeroPoint.point = Point(0,0,0)

        for color in self.ids:

            self.base[color] = constants.BASE[color]
            self.base_mm[color] = util.pixels_2_mm(self.base[color])

            self.center[color] = Point(0, 0, 0)
            self.center_mm[color] = zeroPoint
            self.flag[color] = False

            # Current Score
            self.score[color] = 0



        # Image of Arena
        self.game_image = None
        self.latest_img = None

        # Game State
        self.game_state = 0 # 0 = Waiting, 1 = Running, 2 = Finished, 3 = Test
        self.time_elapsed = 0 # Seconds
        self.start = None

        self.bridge = CvBridge()


    # Scoring logic
    def update_scoring(self):

        # Check to see if any teams already have a flag, and if they have returned to base
        returned_home = {}

        for id in self.ids:

            if(self.flag[id] == True):
                distance = util.calculate_distance(self.center[id], self.base[id])

                if(distance < constants.SCORE_THRESHOLD):
                    returned_home[id] = True

        # Check to see if anyone scored
        if(len(returned_home) > 0):

            for id in returned_home:
                # Update Scores
                self.score[id] = self.score[id] + 1

            # Reset Flags
            for id in self.ids:
                self.flag[id] = False

            return

        # No one has scored, update flags as appropriate
        for id in self.ids:
            for opp_id in self.ids:

                if (id == opp_id):
                    continue # Skip if the same

                distance = util.calculate_distance(self.center[id], self.base[opp_id])

                if (distance < constants.SCORE_THRESHOLD):
                    self.flag[id] = True

    def get_masks(self, cv2_image):

        # Mask by hue and find center
        hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        masks = {}

        for threshold in constants.COLOR_THRESHOLDS:
            mask = None

            for (lower, upper) in constants.COLOR_THRESHOLDS[threshold]:
                if (mask is None):
                    mask = cv2.inRange(hsv, lower, upper)
                else:
                    mask_tmp = cv2.inRange(hsv, lower, upper)
                    mask = mask + mask_tmp

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            masks[threshold] = mask
        return masks

    def process_frame(self, image, compressed = True):
        '''
        This should be overwritten by your specific tracker, updating the positions of the various items, however
        do call at the end to update the game state
        :param image: Incoming image, assumed to be compressed
        :return:
        '''

        if(compressed):
            self.latest_img = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding="passthrough")
        else:
            self.latest_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    def init_ros(self):
        rospy.init_node('sphere_tracker', anonymous=True)
        self.init_publishers()
        self.init_subscribers()

    def init_subscribers(self):

        # Game Controls
        self.sub_image = rospy.Subscriber('/arena/start_game', Bool, self.process_start, queue_size=1)
        self.sub_image = rospy.Subscriber('/arena/reset_game', Bool, self.process_reset, queue_size=1)
        self.sub_image = rospy.Subscriber('/arena/test_mode', Bool, self.process_test_mode, queue_size=1)


    def update_locations(self, sphero_pts, stamp):
        '''
        Update Locations of Spheros
        :param sphero_pts: List of Points to update at the specific time stamp, in pixels
        :param stamp: Time Stamp for points
        :return:
        '''

        for key in sphero_pts:

            if(sphero_pts[key] is not None):
                self.center[key] = sphero_pts[key]
                self.center_mm[key] = self.convert_pixels_mm(sphero_pts[key], stamp)


    def init_pub_id(self, id):

        self.pub_center[id] = rospy.Publisher('/arena/' + id + '/center', Point, queue_size=1)
        self.pub_center_mm[id] = rospy.Publisher('/arena/' + id + '/center_mm', PointStamped, queue_size=1)
        self.pub_base[id] = rospy.Publisher('/arena/' + id + '/base', Point, queue_size=1)
        self.pub_base_mm[id] = rospy.Publisher('/arena/' + id + '/base_mm', Point, queue_size=1)
        self.pub_flag[id] = rospy.Publisher('/arena/' + id + '/flag', Bool, queue_size=1)
        self.pub_score[id] = rospy.Publisher('/arena/' + id + '/score', Int16, queue_size=1)

    def init_publishers(self):

        for id in self.ids:
            self.init_pub_id(id)

        self.pub_game_state     = rospy.Publisher('/arena/game_state', Int16, queue_size=1)
        self.pub_time_elapsed   = rospy.Publisher('/arena/time_elapsed', Int16, queue_size=1)

        self.pub_arena_image    = rospy.Publisher('/arena/game_image', Image, queue_size=1)


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

    def process_start(self, do_start):

        # Start Game if in waiting state
        if(self.game_state == 0 and do_start.data == True):
            self.game_state = 1
            self.start = time.time()

    def process_test_mode(self, do_start):

        # Start Game if in waiting state
        self.game_state = 3

    def process_reset(self, do_reset):

        # Start Game if in waiting state
        if(do_reset.data == True):
            self.game_state = 0
            self.start = None
            self.time_elapsed = 0

            for id in self.ids:
                self.score[id] = 0
                self.flag[id] = False

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
            self.update_scoring()
            self.time_elapsed = 0
            pass
        else: # Invalid States
            pass

    def pub_images(self):
        if (not self.latest_img is None):
            # self.arena_image = self.update_arena(self.latest_img)
            self.game_image = util.update_arena(self.game_state, self.time_elapsed,
                                                self.score, self.center, self.base,
                                                self.flag, self.latest_img)
            arena_image = self.bridge.cv2_to_imgmsg(self.game_image, encoding="bgr8")
            self.pub_arena_image.publish(arena_image)

    def pub_team_info(self, id):
        if (self.center[id] is not None):
            self.pub_center[id].publish(self.center[id])
            self.pub_center_mm[id].publish(self.center_mm[id])

        self.pub_base[id].publish(self.base[id])
        self.pub_base_mm[id].publish(self.base_mm[id])

        self.pub_flag[id].publish(self.flag[id])
        self.pub_score[id].publish(self.score[id])

    def start_tracking(self):

        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():

            self.update_game_state()

            self.pub_images()

            for id in self.ids:
                self.pub_team_info(id)

            self.pub_game_state.publish(self.game_state)
            self.pub_time_elapsed.publish(self.time_elapsed)

            rate.sleep()

        pass
