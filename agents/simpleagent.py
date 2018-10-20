import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3
import host.utilities as util
import time
from random import randint, choice
import copy
import host.constants as constants

class SimpleAgent(object):

    def __init__(self, name = 'default', opponent = 'default'):

        # Class Variables
        self.name = name
        self.arena_position = None

        self.my_position = None
        self.my_velocity = None
        self.my_velocity_mag = None
        self.my_accel = 0

        self.flag = None
        self.flag_change = False
        self.game_state = None
        self.game_over = None

        self.my_base = None
        self.their_base = None

        self.my_base_px = None
        self.their_base_px = None

        self.last_position = None
        self.start_time = time.time()



        # Control Parameters
        self.Kp = .2;
        self.Ki = .001;
        self.Kd = .1;

        self.cmd_vel = None
        self.MAX_SPEED = 20

        self.stuck = None

        self.opponent = opponent

        self.centers = {
            "A": Point(-300., 300, 0),
            "B": Point( 300., 300, 0),
            "C": Point(-300.,-300, 0),
            "D": Point( 300.,-300, 0),
        }

    def set_arena_position(self, pt):
        self.arena_position = pt

    def set_my_position(self, pt):
        self.my_position = pt

    def set_flag(self, flag):

        if(self.flag is not None and self.flag != flag.data):
            # Flag Change!
            print("Flag Change!")
            self.flag_change = True

        self.flag = flag.data

    def set_my_base(self, base):
        self.my_base = base

    def set_their_base(self, base):
        self.their_base = base

    def set_my_base_px(self, base):
        self.my_base_px = base

    def set_their_base_px(self, base):
        self.their_base_px = base

    def set_game_state(self, state):
        self.game_state = int(state.data)
        pass

    def set_odometry(self, data):
        self.my_position = data

    def set_velocity(self, data):
        self.my_velocity = data
        if(data is not None):
            self.my_velocity_mag = np.sqrt(data.point.x ** 2 + data.point.y ** 2)

    def set_accel(self, accel):
        self.my_accel = accel.data

    def setup_ros(self):

        rospy.init_node(str(self.name) + '_simple_agent', anonymous=True)

        prefix = '/' + self.name
        arena_prefix = '/arena/' + self.name

        self.pub_cmd_vel           = rospy.Publisher(prefix + '/cmd_vel', Twist, queue_size=1)
        self.pub_set_pos           = rospy.Publisher(prefix + '/set_position', Point, queue_size=1)
        self.pub_set_reset_heading = rospy.Publisher(prefix + '/reset_heading', Int16, queue_size=1)

        self.sub_get_odometry = rospy.Subscriber(prefix + '/odometry', PointStamped, self.set_odometry, queue_size=1)
        self.sub_get_vel      = rospy.Subscriber(prefix + '/velocity', PointStamped, self.set_velocity, queue_size=1)
        self.sub_get_accel    = rospy.Subscriber(prefix + '/accel',    Int16,        self.set_accel, queue_size=1)

        self.sub_center       = rospy.Subscriber(arena_prefix + '/center_mm', PointStamped, self.set_arena_position, queue_size=1)
        self.sub_base         = rospy.Subscriber(arena_prefix + '/base_mm',   Point,        self.set_my_base, queue_size=1)
        self.sub_base_pixels  = rospy.Subscriber(arena_prefix + '/base', Point, self.set_my_base_px, queue_size=1)
        self.sub_flag         = rospy.Subscriber(arena_prefix + '/flag',      Bool,         self.set_flag, queue_size=1)

        self.sub_opponent_base   = rospy.Subscriber('/arena/'+str(self.opponent) + '/base_mm', Point, self.set_their_base, queue_size=1)
        self.sub_opp_base_pixels = rospy.Subscriber('/arena/' + str(self.opponent) + '/base', Point,
                                                  self.set_their_base_px, queue_size=1)

        self.sub_game_state    = rospy.Subscriber('/arena/game_state', Int16, self.set_game_state, queue_size=1)

    def end_early(self):

        if(self.game_state == 0 or self.game_state == 2):
            return True

        if(self.flag_change):
            self.flag_change = False
            return True

        return False

    def return_false(self):
        return False

    def check_if_stuck(self):
        if(self.cmd_vel is None or self.my_velocity is None):
            self.stuck = None
            return False

        if(self.cmd_vel.linear.x > 15):
            if(self.my_velocity.point.x == 0 and self.my_velocity.point.y == 0):
                curr_time = time.time()
                if(self.stuck is None):
                    self.stuck = curr_time
                elif(curr_time - self.stuck > 5): # Need to have been stuck for at least 5 seconds
                    self.stuck = None
                    return True
            else:
                self.stuck = None

        return False

    def move(self, heading, speed, duration=2):
        t = Twist()
        t.linear = Vector3(speed, 0, 0)
        t.angular = Vector3(0, 0, heading)
        self.pub_cmd_vel.publish(t)
        rospy.sleep(duration)

    def do_movement(self, t):
        self.cmd_vel = t
        self.pub_cmd_vel.publish(t)

        if(self.check_if_stuck()):
            print("Stuck! Movement not completed")
            return  False

        return True

    def fix_heading(self):

        print("Attempting to fix heading")

        self.stop_sphero()

        for direction in range(0,360,90):

            print("Trying: "+ str(direction))

            start_pt = self.get_good_position()

            self.move(direction, 20)

            end_pt = self.get_good_position()

            linear_error = util.calculate_distance(start_pt, end_pt)

            if(linear_error > 100):
                err_heading = util.calculate_error_heading(start_pt, end_pt, positive_only=True)

                update_heading = err_heading - direction

                self.move(update_heading, 1)

                self.pub_set_reset_heading.publish(0)
                print("Updated heading by: " + str(update_heading))
                return

        print("Unable to figure out heading direction")

    def get_good_position(self):
        good_reading_cnt = 0
        old_pt = None

        rate = rospy.Rate(10)
        while (good_reading_cnt < 5):

            if (old_pt is None):
                old_pt = copy.deepcopy(self.arena_position)
                continue
            else:
                pt = copy.deepcopy(self.arena_position)

            if (pt.header.stamp == old_pt.header.stamp):
                continue

            linear_error = util.calculate_distance(old_pt.point, pt.point)

            if (linear_error < 10):
                good_reading_cnt = good_reading_cnt + 1
            else:
                good_reading_cnt = 0

            old_pt = pt

            rate.sleep()

        return pt.point

    def reset_position(self):
        # stop sphero
        self.stop_sphero()

        # Wait for tracker to stop moving
        pt = self.get_good_position()
        print("Resetting Position to: (" + str(pt.x) + ", " + str(pt.y) + ")")

        # Set Position
        self.pub_set_pos.publish(pt)

    def play_game(self):

        while not rospy.is_shutdown():
            if(not self.their_base is None and not self.my_base is None and not self.game_state is None):
                break

        start_msg_shown = False
        game_start_msg_shown = False
        game_end_msg_shown = False

        rate = rospy.Rate(5)  # Hz

        while not rospy.is_shutdown():

            if(self.game_state == 0): # Waiting for game to start
                if(not start_msg_shown):
                    print("Waiting for game to start...")

                    start_msg_shown = True
                    game_start_msg_shown = False
                    game_end_msg_shown = False
                pass
            elif(self.game_state == 1): # Game Active
                if(not game_start_msg_shown):
                    print("Starting Game...")
                    start_msg_shown = False
                    game_start_msg_shown = True
                    game_end_msg_shown = False

                if(self.flag):
                    self.go_via_path(self.my_base, self.end_early)
                else:
                    self.go_via_path(self.their_base, self.end_early)
                self.reset_position()

            elif(self.game_state == 2): # Game Over
                if (not game_end_msg_shown):
                    print("Game Ended")
                    start_msg_shown = False
                    game_start_msg_shown = False
                    game_end_msg_shown = True
                pass
            elif(self.game_state == 3): # Test Mode
                if(not game_start_msg_shown):
                    print("Entering Test Mode...")
                    start_msg_shown = False
                    game_start_msg_shown = True
                    game_end_msg_shown = False

                if (self.flag):
                    self.go_via_path(self.my_base, self.end_early)
                else:
                    self.go_via_path(self.their_base, self.end_early)
                self.reset_position()

            rate.sleep()

    def stop_sphero(self):
        t = Twist()
        t.linear = Vector3(0, 0, 0)
        t.angular = Vector3(0, 0, 0)

        self.do_movement(t)

    def get_cell(self, pt):
        '''
        Determines which of the 4 cells the robot is in,
        A is upper right, B is upper left,
        C is lower right, D is lower left

        |-------|-------|
        |   A   |   B   |
        |-------|-------|
        |   C   |   D   |
        |-------|-------|

        :param pt:
        :return: letter of cell
        '''
        if(pt.x < 0 and pt.y < 0):
            return 'C'
        elif (pt.x < 0 and pt.y > 0):
            return 'A'
        elif (pt.x > 0 and pt.y > 0):
            return 'B'
        elif (pt.x > 0 and pt.y < 0):
            return 'D'

        return None

    def path_plan(self, start, end):

        if(end is None):
            return []

        if(start == end):
            return [end]

        if (end == 'A' and start == 'D'):
            return [start, 'B', end]
        elif (end == 'B' and start == 'C'):
            return [start, 'A', end]
        elif (end == 'C' and start == 'B'):
            return [start, 'D', end]
        elif (end == 'D' and start == 'A'):
            return [start, 'C', end]
        else:
            return [start, end]

    def go_via_path(self, target, monitor_function = None):

        if(monitor_function is None):
            monitor_function = self.end_early

        if(self.arena_position is None):
            print("Position invalid, returning")
            return False

        my_pt = copy.deepcopy(self.arena_position.point)
        my_target = copy.deepcopy(target)

        if(my_pt is None or my_target is None):
            print("Target Invalid, returning")
            return False

        print("Attempting to go via path to: (" + str(target.x) + ", " + str(target.y) + ")")


        target_cell = self.get_cell(my_target)
        my_cell = self.get_cell(my_pt)
        print("I think I am in: " + my_cell)

        intermediate_pts = self.path_plan(my_cell, target_cell)

        print("Path Plan: " + str(intermediate_pts))

        self.reset_position()

        for pt in intermediate_pts:

            success = self.go_to_position(self.centers[pt], monitor_function)
            self.reset_position()
            if(not success):
                return False

        success = self.go_to_position(my_target, monitor_function)

        if (not success):
            return False

        return True

    def go_to_position(self, target, monitor_function, have_flag = False, allowed_error = 20, dwell_time = 1):
        '''
        Attempts to get Sphero to go to target position, existing out of loop as soon as it is within allowed error
        :param target:
        :param monitor_function:
        :param have_flag:
        :param allowed_error:
        :param dwell_time:
        :return:
        '''

        if(target is None):
            return False

        target_px = util.mm_2_pixel(target)

        print("Going to (" + str(target.x) + ", " + str(target.y) + ") mm, or (" + str(target_px.x) + ", " + str(target_px.y) + ") px")

        time_center = time.time()

        at_goal = False

        accumulated_error = 0

        rate = rospy.Rate(10)

        monitor_results = monitor_function()
        while not rospy.is_shutdown() and not monitor_results:

            if (self.my_position is None or target is None):
                rate.sleep()
                continue

            linear_error = util.calculate_distance(self.my_position.point, target)
            err_heading = util.calculate_error_heading(self.my_position.point, target, positive_only=True)

            if(linear_error is None or err_heading is None):
                rate.sleep()
                continue

            if(np.abs(linear_error) < allowed_error):
                accumulated_error = 0
                if(not at_goal):
                    #print("Touched Goal")
                    t = Twist()
                    t.linear = Vector3(0, 0, 0)
                    t.angular = Vector3(0, 0, err_heading)

                    if(not self.do_movement(t)):
                        print("Movement Failed, exiting")
                        return False

                    at_goal = True
                    time_center = time.time()
                    rate.sleep()
                    continue
                else:
                    secs = time.time() - time_center
                    if(secs > dwell_time):
                        print("Reached Target" + str(target.x) + ", " + str(target.y) + ") mm")
                        return True
                    else:
                        #print("Close to goal")
                        rate.sleep()
                        continue
            else:
                at_goal = False
                accumulated_error += linear_error

            # Control Parameters
            vel = linear_error * self.Kp

            if(self.my_velocity_mag is not None):
                vel = vel - self.Kd * self.my_velocity_mag

            if(vel > self.MAX_SPEED):
                vel = self.MAX_SPEED

            if(vel < 0):
                vel = 0

            t = Twist()
            t.linear = Vector3(vel, 0, 0)
            t.angular = Vector3(0, 0, err_heading)
            if(not self.do_movement(t)):
                print("Movement Failed, exiting")
                return False

            rate.sleep()
            monitor_results = monitor_function()
            if(monitor_results):
                print("Monitor tripped")
                return False

        return True


if(__name__ == "__main__"):
    b = SimpleAgent('red_sphero', opponent='blue_sphero')

    b.setup_ros()

    b.play_game()
