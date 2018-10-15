import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3
import host.utilities as util
import time
from random import randint, choice

class simple_agent(object):

    def __init__(self, name, opponent):

        # Class Variables
        self.name = name
        self.arena_position = None

        self.my_position = None
        self.my_velocity = None
        self.my_accel = 0

        self.flag = False
        self.game_state = None
        self.game_over = None

        self.my_base = None
        self.their_base = None

        self.my_base_px = None
        self.their_base_px = None

        self.last_position = None
        self.start_time = time.time()

        # Control Parameters
        self.Kp = .1;
        self.Ki = .001;
        self.Kd = .1;

        self.cmd_vel = None
        self.MAX_SPEED = 30

        self.stuck = None

        self.opponent = opponent

    def set_arena_position(self, pt):
        self.arena_position = pt

    def set_my_position(self, pt):
        self.my_position = pt

    def set_flag(self, flag):
        self.flag = flag

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

    def set_accel(self, accel):
        self.my_accel = accel.data

    def setup_ros(self):

        rospy.init_node(str(self.name) + '_simple_agent', anonymous=True)

        prefix = '/' + self.name
        arena_prefix = '/arena/' + self.name

        self.pub_cmd_vel      = rospy.Publisher(prefix + '/cmd_vel', Twist, queue_size=1)

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
        if(self.game_state !=1):
            return True

    def return_false(self):
        return False

    def check_if_stuck(self):
        if(self.cmd_vel is None):
            return False

        if(self.cmd_vel.linear.x > 0):
            if(self.my_velocity.point.x == 0 and self.my_velocity.point.y == 0):
                curr_time = time.time()
                if(self.stuck is None):
                    self.stuck = curr_time
                elif(curr_time - self.stuck > 3):
                    self.stuck = None
                    return True

        return False

    def do_movement(self, t):
        self.cmd_vel = t
        self.pub_cmd_vel.publish(t)

        if(self.check_if_stuck()):
            direction = choice([0, 90, 180, 270])

            print("Stuck! Trying to get unstuck, heading: " + str(direction))

            t = Twist()
            t.linear = Vector3(50,0,0)
            t.angular = Vector3(0,0,direction)

            self.cmd_vel = t
            self.pub_cmd_vel.publish(t)

            rospy.sleep(2)

    def test_game(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if(not self.their_base is None and not self.my_base is None):
                rate.sleep()
                continue

            self.go_to_position(self.their_base, self.their_base_px, self.return_false, have_flag=False)
            self.go_to_position(self.my_base, self.my_base_px, self.return_false, have_flag=True)
            rate.sleep()


    def play_game(self):

        while not rospy.is_shutdown():
            if(not self.their_base is None and not self.my_base is None and not self.game_state is None):
                break

        start_msg_shown = False
        game_start_msg_shown = False
        game_end_msg_shown = False

        rate = rospy.Rate(2)  # Hz

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

                self.go_to_their_base()
                self.go_to_my_base()
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
                self.go_to_their_base()
                self.go_to_my_base()

            rate.sleep()

    def go_to_my_base(self):

        my_base = self.my_base
        my_base_px = self.my_base_px
        my_base = self.my_base

        p1 = Point(0,-300, 0)
        p1_px = util.mm_2_pixel(p1)

        p2 = Point(300, 0, 0)
        p2_px = util.mm_2_pixel(p2)

        if(my_base.x > my_base.x):
            self.go_to_position(p1, p1_px, self.end_early)
            self.go_to_position(p2, p2_px, self.end_early)

        else:
            self.go_to_position(p2, p2_px, self.end_early)
            self.go_to_position(p1, p1_px, self.end_early)

        self.go_to_position(my_base, my_base_px, self.end_early)


    def go_to_their_base(self):

        their_base = self.their_base
        their_base_px = self.their_base_px
        my_base = self.my_base

        p1 = Point(0,their_base.y/2, 0)
        p1_px = util.mm_2_pixel(p1)

        p2 = Point(their_base.x/2, 0, 0)
        p2_px = util.mm_2_pixel(p2)

        if(their_base.x > my_base.x):
            self.go_to_position(p1, p1_px, self.end_early)
            self.go_to_position(p2, p2_px, self.end_early)

        else:
            self.go_to_position(p2, p2_px, self.end_early)
            self.go_to_position(p1, p1_px, self.end_early)

        self.go_to_position(their_base, their_base_px, self.end_early)

    def go_to_position(self, target, target_px, monitor_function, have_flag = False, allowed_error = 20, dwell_time = 2):
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

        print("Target Location: (" + str(target.x) + ", " + str(target.y) + ")")
        print("Target Pixels  : (" + str(target_px.x) + ", " + str(target_px.y) + ")")

        time_center = time.time()

        at_goal = False

        accumulated_error = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not monitor_function():

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

                    self.do_movement(t)

                    at_goal = True
                    time_center = time.time()
                    rate.sleep()
                    continue
                else:
                    secs = time.time() - time_center
                    if(secs > dwell_time):
                        print("Found goal")
                        return
                    else:
                        #print("Close to goal")
                        rate.sleep()
                        continue
            else:
                at_goal = False
                accumulated_error += linear_error

            # Control Parameters
            vel = linear_error * self.Kp #+ accumulated_error*self.Ki

            if(vel > self.MAX_SPEED):
                vel = self.MAX_SPEED

            t = Twist()
            t.linear = Vector3(vel, 0, 0)
            t.angular = Vector3(0, 0, err_heading)
            self.do_movement(t)

            rate.sleep()

        return True


if(__name__ == "__main__"):
    b = simple_agent('red_sphero', opponent='blue_sphero')

    b.setup_ros()

    #b.test_game()

    b.play_game()
