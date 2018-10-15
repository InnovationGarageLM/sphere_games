import rospy
from std_msgs.msg import Bool, Int16, ColorRGBA
from geometry_msgs.msg import Point, Twist, Vector3, PointStamped
import sys
import socket
import rosgraph

import utilities as util


class ArenaSetup(object):

    def __init__(self):

        # Member Variables
        self.arena_center = {}

        # Subscriptions
        self.sub_arena_centers = {}

        # Publications
        self.pub_sphero_cmd_vel = {}
        self.pub_sphero_set_heading = {}
        self.pub_sphero_reset_heading = {}
        self.pub_sphero_set_color = {}
        self.pub_sphero_set_tail = {}
        self.pub_sphero_set_stab = {}
        self.pub_sphero_set_position = {}

        self.TIME_TO_MOVE = 3
        self.CAL_RES_ANGLE = 5 # Angle to turn sphero when determining heading
        self.TOLERANCE = 10 # degrees
        self.SETTLE_TIME = 10

        self.robot_list = [
            {'name': 'red_sphero' , 'color': ColorRGBA(128,0,0,0)},
            {'name': 'blue_sphero', 'color': ColorRGBA(0, 0, 128, 0)},
            #{'name': 'sphero', 'color': ColorRGBA(0, 0, 128, 0)},
        ]

        self.ready = False

    def set_arena_center(self, current_point, robot):
        self.arena_center[robot] = current_point
        self.ready = True

    def set_robot_center(self, current_point, robot):
        self.arena_center[robot] = current_point

    def setup_ros(self):

        try:
            rosgraph.Master('/rostopic').getPid()
        except socket.error:
            raise rospy.ROSTopicIOException("Unable to communicate with master!")

        rospy.init_node('arena_setup', anonymous=True)

        for robot in self.robot_list:
            self.setup_robot(robot['name'])



    def setup_robot(self, name):

        # Subsciptions
        self.sub_arena_centers[name] = rospy.Subscriber('/arena/' + name + '/center_mm', PointStamped,
                                                  self.set_arena_center, callback_args=name, queue_size=1)

        # Publications
        self.pub_sphero_cmd_vel[name] = rospy.Publisher('/'+name+'/cmd_vel', Twist, queue_size=1)
        self.pub_sphero_set_heading[name] = rospy.Publisher('/'+name+'/set_heading', Int16, queue_size=1)
        self.pub_sphero_reset_heading[name] = rospy.Publisher('/' + name + '/reset_heading', Int16, queue_size=1)
        self.pub_sphero_set_color[name] = rospy.Publisher('/'+name+'/set_color', ColorRGBA, queue_size=1)
        self.pub_sphero_set_tail[name] = rospy.Publisher('/' + name + '/set_tail', Int16, queue_size=1)
        self.pub_sphero_set_stab[name] = rospy.Publisher('/' + name + '/set_stabilization', Bool, queue_size=1)
        self.pub_sphero_set_position[name] = rospy.Publisher('/' + name + '/set_position', Point, queue_size=1)



    def reset_heading(self):
        twist_msg = Twist()
        twist_msg.linear = Vector3(0, 0, -1)
        twist_msg.angular = Vector3(0, 0, 0)
        return twist_msg

    def yaw_vel_to_twist(self,yaw, vel):
        twist_msg = Twist()
        twist_msg.linear = Vector3(vel, 0, 0)
        twist_msg.angular.z = yaw
        return twist_msg

    def turn_sphero(self,color, angle=90):
        self.pub_sphero_cmd_vel[color].publish(self.yaw_vel_to_twist(angle, self.MOVE_VEL))

        rospy.sleep(self.SETTLE_TIME)

        cmd = self.reset_heading()

        self.pub_sphero_cmd_vel[color].publish(cmd)
        print("Reset Heading")

    def set_angle(self, color, angle):
        t = Twist()
        t.linear = Vector3(0, 0, 0)
        t.angular = Vector3(0, 0, angle)
        self.pub_sphero_cmd_vel[color].publish(t)
        rospy.sleep(self.TIME_TO_MOVE)

    def roll(self, color, speed, heading, time = 5):
        t = Twist()
        t.linear = Vector3(speed, 0, 0)
        t.angular = Vector3(0, 0, heading)
        self.pub_sphero_cmd_vel[color].publish(t)
        rospy.sleep(time)

        t = Twist()
        t.linear = Vector3(0, 0, 0)
        t.angular = Vector3(0, 0, 0)
        self.pub_sphero_cmd_vel[color].publish(t)

    def calibrate_sphero(self, robot):
        print("Calibrating "+robot)

        self.pub_sphero_set_tail[robot].publish(255)
        self.set_angle(robot, 0)

        # Get Robot initial position
        start = self.arena_center[robot]

        # Move robot slightly
        self.roll(robot, 20, 0)

        # Get updated position
        end = self.arena_center[robot]

        # calculate heading difference
        error_deg = int(util.calculate_error_heading(start.point, end.point, positive_only=True))

        print(robot + " error degrees: "+str(error_deg))

        # Set new heading
        self.pub_sphero_set_heading[robot].publish(error_deg)

        self.pub_sphero_set_tail[robot].publish(0)
        print("Done Calibrating " + robot)

    def stop_sphero(self,color):
        self.pub_sphero_cmd_vel[color].publish(self.yaw_vel_to_twist(0, 0))

    def info_messages(self):
        print("Arena Setup v0.3")
        print("Please Verify that you are ready to start")

        ready = False
        if(not ready):
            if sys.version_info[0] == 3:
                response = str(input("Ready? (Y/N):")).lower()
            else:
                response = raw_input("Ready? (Y/N):").lower()

            if(response == "y" or response == "yes"):
                ready = True

    def set_position(self):
        print "Resetting Robot Locations, Verify tracker is correctly centering robots"

        ready = False
        if (not ready):
            if sys.version_info[0] == 3:
                response = str(input("Ready? (Y/N):")).lower()
            else:
                response = raw_input("Ready? (Y/N):").lower()

            if (response == "y" or response == "yes"):
                ready = True

        for robot in self.robot_list:
            curr_position = self.arena_center[robot['name']]

            self.pub_sphero_set_position[robot['name']].publish(curr_position.point)



    def start_setup(self, manual = False):
        '''
        Setup
        :return:
        '''
        # Info Messages
        self.info_messages()

        # Verify ROS is "running" in the arena
        self.setup_ros()

        rate = rospy.Rate(40)  # Hz
        while(not rospy.is_shutdown() and not self.ready):
            rate.sleep()

        rospy.sleep(2)

        if(manual):
            for robot in self.robot_list:
                self.pub_sphero_set_tail[robot['name']].publish(255)
                self.pub_sphero_set_stab[robot['name']].publish(False)
                rospy.sleep(2)

            if sys.version_info[0] == 3:
                response = str(input("Hit Return once ALL robots are positioned")).lower()
            else:
                response = raw_input("Hit Return once ALL robots are positioned").lower()

            for robot in self.robot_list:
                self.pub_sphero_reset_heading[robot['name']].publish(0)
                self.pub_sphero_set_stab[robot['name']].publish(True)
                self.pub_sphero_set_tail[robot['name']].publish(0)
                rospy.sleep(2)

        else:
            for robot in self.robot_list:
                self.calibrate_sphero(robot['name'])

        self.set_position()

        print("Done")

        pass


if(__name__ == "__main__"):

    manual = False

    if(len(sys.argv) == 2 and sys.argv[1] == "-m"):
        print("Using Manual Control")
        manual = True


    a = ArenaSetup()
    a.start_setup(manual)
