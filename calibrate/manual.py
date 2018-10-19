
# Heavily altered copy of Alex's manual_calibration
# This allows you to exercise all the available actions

from __future__ import print_function

import sys
from getkey import getkey, keys

import rospy
from std_msgs.msg import Int16, Bool, ColorRGBA
from geometry_msgs.msg import Point, Twist, Vector3, PointStamped

class Mover:
    def __init__(self,color):
        self._color = color
        self._direct = self._speed = 0
        self._tail=0
        self._stab=False
        self._tracker_estimate = None

    def set_tracker_estimate(self, position):
        self._tracker_estimate = position

    def ros_setup(self):
        topic = '/'+self._color+'_sphero'
        self.pub_cmd_vel = rospy.Publisher(topic + '/cmd_vel', Twist, queue_size=1)
        self.pub_color   = rospy.Publisher(topic + '/set_color', ColorRGBA, queue_size=1)
        self.pub_tail    = rospy.Publisher(topic + '/set_tail', Int16, queue_size=1)
        self.pub_stab    = rospy.Publisher(topic + '/set_stabilization', Bool, queue_size=1)
        self.pub_heading = rospy.Publisher(topic + '/set_heading', Int16, queue_size=1)
        self.pub_reset   = rospy.Publisher(topic + '/reset_heading', Int16, queue_size=1)
        self.pub_set_pos = rospy.Publisher(topic + '/set_position', Point, queue_size=1)

        self.sub_center = rospy.Subscriber('/arena' + topic + '/center_mm', PointStamped, self.set_tracker_estimate,
                                           queue_size=1)

        # Available for subscription
        #self.sub_odometry = rospy.Subscriber(topic + '/odometry', PointStamped, cb, queue_size=1)
        #self.sub_velocity = rospy.Subscriber(topic + '/velocity', PointStamped, cb, queue_size=1)
        #self.sub_accel    = rospy.Subscriber(topic + '/accel', Int16, cb, queue_size=1)

    def send_twist(self):
        twist = Twist() 
        twist.linear = Vector3(self._speed, 0, 0)
        twist.angular = Vector3(0,0,self._direct)
        print("Speed %d, direction %d" % (self._speed,self._direct))
        self.pub_cmd_vel.publish(twist)

    def speed(self, incr): 
        self._speed += incr
        if self._speed<0: self._speed=0
        print("Speed now",self._speed)
        self.send_twist()

    def direct(self, incr):
        self._direct += incr; self._direct %= 360
        print("Direction now",self._direct)
        self.send_twist()

    def color(self):
        pass #need to have a set through which to cycle; not yet implemented

    def tail(self):
        self._tail += 50
        if self._tail>255: self._tail=0
        print("Tail brightness",self._tail)
        self.pub_tail.publish(Int16(self._tail))

    def stabilize(self):
        self._stab = not self._stab
        print("Stabilize "+("ON" if self._stab else "OFF"))
        self.pub_stab.publish(Bool(self._stab))

    def sethead(self):
        print("Set heading to",self._direct,"; also resets tail and stablize")
        self.pub_heading.publish(Int16(self._direct))
        self._stab=True; self._tail=0  #resets both

    def reset(self):
        print("Reset heading")
        self.pub_reset.publish(Int16(0)) # value ignored
        self._direct = self._speed = 0

    def setpos(self):
        '''
        Sets position to current tracker estimate (assuming not None)
        :return:
        '''
        pt = self._tracker_estimate.point
        pt.x = int(pt.x)
        pt.y = int(pt.y)
        print("Setting position to (" + str(pt.x) + ", " + str(pt.y) + ")")
        self.pub_set_pos.publish(Point(pt.x,pt.y,0))  #value in mm

#end
##############################################################################

def manual_control():

    # Setup ROS message handling
    rospy.init_node('manual', anonymous=True)

    # Pass command line argument 'r' for red, otherwise blue
    who = "red" if len(sys.argv) > 1 and sys.argv[1][0].lower() == 'r' else "blue"

    print("Controlling "+who+" sphero")
    mover = Mover(who)
    mover.ros_setup()

    # Agent control loop
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        key = getkey()
        if key == 'q': break  #quit
        elif key == keys.UP: mover.speed(+1)
        elif key == keys.DOWN: mover.speed(-1)
        elif key == keys.LEFT: mover.direct(-22)
        elif key == keys.RIGHT: mover.direct(+22)
        elif key == 't': mover.tail()
        elif key == 's': mover.stabilize()
        elif key == 'h': mover.sethead()
        elif key == 'r': mover.reset()
        elif key == 'p': mover.setpos()
        elif key == keys.SPACE: mover.send_twist()
        else:
            print("UP/DN-speed, LEFT/RGHT-direction, t-tail, s-stablize, h-heading, "\
                  "p-position, r-reset, q-quit")

        rate.sleep()
    return

if __name__ == '__main__':
    try:
        manual_control()
    except rospy.ROSInterruptException:
        pass

#end
