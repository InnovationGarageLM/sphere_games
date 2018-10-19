from sphero_sprk import Sphero
import rospy
from geometry_msgs.msg import Point, Twist, Vector3Stamped, PointStamped
from std_msgs.msg import Bool, Int16, ColorRGBA
import struct
import sys

class SpheroNode(object):

    VERSION = "1.0"

    def __init__(self, name = "sphero", color = None, addr=None):
        self._name = name
        self._addr = addr

        self._color = color

        self._orb = Sphero(addr)
        self._connected = False

        if (self._color is None):
            self._color = ColorRGBA(0, 128, 0, 0)
        elif(self._color == "blue"):
            self._color = ColorRGBA(0, 0, 128, 0)
        elif (self._color == "red"):
            self._color = ColorRGBA(128, 0, 0, 0)

    def connect(self):
        self._orb.connect()
        orb = self._orb.get_device_name()
        print("Connected to: " + orb['name'])
        rospy.loginfo("Connected to: " + orb['name'])

        version = self._orb.version()
        print("Model: " + str(version['MDL']) + " HW Version: "+str(version['HW']))
        print("App Version: " + str(version['MSA-ver']) + "." + str(version['MSA-rev']))
        print("Firmware Version: " + str(version['BL']))

        rospy.loginfo("Model: " + str(version['MDL']) + " HW Version: " + str(version['HW']))
        rospy.loginfo("App Version: " + str(version['MSA-ver']) + "." + str(version['MSA-rev']))
        rospy.loginfo("Firmware Version: " + str(version['BL']))

        self._connected = True

    def cmd_vel(self, twist):
        if(not self._connected):
            return

        self._orb.roll(int(twist.linear.x), int(twist.angular.z))

    def set_tail(self, brightness):
        if(not self._connected):
            return

        self._orb.set_tail_light(brightness.data)


    def set_stab(self, flag):
        if(not self._connected):
            return

        goal_state = flag.data

        print("Setting it to: " + str(goal_state))

        if(goal_state):
            print("Turning ON Stabilization")
        else:
            print("Turning OFF Stabilization")
        self._orb.set_stabilization(goal_state)

    def set_color(self, color):
        if(not self._connected):
            return

        self._orb.set_rgb_led(int(color.r), int(color.g), int(color.b))

    def reset_heading(self, heading):
        if (not self._connected):
            print(self._name + " is not connected)")
            return
        print("Reset Heading")
        self._orb.set_heading(0)

    def set_position(self, pt):
        '''
        Set Position of Sphero, in mm (automatically convert to cm here)
        :param pt:
        :return:
        '''
        if (not self._connected):
            print(self._name + " is not connected)")
            return

        print("Setting Position to: \n" + str(pt))

        x = int(pt.x/10)
        y = int(pt.y/10)

        self._orb.config_locator(x, y, 0)


    def set_heading(self, heading):
        if(not self._connected):
            print(self._name + " is not connected)")
            return
        print("Set Heading to: " + str(heading.data))
        self._orb.set_stabilization(False)

        self._orb.set_tail_light(255)
        rospy.sleep(2)
        self._orb.roll(1,heading.data)
        rospy.sleep(2)
        self._orb.set_heading(0)
        rospy.sleep(2)
        self._orb.set_tail_light(0)
        self._orb.set_stabilization(True)

    def init_ros(self):
        rospy.init_node(self._name, anonymous=True)

        topic_root = '/'+self._name

        # Subscriptions
        sub_cmd_vel = rospy.Subscriber(topic_root + '/cmd_vel', Twist, self.cmd_vel, queue_size=1)
        sub_color   = rospy.Subscriber(topic_root + '/set_color', ColorRGBA, self.set_color, queue_size=1)
        sub_tail    = rospy.Subscriber(topic_root + '/set_tail', Int16, self.set_tail, queue_size=1)
        sub_stab    = rospy.Subscriber(topic_root + '/set_stabilization', Bool, self.set_stab, queue_size=1)
        sub_heading = rospy.Subscriber(topic_root + '/set_heading', Int16, self.set_heading, queue_size=1)
        sub_heading = rospy.Subscriber(topic_root + '/reset_heading', Int16, self.reset_heading, queue_size=1)
        sub_set_pos = rospy.Subscriber(topic_root + '/set_position', Point, self.set_position, queue_size=1)

        # Publishables
        self.pub_odometry = rospy.Publisher(topic_root + '/odometry', PointStamped, queue_size=1)
        self.pub_velocity = rospy.Publisher(topic_root + '/velocity', PointStamped, queue_size=1)
        self.pub_accel    = rospy.Publisher(topic_root + '/accel', Int16, queue_size=1)
        #self.pub_accel_raw = rospy.Publisher(topic_root + '/accel', PointStamped, queue_size=1)
        #self.pub_imu = rospy.Publisher(topic_root + '/imu', PointStamped, queue_size=1)
        #self.pub_gyro = rospy.Publisher(topic_root + '/gyro', PointStamped, queue_size=1)

    def pub_accelone_data(self, data):
        val = struct.unpack('>h', data)
        self.pub_accel.publish(val[0])

    def pub_accel_raw_data(self, data):

        val = struct.unpack('>hhh', data)

        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.point.x = val[0]/4096 # assuming filtered values
        ps.point.y = val[1]/4096
        ps.point.z = val[2]/4096
        self.pub_accel.publish(ps)


    def pub_imu_data(self, data):
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.point.x = data['x']
        ps.point.y = data['y']
        ps.point.z = data['z']
        self.pub_imu.publish(ps)

    def pub_gyro_data(self, data):
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.point.x = data['x']
        ps.point.y = data['y']
        ps.point.z = data['z']
        self.pub_gyro.publish(ps)

    def pub_odom_data(self, data):
        '''
        Publish Odometry data in mm
        :param data:
        :return:
        '''
        if(len(data) == 0):
            return

        val = struct.unpack('>hh', data)

        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.point.x = val[0]*10 # assuming filtered values
        ps.point.y = val[1]*10

        self.pub_odometry.publish(ps)

    def pub_vel_data(self, data):
        if(len(data) == 0):
            return

        val = struct.unpack('>hh', data)

        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.point.x = val[0] # assuming filtered values
        ps.point.y = val[1]

        self.pub_velocity.publish(ps)

    def setup_publishables(self):
        #self._orb.start_accel_callback(rate=10, callback=self.pub_accel_raw_data)
        #self._orb.start_IMU_callback(rate=10, callback=self.pub_imu_data)
        #self._orb.start_gyro_callback(rate=10, callback=self.pub_gyro_data)
        self._orb.set_stream_callback('odometer', callback=self.pub_odom_data, mask_id=2)
        self._orb.set_stream_callback('accelone', callback=self.pub_accelone_data, mask_id=2)
        self._orb.set_stream_callback('velocity', callback=self.pub_vel_data, mask_id=2)

        self._orb.update_streaming(rate=10)

    def my_ping(self, last_call):
        self._orb.ping()

    def my_streaming(self, last_call):
        self._orb.update_streaming(rate=5)

    def info(self):
        print("Sphero Node v"+SpheroNode.VERSION)
        print("Node: "+self._name+", Color: ("+ str(self._color.r) + "," +
                                                str(self._color.g) + "," +
                                                str(self._color.b) + "," +
                                                str(self._color.a) + ")" )

    def main(self):
        self.info()
        self.connect()
        self.init_ros()

        rospy.sleep(2)

        self.setup_publishables()

        heartbeat = rospy.Timer(rospy.Duration(.05), self.my_ping)
        streaming = rospy.Timer(rospy.Duration(30), self.my_streaming)

        self.set_color(self._color)

        rate = rospy.Rate(40)  # Hz
        while not rospy.is_shutdown():
            rate.sleep()

if(__name__ == "__main__"):

    num_args = len(sys.argv)

    name = "sphero"
    color = "blue"
    #addr = "E7:D7:34:B0:4B:DB"
    #addr = "DC:79:91:5A:9B:92"
    addr = "D2:E6:0C:03:02:7D"

    if(num_args == 4):
        name = sys.argv[1]
        color = sys.argv[2]
        addr = sys.argv[3]

    s = SpheroNode(name=name, color=color, addr=addr)
    s.main()
