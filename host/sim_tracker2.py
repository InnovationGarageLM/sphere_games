import constants
from host.base_tracker import BaseTracker
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Int16

import cv2
import rospy

import copy
import time
import numpy as np

class SimTracker(BaseTracker):


    def __init__(self, tracked_ids=['red_sphero', 'blue_sphero']):

        super(SimTracker, self).__init__(tracked_ids=tracked_ids)

        self.odometry = {}
        self.velocity = {}
        self.acceleration = {}

        self.odometry_old = {}
        self.velocity_old = {}

        zeroPoint = PointStamped()
        zeroPoint.header.stamp = rospy.Time()
        zeroPoint.point = Point(0,0,0)

        self.pub_odometry = {}
        self.pub_velocity = {}
        self.pub_accel = {}
        self.pub_mask = {}

        for color in self.ids:

            self.odometry[color] = zeroPoint
            self.odometry_old[color] = zeroPoint

            self.velocity[color] = zeroPoint
            self.velocity_old[color] = zeroPoint

            self.acceleration[color] = 0

    def init_subscribers(self):

        super(SimTracker, self).init_subscribers()

        sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage,
                                     self.process_frame, queue_size=1)

    def init_publishers(self):

        super(SimTracker, self).init_publishers()

        for id in self.ids:
            topic_root = '/' + id

            # Publishables
            self.pub_odometry[id] = rospy.Publisher(topic_root + '/odometry', PointStamped, queue_size=1)
            self.pub_velocity[id] = rospy.Publisher(topic_root + '/velocity', PointStamped, queue_size=1)
            self.pub_accel[id]    = rospy.Publisher(topic_root + '/accel', Int16, queue_size=1)

        for color in constants.COLOR_THRESHOLDS:
            self.pub_mask[color] = rospy.Publisher('/arena/' + color + '_mask', Image, queue_size=1)

    def pub_images(self):

        super(SimTracker, self).pub_images()

    def find_contours(self, mask):
        center = None

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contours) > 0:
            M = cv2.moments(max(contours, key=cv2.contourArea))
            center = Point(int(M['m10'] / M['m00']), int(M['m01'] / M['m00']), 0)

        return center

    def get_spheros(self, cv2_image):

        masks = self.get_masks(cv2_image)

        centers = {}
        for mask in masks:
            self.pub_mask[mask].publish(self.bridge.cv2_to_imgmsg(masks[mask], encoding="mono8"))
            centers[mask] = self.find_contours(masks[mask])

        return centers

    def calc_velocity(self, old_pt, new_pt):

        velocity = PointStamped()

        delta_t = ((new_pt.header.stamp.secs
                    + new_pt.header.stamp.nsecs / 1000000000.)
                   - (old_pt.header.stamp.secs +
                      old_pt.header.stamp.nsecs / 1000000000.))
        if delta_t == 0:
            delta_t = 0.1

        velocity.header.stamp = new_pt.header.stamp
        velocity.point = Point((new_pt.point.x
                                - old_pt.point.x) / delta_t,
                               (new_pt.point.y
                                - old_pt.point.y) / delta_t,
                               (new_pt.point.z
                                - old_pt.point.z) / delta_t)
        return velocity

    def calc_accel(self, old_vel, new_vel):
        delta_t = ((new_vel.header.stamp.secs
                    + new_vel.header.stamp.nsecs / 1000000000.)
                   - (old_vel.header.stamp.secs +
                      old_vel.header.stamp.nsecs / 1000000000.))
        if delta_t == 0:
            delta_t = 0.1

        accel_vector = Point((new_vel.point.x
                              - old_vel.point.x) / delta_t,
                             (new_vel.point.y
                              - old_vel.point.y) / delta_t,
                             (new_vel.point.z
                              - old_vel.point.z) / delta_t)

        acceleration = np.sqrt(accel_vector.x ** 2
                               + accel_vector.y ** 2
                               + accel_vector.z ** 2)

        return acceleration

    def update_sphero_info(self, positions, stamp):

        for key in positions:
            # Save old position and velocity
            self.odometry_old[key] = copy.deepcopy(self.center_mm[key])
            self.velocity_old[key] = copy.deepcopy(self.velocity[key])

        # Update current position and velocity
        self.update_locations(positions, stamp)

        for key in positions:
            # Save old position and velocity
            self.odometry[key] = self.center_mm[key]
            self.velocity[key] = self.calc_velocity(self.odometry_old[key], self.odometry[key])
            self.acceleration[key] = self.calc_accel(self.velocity_old[key], self.velocity[key])

    def pub_team_info(self, id):
        super(SimTracker, self).pub_team_info(id)

        self.pub_odometry[id].publish(self.odometry[id])
        self.pub_velocity[id].publish(self.velocity[id])
        self.pub_accel[id].publish(self.acceleration[id])

    def process_frame(self, image_data):
        '''
        Take a specific image and identify sphero locations
        :return:
        '''
        stamp = image_data.header.stamp

        cv2_img = self.bridge.compressed_imgmsg_to_cv2(image_data, desired_encoding="passthrough")

        spheros = self.get_spheros(cv2_img)

        self.update_sphero_info(spheros, stamp)

        super(SimTracker, self).process_frame(image_data)


if(__name__ == "__main__"):
    t = SimTracker()
    t.init_ros()
    t.start_tracking()
