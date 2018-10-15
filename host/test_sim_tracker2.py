import unittest
from sim_tracker2 import SimTracker
from geometry_msgs.msg import Point, PointStamped
import rospy
import copy

class SimTrackerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    def test_calc_velocity(self):
        s = SimTracker()
        old_pt = PointStamped()
        old_pt.header.stamp = rospy.Time()
        old_pt.point = Point(0,0,0)


        new_time = copy.deepcopy(old_pt.header.stamp)
        new_time.secs = new_time.secs + 2

        new_pt = PointStamped()
        new_pt.header.stamp = new_time
        new_pt.point = Point(10, 0, 0)

        vel = s.calc_velocity(old_pt, new_pt)

        self.assertEqual(vel.point.x, 5.)
        self.assertEqual(vel.point.y, 0.)
        self.assertEqual(vel.point.z, 0.)


if __name__ == '__main__':
    unittest.main()
