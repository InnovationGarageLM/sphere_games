import unittest
from base_tracker import BaseTracker
import constants
from geometry_msgs.msg import Point, PointStamped

class BaseTrackerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        constants.BASE['red'] = Point(200,200,0)
        constants.BASE['blue'] = Point(400, 400, 0)
        constants.BASE['green'] = Point(100, 500, 0)

    def test_update_scoring_init(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)


    def test_update_scoring_get_red_gets_flag(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.center['red'] = constants.BASE['blue']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], True)
        self.assertEqual(b.flag['blue'], False)

    def test_update_scoring_get_green_gets_flag(self):

        b = BaseTracker(tracked_ids=['red','blue','green'])

        b.center['green'] = constants.BASE['blue']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)
        self.assertEqual(b.flag['green'], True)

    def test_update_scoring_get_blue_gets_flag(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.center['blue'] = constants.BASE['red']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], True)

    def test_update_scoring_get_red_and_blue_gets_flag(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.center['red'] = constants.BASE['blue']
        b.center['blue'] = constants.BASE['red']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], True)
        self.assertEqual(b.flag['blue'], True)


    def test_update_scoring_red_flag(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['red'] = True

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], True)
        self.assertEqual(b.flag['blue'], False)

    def test_update_scoring_red_score(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['red'] = True
        b.center['red'] = constants.BASE['red']

        b.update_scoring()

        self.assertEqual(b.score['red'], 1)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)

    def test_update_scoring_red_score2(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['red'] = True
        b.score['red'] = 5
        b.center['red'] = constants.BASE['red']

        b.update_scoring()

        self.assertEqual(b.score['red'], 6)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)


    def test_update_scoring_red_score_neg_test(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['red'] = True
        b.center['red'] = constants.BASE['blue']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], True)
        self.assertEqual(b.flag['blue'], False)


    def test_update_scoring_blue_score(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['blue'] = True
        b.center['blue'] = constants.BASE['blue']

        b.update_scoring()

        self.assertEqual(b.score['red'], 0)
        self.assertEqual(b.score['blue'], 1)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)

    def test_update_scoring_red_score_reset_flags(self):

        b = BaseTracker(tracked_ids=['red','blue'])

        b.flag['red'] = True
        b.flag['blue'] = True
        b.center['red'] = constants.BASE['red']

        b.update_scoring()

        self.assertEqual(b.score['red'], 1)
        self.assertEqual(b.score['blue'], 0)

        self.assertEqual(b.flag['red'], False)
        self.assertEqual(b.flag['blue'], False)

if __name__ == '__main__':
    unittest.main()
