import rospy
from std_msgs.msg import Bool

pub_reset = rospy.Publisher('/arena/reset_game', Bool, queue_size=1)

rospy.init_node('arena_reset', anonymous=True)

pub_reset.publish(True)
