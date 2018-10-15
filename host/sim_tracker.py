import time
import copy

import numpy as np
import rospy
import cv2

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, PointStamped

# Capture the Flag base configuration
print("Starting Capture the Flag")

red_odometry = PointStamped()
blue_odometry = PointStamped()
red_acceleration = 0
blue_acceleration = 0
red_center = Point()
blue_center = Point()
red_pos_mm = PointStamped()
blue_pos_mm = PointStamped()
red_velocity = PointStamped()
blue_velocity = PointStamped()
red_base = Point(1400, 98, 0)
blue_base = Point(518, 979, 0)
red_base_mm = Point((red_base.x - 959) / 0.771, 
                 -1*(red_base.y - 538) / 0.771, 0) # Flip y-axis
blue_base_mm = Point((blue_base.x - 959) / 0.771, 
                  -1*(blue_base.y - 538) / 0.771, 0) # Flip y-axis
red_flag = False
blue_flag = False
red_score = 0
blue_score = 0

counter = 0

start = time.time()

def receive_image(image_data):
    global red_center, blue_center, red_pos_mm, blue_pos_mm, red_odometry
    global blue_odometry, red_velocity, blue_velocity, red_acceleration
    global blue_acceleration, counter

    image_array = np.fromstring(image_data.data, np.uint8)
    cv2_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    # Mask by hue and find center
    hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

    red_lower_1 = np.array([0, 50, 50])
    red_upper_1 = np.array([int(1.0 * 180. / 6.), 255, 255])
    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)

    red_lower_2 = np.array([int(5.5 * 180. / 6.), 50, 50])
    red_upper_2 = np.array([255, 255, 255])
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)

    red_mask = red_mask_1 + red_mask_2
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    blue_lower = np.array([int(3.0 * 180. / 6.), 50, 50])
    blue_upper = np.array([int(4.5 * 180. / 6.), 255, 255])
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    red_contours = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(red_contours) > 0:
        red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
        red_center = Point(int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']), 0)

    blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(blue_contours) > 0:
        blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
        blue_center = Point(int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']), 0)

    # Store previous positions for calculating velocity
    # and velocities for calculating accerlation
    red_pos_mm_old = copy.deepcopy(red_pos_mm)
    blue_pos_mm_old = copy.deepcopy(blue_pos_mm)
    red_vel_old = copy.deepcopy(red_velocity)
    blue_vel_old = copy.deepcopy(blue_velocity)

    # Convert camera pixel coordinates to millimeters.
    # Position (0, 0, 0) is at the center of the arena.
    red_pos_mm.header.stamp = rospy.Time.now()
    red_pos_mm.point = Point((red_center.x - 959) / 0.771, 
                          -1*(red_center.y - 538) / 0.771, 0) # Flip y-axis
    blue_pos_mm.header.stamp = rospy.Time.now()
    blue_pos_mm.point = Point((blue_center.x - 959) / 0.771, 
                           -1*(blue_center.y - 538) / 0.771, 0) # Flip y-axis

    # Calculate current deltas from starting positions, in millimeters
    # In the simulation this is simply the distance from their home base
    red_odometry.header.stamp = rospy.Time.now()
    # red_odometry.point = Point(red_pos_mm.point.x - red_base_mm.x, 
    #                            red_pos_mm.point.y - red_base_mm.y, 0)
    red_odometry.point = red_pos_mm.point
    blue_odometry.header.stamp = rospy.Time.now()
    # blue_odometry.point = Point(blue_pos_mm.point.x - blue_base_mm.x, 
    #                             blue_pos_mm.point.y - blue_base_mm.y, 0)
    blue_odometry.point = blue_pos_mm.point

    red_velocity.header.stamp = rospy.Time.now()
    delta_t = ((red_pos_mm.header.stamp.secs 
              + red_pos_mm.header.stamp.nsecs / 1000000000.) 
             - (red_pos_mm_old.header.stamp.secs + 
                red_pos_mm_old.header.stamp.nsecs / 1000000000.))
    if delta_t == 0:
        delta_t = 0.1
    red_velocity.point = Point((red_pos_mm.point.x 
                              - red_pos_mm_old.point.x) / delta_t, 
                               (red_pos_mm.point.y 
                              - red_pos_mm_old.point.y) / delta_t, 
                               (red_pos_mm.point.z 
                              - red_pos_mm_old.point.z) / delta_t)
    blue_velocity.header.stamp = rospy.Time.now()
    delta_t = ((blue_pos_mm.header.stamp.secs 
              + blue_pos_mm.header.stamp.nsecs / 1000000000.) 
             - (blue_pos_mm_old.header.stamp.secs + 
                blue_pos_mm_old.header.stamp.nsecs / 1000000000.))
    if delta_t == 0:
        delta_t = 0.1
    blue_velocity.point = Point((blue_pos_mm.point.x 
                               - blue_pos_mm_old.point.x) / delta_t, 
                                (blue_pos_mm.point.y 
                               - blue_pos_mm_old.point.y) / delta_t, 
                                (blue_pos_mm.point.z 
                               - blue_pos_mm_old.point.z) / delta_t)

    delta_t = ((red_velocity.header.stamp.secs 
              + red_velocity.header.stamp.nsecs / 1000000000.) 
             - (red_vel_old.header.stamp.secs + 
                red_vel_old.header.stamp.nsecs / 1000000000.))
    if delta_t == 0:
        delta_t = 0.1
    red_accel_vector = Point((red_velocity.point.x 
                            - red_vel_old.point.x) / delta_t,  
                             (red_velocity.point.y 
                            - red_vel_old.point.y) / delta_t,  
                             (red_velocity.point.z 
                            - red_vel_old.point.z) / delta_t)
    delta_t = ((blue_velocity.header.stamp.secs 
              + blue_velocity.header.stamp.nsecs / 1000000000.) 
             - (blue_vel_old.header.stamp.secs + 
                blue_vel_old.header.stamp.nsecs / 1000000000.))
    if delta_t == 0:
        delta_t = 0.1
    blue_accel_vector = Point((blue_velocity.point.x 
                             - blue_vel_old.point.x) / delta_t,  
                              (blue_velocity.point.y 
                             - blue_vel_old.point.y) / delta_t,  
                              (blue_velocity.point.z 
                             - blue_vel_old.point.z) / delta_t)
    red_acceleration = np.sqrt(red_accel_vector.x ** 2 
                             + red_accel_vector.y ** 2 
                             + red_accel_vector.z ** 2)
    blue_acceleration = np.sqrt(blue_accel_vector.x ** 2 
                              + blue_accel_vector.y ** 2 
                              + blue_accel_vector.z ** 2)

    counter += 1
    # if counter % 5 == 0:
    # cv2.imwrite('images/{0:08d}.png'.format(counter), cv2_image)
    # cv2.imshow('cv2_image', cv2_image)
    # cv2.waitKey(2)
    return

# Scoring logic
def host():
    global red_center, blue_center, red_base, blue_base
    global red_flag, blue_flag, red_score, blue_score
    red_at_away = False
    red_at_home = False
    blue_at_away = False
    blue_at_home = False

    if red_flag != False:
        distance = np.sqrt((red_center.x - red_base.x) ** 2 + 
                           (red_center.y - red_base.y) ** 2)
        if distance < 70:
            red_at_home = True
    else:
        distance = np.sqrt((red_center.x - blue_base.x) ** 2 + 
                           (red_center.y - blue_base.y) ** 2)
        if distance < 70:
            red_at_away = True

    if blue_flag != False:
        distance = np.sqrt((blue_center.x - blue_base.x) ** 2 + 
                           (blue_center.y - blue_base.y) ** 2)
        if distance < 70:
            blue_at_home = True
    else:
        distance = np.sqrt((blue_center.x - red_base.x) ** 2 + 
                           (blue_center.y - red_base.y) ** 2)
        if distance < 70:
            blue_at_away = True

    if red_at_home and blue_at_home:
        red_score += 1
        blue_score += 1
        red_flag = False
        blue_flag = False
    elif red_at_home:
        red_score += 1
        red_flag = False
        blue_flag = False
    elif blue_at_home:
        blue_score += 1
        red_flag = False
        blue_flag = False
    else:
        if red_at_away:
            red_flag = True
        if blue_at_away:
            blue_flag = True
    return

def pub_sub_init():
    pub_red_odom = rospy.Publisher('/red_sphero/odometry', PointStamped, queue_size=1)
    pub_blue_odom = rospy.Publisher('/blue_sphero/odometry', PointStamped, queue_size=1)
    pub_red_vel = rospy.Publisher('/red_sphero/velocity', PointStamped, queue_size=1)
    pub_blue_vel = rospy.Publisher('/blue_sphero/velocity', PointStamped, queue_size=1)
    pub_red_accel = rospy.Publisher('/red_sphero/accel', Int16, queue_size=1)
    pub_blue_accel = rospy.Publisher('/blue_sphero/accel', Int16, queue_size=1)
    pub_red_center = rospy.Publisher('/arena/red_sphero/center', Point, queue_size=1)
    pub_blue_center = rospy.Publisher('/arena/blue_sphero/center', Point, queue_size=1)
    pub_red_pos_mm = rospy.Publisher('/arena/red_sphero/center_mm', PointStamped, queue_size=1)
    pub_blue_pos_mm = rospy.Publisher('/arena/blue_sphero/center_mm', PointStamped, queue_size=1)
    pub_red_base = rospy.Publisher('/arena/red_sphero/base', Point, queue_size=1)
    pub_blue_base = rospy.Publisher('/arena/blue_sphero/base', Point, queue_size=1)
    pub_red_base_mm = rospy.Publisher('/arena/red_sphero/base_mm', Point, queue_size=1)
    pub_blue_base_mm = rospy.Publisher('/arena/blue_sphero/base_mm', Point, queue_size=1)
    pub_red_flag = rospy.Publisher('/arena/red_sphero/flag', Bool, queue_size=1)
    pub_blue_flag = rospy.Publisher('/arena/blue_sphero/flag', Bool, queue_size=1)
    pub_red_score = rospy.Publisher('/arena/red_sphero/score', Int16, queue_size=1)
    pub_blue_score = rospy.Publisher('/arena/blue_sphero/score', Int16, queue_size=1)

    pub_game_state = rospy.Publisher('/arena/game_state', Int16, queue_size=1)

    sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)
    pub_game_state.publish(0)

    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        host()

        pub_red_odom.publish(red_odometry)
        pub_blue_odom.publish(blue_odometry)
        pub_red_vel.publish(red_velocity)
        pub_blue_vel.publish(blue_velocity)
        pub_red_accel.publish(red_acceleration)
        pub_blue_accel.publish(blue_acceleration)
        pub_red_center.publish(red_center)
        pub_blue_center.publish(blue_center)
        pub_red_pos_mm.publish(red_pos_mm)
        pub_blue_pos_mm.publish(blue_pos_mm)
        pub_red_base.publish(red_base)
        pub_blue_base.publish(blue_base)
        pub_red_base_mm.publish(red_base_mm)
        pub_blue_base_mm.publish(blue_base_mm)
        pub_red_flag.publish(red_flag)
        pub_blue_flag.publish(blue_flag)
        pub_red_score.publish(red_score)
        pub_blue_score.publish(blue_score)
        pub_game_state.publish(1)

        print("Time: {} / 300".format(time.time() - start))
        print("Red: [{}, {}], [{}, {}]".format(red_center.x, red_center.y, 
            red_flag, red_score))
        print("Blue: [{}, {}], [{}, {}]".format(blue_center.x, blue_center.y, 
            blue_flag, blue_score))

        if time.time() - start > 300:
            pub_game_state.publish(2)
            break

        rate.sleep()

    pub_game_state.publish(2)
    if blue_score > red_score:
        print("Winner: Blue Team")
    elif blue_score < red_score:
        print("Winner: Red Team")
    else:
        print("Draw!")
    print("Final Score - Red: {}, Blue: {}".format(red_score, blue_score))
    return

if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

