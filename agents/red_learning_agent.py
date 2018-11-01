import os

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Twist, Point, PointStamped, Vector3
import host.utilities as util

# Global variables
red_center = Point()
red_flag = False
red_base = Point()
blue_base = Point()
game_over = False
game_state = 0

red_twist = Twist()
Q_table = {}
yaw_actions = np.array(list(range(8))) * np.pi / 4
vel_actions = np.array(list(range(1, 2))) * 25 # One speed

# Helper functions
def set_center(sphere_center):
    global red_center
    # red_center = util.mm_2_pixel(sphere_center.point)
    red_center = sphere_center
    return

def set_flag(flag_status):
    global red_flag
    red_flag = flag_status.data
    return

def set_game_over(game_state):
    global game_over
    game_over = game_state.data
    return


def set_game_state(state):
    global game_over, game_state
    if(state.data == 2):
        game_over = True
        game_state = 2
    else:
        game_state = state.data

def set_blue_base(base):
    global blue_base
    blue_base = base
    return

def set_red_base(base):
    global red_base
    red_base = base
    return

def yaw_vel_to_twist(yaw, vel):
    twist_msg = Twist()
    twist_msg.linear = Vector3(vel, 0, 0)
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = (1080 + np.rad2deg(yaw))%360
    return twist_msg

def parse_dict(unformatted):
    formatted = {}
    for key in unformatted.item().keys():
        formatted[key] = unformatted.item().get(key)
    return formatted

def get_heading_and_distance():
    global red_center, red_flag, red_base, blue_base
    # print(red_flag)
    if red_flag != False: # Have flag, go home
        # target = util.pixels_2_mm(red_base)
        target = red_base
    else: # Don't have flag, go to opponent's base
        # target = util.pixels_2_mm(blue_base)
        target = blue_base
    delta_x = target.x - red_center.x
    delta_y = target.y - red_center.y
    # print("tx: {}, ty: {}, cx: {}, cy: {}".format(target.x, target.y, 
    #                                               red_center.x, red_center.y))
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance, delta_x, delta_y

# Agent function
def Q_learning():
    global Q_table, red_twist, yaw_actions, vel_actions
    expectation = 0.

    heading, distance, dx, dy = get_heading_and_distance()
    # print(dx, dy)
    current_value = 1 - distance / 1250. # Scale to [1, ~0]
    # heading = int(4 * heading / np.pi)   # Convert to range(8)
    # distance = int(8 * distance / 1250.)  # Convert to range(8)

    # Bin dx and dy
    bins = 17 # Technically there will be bins * 2 + 1 values
    dx = int(np.round(bins * dx / 878.))
    dy = int(np.round(bins * dy / 878.))
    # print(dx, dy)

    # Assure values don't go out of desired range
    if dx > bins:
        dx = bins
    if dx < -bins:
        dx = -bins
    if dy > bins:
        dy = bins
    if dy < -bins:
        dy = -bins

    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value)
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] = (reward 
            + Q_table[previous_grid][previous_choice]) / 2
        print("Distance: {}, Reward: {}".format(distance, reward))

    mode = 'test' # 'train'
    if mode == 'train':
        chance = 0.5
    else:
        chance = 0.0

    if (np.random.random() < chance 
        or (dx, dy) not in Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = Q_table[(dx, dy)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = Q_table[(dx, dy)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0.01:
            print(highest_value)
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (dx, dy) not in Q_table:
        Q_table[(dx, dy)] = {}
    if (yaw_choice, vel_choice) not in Q_table[(dx, dy)]:
        Q_table[(dx, dy)][(yaw_choice, vel_choice)] = 0.
    Q_table['previous_value'] = current_value
    Q_table['previous_grid'] = (dx, dy)
    Q_table['previous_choice'] = (yaw_choice, vel_choice)

    print("Yaw: {}, Vel: {}, Value: {}".format(yaw_choice, vel_choice,
        current_value))
    yaw_choice = -yaw_choice # Switch from camera to world coordinates
    red_twist = yaw_vel_to_twist(yaw_choice, vel_choice)
    return

# Init function
def learning_agent():
    # Load any existing agent
    global Q_table, game_over, game_state
    agent_file = 'chris_17_agent.npy'
    if os.path.isfile(agent_file):
        Q_table = parse_dict(np.load(agent_file))
        print("Loaded red agent from file.")
    else:
        print("New agent started.")

    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    pub_red_cmd = rospy.Publisher('/red_sphero/cmd_vel', Twist, queue_size=1)
    sub_red_center = rospy.Subscriber('/arena/red_sphero/center', Point, set_center, queue_size=1)
    sub_red_flag = rospy.Subscriber('/arena/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/arena/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/arena/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_state = rospy.Subscriber('/arena/game_state', Int16, set_game_state, queue_size=1)

    start_msg_shown = False
    game_start_msg_shown = False
    game_end_msg_shown = False

    # Agent control loop
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():

        if (game_state == 0):  # Waiting for game to start
            if (not start_msg_shown):
                print("Waiting for game to start...")

                start_msg_shown = True
                game_start_msg_shown = False
                game_end_msg_shown = False
            pass
        elif (game_state == 1):  # Game Active
            if (not game_start_msg_shown):
                print("Starting Game...")
                start_msg_shown = False
                game_start_msg_shown = True
                game_end_msg_shown = False

            Q_learning()
            pub_red_cmd.publish(red_twist)

        elif (game_state == 2):  # Game Over
            if (not game_end_msg_shown):
                print("Game Ended")
                start_msg_shown = False
                game_start_msg_shown = False
                game_end_msg_shown = True
                np.save(agent_file, Q_table)
                print("Game ended. Agent saved.")

        elif (game_state == 3):  # Test Mode
            if (not game_start_msg_shown):
                print("Entering Test Mode...")
                start_msg_shown = False
                game_start_msg_shown = True
                game_end_msg_shown = False
            Q_learning()
            pub_red_cmd.publish(red_twist)

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        learning_agent()
    except rospy.ROSInterruptException:
        pass

