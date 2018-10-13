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
    red_center = util.mm_2_pixel(sphere_center.point)
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
    if red_flag != False: # Have flag, go home
        target_x = red_base.x
        target_y = red_base.y
    else: # Don't have flag, go to opponent's base
        target_x = blue_base.x
        target_y = blue_base.y
    delta_x = target_x - red_center.x
    delta_y = target_y - red_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

# Agent function
def Q_learning():
    global Q_table, red_twist, yaw_actions, vel_actions
    expectation = 0.

    heading, distance = get_heading_and_distance()
    current_value = 1 - distance / 1250. # Scale to [1, ~0]
    heading = int(4 * heading / np.pi)   # Convert to range(8)
    distance = int(8 * distance / 1250.)  # Convert to range(8)

    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.005
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] += reward

    if (np.random.random() < 0.2
        or (heading, distance) not in Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = Q_table[(heading, distance)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = Q_table[(heading, distance)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            print(highest_value)
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (heading, distance) not in Q_table:
        Q_table[(heading, distance)] = {}
    if (yaw_choice, vel_choice) not in Q_table[(heading, distance)]:
        Q_table[(heading, distance)][(yaw_choice, vel_choice)] = 0.
    Q_table['previous_value'] = current_value
    Q_table['previous_grid'] = (heading, distance)
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
    agent_file = 'test_agent.npy'
    if os.path.isfile(agent_file):
        Q_table = parse_dict(np.load(agent_file))
        print("Loaded red agent from file.")
    else:
        print("New agent started.")

    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    pub_red_cmd = rospy.Publisher('/red_sphero/cmd_vel', Twist, queue_size=1)
    sub_red_center = rospy.Subscriber('/red_sphero/odometry', PointStamped, set_center, queue_size=1)
    sub_red_flag = rospy.Subscriber('/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/arena/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/arena/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_state = rospy.Subscriber('/arena/game_state', Int16, set_game_state, queue_size=1)

    start_msg_shown = False
    game_start_msg_shown = False
    game_end_msg_shown = False

    # Agent control loop
    rate = rospy.Rate(5) # Hz
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

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        learning_agent()
    except rospy.ROSInterruptException:
        pass

