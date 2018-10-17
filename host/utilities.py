import cv2
import constants

import numpy as np
from geometry_msgs.msg import Point

def pixels_2_mm(pt_px):
    x = (pt_px.x - constants.ORIGIN_PIXELS.x) * constants.COVERT_PIXEL2MM
    y = -1 * ((pt_px.y - constants.ORIGIN_PIXELS.y) * constants.COVERT_PIXEL2MM)
    z = (constants.ORIGIN_PIXELS.z - pt_px.z) * constants.COVERT_PIXEL2MM

    return Point(int(x),int(y),int(z))

def mm_2_pixel(pt_mm):
    x = constants.ORIGIN_PIXELS.x + (pt_mm.x * constants.COVERT_MM2PIXEL)
    y = constants.ORIGIN_PIXELS.y - (pt_mm.y * constants.COVERT_MM2PIXEL)
    z = constants.ORIGIN_PIXELS.z + (pt_mm.z * constants.COVERT_MM2PIXEL)

    return Point(int(x),int(y),int(z))

def calculate_distance(start, end):
    if(start is None):
        return

    if(end is None):
        return

    distance = np.sqrt((start.x - end.x) ** 2 +
                       (start.y - end.y) ** 2)

    return distance

def calculate_error_heading(start, end, positive_only=False):

    if(start is None):
        return

    if(end is None):
        return

    x = end.x - start.x
    y = end.y - start.y

    theta = np.arctan2(y,x)

    angle = 90 - np.rad2deg(theta) # Rotate so heading is from true north

    if(angle > 180):
        angle = angle - 360
    elif(angle < -180):
        angle = angle + 360

    if(positive_only):
        angle = (720 + angle)%360

    return angle

def update_arena(game_state, time_elapsed, score, center, base, flag, img):
    # Write some Text

    arena_img = img.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10, 40)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2

    cv2.putText(arena_img, 'LM Autonomy Hackathon',
                bottomLeftCornerOfText,
                font,
                fontScale,
                fontColor,
                lineType)

    # Game State
    if (game_state == 0):
        game_text = "Waiting"
    elif (game_state == 1):
        game_text = "Running"
    elif (game_state == 2):
        game_text = "Game Over"
    elif (game_state == 3):
        game_text = "Test Mode"
    else:
        game_text = "ERROR"

    cv2.putText(arena_img, 'Status: ' + game_text,
                (10, 870),
                font,
                fontScale,
                fontColor,
                lineType)

    if(time_elapsed is None):
        time_elapsed = 0

    # Valid Game Area
    cv2.rectangle(arena_img,
                  (constants.ARENA_BOUNDS['left'], constants.ARENA_BOUNDS['top']),
                  (constants.ARENA_BOUNDS['right'], constants.ARENA_BOUNDS['bottom']),
                  (255, 255, 255))
    cv2.rectangle(arena_img,
                  (constants.ARENA_BOUNDS['left'], constants.ARENA_BOUNDS['top']),
                  (constants.ARENA_BOUNDS['right'], constants.ARENA_BOUNDS['bottom']),
                  (0, 255, 0))


    # Time Remaining
    cv2.putText(arena_img, 'Time Remaining: ' + str(constants.TOTAL_ALLOWED_TIME - time_elapsed) + ' s',
                (10, 910),
                font,
                fontScale,
                fontColor,
                lineType)

    if(center['red_sphero'] is not None):
        red_position = 'Red (' + str(int(center['red_sphero'].x)) + ', ' + str(int(center['red_sphero'].y)) + ')'
    else:
        red_position = "Red not found"

    # Position Information
    cv2.putText(arena_img, red_position,
                (1000, 40),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    if (center['blue_sphero'] is not None):
        blue_position = 'Blue (' + str(int(center['blue_sphero'].x)) + ', ' + str(int(center['blue_sphero'].y)) + ')'
    else:
        blue_position = "Blue not found"

    cv2.putText(arena_img, blue_position,
                (600, 40),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    # Score Information
    cv2.putText(arena_img, 'Red Team: ' + str(score['red_sphero']),
                (1050, 910),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    cv2.putText(arena_img, 'Blue Team: ' + str(score['blue_sphero']),
                (780, 910),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    # Center
    cv2.circle(arena_img, (constants.ORIGIN_PIXELS.x, constants.ORIGIN_PIXELS.y), 3, (255, 255, 255), -1)

    # Sphero Locations
    if(center['red_sphero'] is not None):
        cv2.circle(arena_img, (int(center['red_sphero'].x), int(center['red_sphero'].y)), 10, (0, 0, 255), thickness=2)

        if (flag['red_sphero']):
            cv2.circle(arena_img, (int(center['red_sphero'].x), int(center['red_sphero'].y)), 8, (255, 0, 0), thickness=-1)

    if(center['blue_sphero'] is not None):
        cv2.circle(arena_img, (int(center['blue_sphero'].x), int(center['blue_sphero'].y)), 10, (255, 0, 0), thickness=2)

        if (flag['blue_sphero']):
            cv2.circle(arena_img, (int(center['blue_sphero'].x), int(center['blue_sphero'].y)), 8, (0, 0, 255), thickness=-1)


    # Base Locations
    if (not flag['blue_sphero']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (base['red_sphero'].x, base['red_sphero'].y), 10, (0, 0, 255), thickness=thickness)

    if (not flag['red_sphero']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (base['blue_sphero'].x, base['blue_sphero'].y), 10, (255, 0, 0), thickness=thickness)

    return arena_img
