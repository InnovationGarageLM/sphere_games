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
    fontScale = .6
    fontColor = (255, 255, 255)
    lineType = 2
    line_height = 25

    cv2.putText(arena_img, 'LM Autonomy Hackathon',
                (10,line_height),
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


    cv2.putText(arena_img, 'Status: ',
                (10, constants.PICTURE_SIZE[1] - 4*line_height),
                font,
                fontScale,
                fontColor,
                lineType)

    cv2.putText(arena_img, game_text,
                (30, constants.PICTURE_SIZE[1] - 3*line_height),
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
    cv2.putText(arena_img, 'Time Left: ',
                (10, constants.PICTURE_SIZE[1] - 2*line_height),
                font,
                fontScale,
                fontColor,
                lineType)

    # Time Remaining
    cv2.putText(arena_img, str(int(constants.TOTAL_ALLOWED_TIME - time_elapsed)) + ' s',
                (30, constants.PICTURE_SIZE[1] - line_height),
                font,
                fontScale,
                fontColor,
                lineType)

    left_column = constants.PICTURE_SIZE[0] - 200

    # Position Information
    cv2.putText(arena_img, "Position Info",
                (left_column, 2*line_height),
                font,
                fontScale,
                (255, 255, 255),
                lineType)

    if(center['red_sphero'] is not None):
        center_mm = pixels_2_mm(center['red_sphero'])
        red_position    = '(' + str(int(center['red_sphero'].x)) + ', ' + str(int(center['red_sphero'].y)) + ') px'
        red_position_mm = '(' + str(int(center_mm.x)) + ', ' + str(int(center_mm.y)) + ') mm'
    else:
        red_position = "Red not found"
        red_position_mm = "Red not found"

    # Red Position Information
    cv2.putText(arena_img, "Red",
                (left_column, 4*line_height),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    # Position Information
    cv2.putText(arena_img, red_position,
                (left_column, 5*line_height),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    # Position Information
    cv2.putText(arena_img, red_position_mm,
                (left_column, 6*line_height),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    if (center['blue_sphero'] is not None):
        center_mm = pixels_2_mm(center['blue_sphero'])
        blue_position    = '(' + str(int(center['blue_sphero'].x)) + ', ' + str(int(center['blue_sphero'].y)) + ') px'
        blue_position_mm = '(' + str(int(center_mm.x)) + ', ' + str(int(center_mm.y)) + ') mm'
    else:
        blue_position = "Blue not found"
        blue_position_mm = "Blue not found"

    cv2.putText(arena_img, "Blue",
                (left_column, 8 * line_height),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    cv2.putText(arena_img, blue_position,
                (left_column, 9*line_height),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    cv2.putText(arena_img, blue_position_mm,
                (left_column, 10*line_height),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    # Score Information
    # Score Information
    cv2.putText(arena_img, 'Scores:',
                (left_column, constants.PICTURE_SIZE[1] - 7 * line_height),
                font,
                fontScale,
                (255, 255, 255),
                lineType)

    cv2.putText(arena_img, 'Red Team: ' + str(score['red_sphero']),
                (left_column, constants.PICTURE_SIZE[1] - 5 * line_height),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    cv2.putText(arena_img, 'Blue Team: ' + str(score['blue_sphero']),
                (left_column, constants.PICTURE_SIZE[1] - 3 * line_height),
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
