from geometry_msgs.msg import Point
import numpy as np
import os

# Game runs for 5 minutes
TOTAL_ALLOWED_TIME = 300 # Seconds

SIMULATOR = os.getenv('AUTONOMY_SIMULATOR', False)

if(not SIMULATOR):
    # Attribute of camera image
    PICTURE_SIZE = (1280, 960)

    # Sites may need to adjust the following 4 settings to match their
    # arena and camera positions
    ORIGIN_PIXELS = Point(619, 501, 0)
    BASE = {}
    BASE['red_sphero'] = Point(241, 872, 0)
    BASE['blue_sphero'] = Point(1003, 101, 0)

    ARENA_WIDTH_PIXELS = 1021 - 205 # assumed square
    ARENA_WIDTH_MM = 1143           # assumed square
    WALL_TO_BASE = 70               # Assume square, and that bases are in corners, fudge factor included
else:
    # Attribute of camera image
    PICTURE_SIZE = (1920, 1080)

    # Sites may need to adjust the following 4 settings to match their
    # arena and camera positions
    ORIGIN_PIXELS = Point(959, 546, 0)
    BASE = {}
    BASE['red_sphero'] = Point(1392, 114, 0)
    BASE['blue_sphero'] = Point(514, 981, 0)

    ARENA_WIDTH_PIXELS = 1027 - 212  # assumed square
    ARENA_WIDTH_MM = 1143  # assumed square
    WALL_TO_BASE = 70  # Assume square, and that bases are in corners, fudge factor included

ARENA_BOUNDS = {'left': min(BASE['red_sphero'].x, BASE['blue_sphero'].x) - WALL_TO_BASE,
                'right': max(BASE['red_sphero'].x, BASE['blue_sphero'].x) + WALL_TO_BASE,
                'top': min(BASE['red_sphero'].y, BASE['blue_sphero'].y) - WALL_TO_BASE,
                'bottom': max(BASE['red_sphero'].y, BASE['blue_sphero'].y) + WALL_TO_BASE}

for key in ARENA_BOUNDS:
    if(int(ARENA_BOUNDS[key]) < 0):
        ARENA_BOUNDS[key] = 0

# These should be good enough if arena built-to-spec
COVERT_PIXEL2MM = (ARENA_WIDTH_MM / 10.) / (ARENA_WIDTH_PIXELS / 10.)
COVERT_MM2PIXEL = (ARENA_WIDTH_PIXELS / 10.) / (ARENA_WIDTH_MM / 10.)

def set_conversions():
    global ARENA_BOUNDS, COVERT_MM2PIXEL, COVERT_PIXEL2MM
    ARENA_BOUNDS = {'left': min(BASE['red_sphero'].x, BASE['blue_sphero'].x) - WALL_TO_BASE,
                    'right': max(BASE['red_sphero'].x, BASE['blue_sphero'].x) + WALL_TO_BASE,
                    'top': min(BASE['red_sphero'].y, BASE['blue_sphero'].y) - WALL_TO_BASE,
                    'bottom': max(BASE['red_sphero'].y, BASE['blue_sphero'].y) + WALL_TO_BASE}

    for key in ARENA_BOUNDS:
        if (int(ARENA_BOUNDS[key]) < 0):
            ARENA_BOUNDS[key] = 0

    # These should be good enough if arena built-to-spec
    COVERT_PIXEL2MM = (ARENA_WIDTH_MM/10.) / (ARENA_WIDTH_PIXELS/10.)
    COVERT_MM2PIXEL = (ARENA_WIDTH_PIXELS/10.) / (ARENA_WIDTH_MM/10.)

set_conversions()

# Threshold to count a "score" in pixels
SCORE_THRESHOLD = 100

red_lower_1 = np.array([0, 50, 50])
red_upper_1 = np.array([10, 255, 255])

red_lower_2 = np.array([160, 50, 50])
red_upper_2 = np.array([180, 255, 255])

blue_lower = np.array([110, 50, 50])
blue_upper = np.array([120, 255, 255])

COLOR_THRESHOLDS = {}
COLOR_THRESHOLDS['red_sphero'] = [(red_lower_1, red_upper_1), (red_lower_2, red_upper_2)]
COLOR_THRESHOLDS['blue_sphero'] = [(blue_lower, blue_upper)]