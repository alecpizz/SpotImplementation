"""spot_controller controller."""
from controller import AnsiCodes
from spot_driver import SpotDriver
import numpy as np
spot = SpotDriver()

FRONT_DISTANCE = 0.27
#TODO: Figure out thresholds. will probably need to be setup based on the size of the maze.
WALL_MIN_THRESHOLD = 0.25
WALL_MAX_THRESHOLD = 0.75
LEFT_WALL_INDEX = 2
RIGHT_WALL_INDEX = 5
FRONT_WALL_INDEX = 3
left_wall = False
right_wall = False
front_wall = False
sectors = []

def check_walls():
    left_wall_present()
    right_wall_present()
    front_wall_present()

def left_wall_present():
    global left_wall, sectors
    current_left = np.mean(sectors[LEFT_WALL_INDEX])
    print("Left Wall diff at sector:", LEFT_WALL_INDEX,  current_left)
    left_wall = WALL_MIN_THRESHOLD <= current_left <= WALL_MAX_THRESHOLD

def right_wall_present():
    global right_wall, sectors
    current_right = np.mean(sectors[RIGHT_WALL_INDEX])
    print("Right Wall diff at sector:", RIGHT_WALL_INDEX, current_right)
    right_wall = WALL_MIN_THRESHOLD <= current_right <= WALL_MAX_THRESHOLD

def front_wall_present():
    global front_wall, sectors
    current_front = np.mean(sectors[FRONT_WALL_INDEX])
    print("Front Wall diff at sector:", FRONT_WALL_INDEX, current_front)
    front_wall = WALL_MIN_THRESHOLD <= current_front <= WALL_MAX_THRESHOLD



def is_front(value):
    return value > FRONT_DISTANCE
#lidar distances are scanned in left to right, aka clockwise. if the index of the value is before
# our front marker, it is to the left. if it's more, it's to the right. where does it start?

def wall_direction( lidar_data, num_sectors=8):
    global sectors
    sectors = np.array_split(lidar_data, num_sectors)
    for i in range(num_sectors):
        print("sector", i, ":", "data size:", len(sectors[i]), "avg:", np.mean(sectors[i]))
    #split the data into 8 sectors, store em somewhere


while spot.step(spot.get_timestep()) != -1:
    print(AnsiCodes.CLEAR_SCREEN + "\n")
    lidar = np.array(spot.get_lidar_image())
    lidar = lidar[np.isfinite(lidar)]
    wall_direction(lidar)
    check_walls()
    if left_wall:
        print("Left Wall detected")
    elif right_wall:
        print("Right Wall detected")
    elif front_wall:
        print("Front Wall detected")
    else:
        print("No Wall detected")


