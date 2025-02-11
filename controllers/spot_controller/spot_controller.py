"""spot_controller controller."""
from controller import AnsiCodes
from spot_driver import SpotDriver
import numpy as np
from enum import Enum
spot = SpotDriver()

class Direction(Enum):
    Forward = 0
    Left = 1
    Right = 2

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
left_wall_prev = False
right_wall_prev = False
front_wall_prev = False
sectors = []

def check_walls():
    left_wall_present()
    right_wall_present()
    front_wall_present()

def left_wall_present():
    global left_wall, left_wall_prev, sectors
    left_wall_prev = left_wall
    current_left = np.mean(sectors[LEFT_WALL_INDEX])
    print("Left Wall diff at sector:", LEFT_WALL_INDEX,  current_left)
    left_wall = WALL_MIN_THRESHOLD <= current_left <= WALL_MAX_THRESHOLD

def right_wall_present():
    global right_wall, sectors, right_wall_prev, sectors
    right_wall_prev = right_wall
    current_right = np.mean(sectors[RIGHT_WALL_INDEX])
    print("Right Wall diff at sector:", RIGHT_WALL_INDEX, current_right)
    right_wall = WALL_MIN_THRESHOLD <= current_right <= WALL_MAX_THRESHOLD

def front_wall_present():
    global front_wall, front_wall_prev, sectors
    front_wall_prev = front_wall
    current_front = np.mean(sectors[FRONT_WALL_INDEX])
    print("Front Wall diff at sector:", FRONT_WALL_INDEX, current_front)
    front_wall = WALL_MIN_THRESHOLD <= current_front <= WALL_MAX_THRESHOLD

def right_wall_changed():
    return right_wall_prev != right_wall

def left_wall_changed():
    return left_wall_prev != left_wall

def front_wall_changed():
    return front_wall_prev != front_wall

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
    print(AnsiCodes.CLEAR_SCREEN + "\nDATA:")
    lidar = np.array(spot.get_lidar_image())
    lidar = lidar[np.isfinite(lidar)]
    wall_direction(lidar)
    check_walls()
    if not right_wall:
        print("wheres my fuckin right wall!")
        # turn or something
        spot.turnright(100)
    else:
        spot.forward(15)
