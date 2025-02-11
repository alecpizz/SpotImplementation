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
# TODO: Figure out thresholds. will probably need to be setup based on the size of the maze.
WALL_MIN_THRESHOLD = 0.15
WALL_MAX_THRESHOLD = 1.05
LEFT_WALL_INDEX = 2
RIGHT_WALL_INDEX = 5
FRONT_WALL_INDEX = 0, 7
left_wall = False
right_wall = False
front_wall = False
left_average = 0.0
right_average = 0.0
front_average = 0.0
left_wall_prev = False
right_wall_prev = False
front_wall_prev = False
sectors = []


def inverse_lerp(a, b, v):
    return np.clip((v - a) / (b - a), 0, 1)


def lerp(a, b, t):
    return (1.0 - t) * a + b * t


def remap(i_min, i_max, o_min, o_max, v):
    t = inverse_lerp(i_min, i_max, v)
    return lerp(o_min, o_max, t)


def check_walls():
    left_wall_present()
    right_wall_present()
    front_wall_present()


def left_wall_present():
    global left_wall, left_wall_prev, sectors, lidar, left_average
    left_wall_prev = left_wall
    num_data = len(lidar)
    start_index_float = (225 / 360.0) * num_data
    end_index_float = (270 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    extracted_data = lidar[start_index: end_index + 1]
    current_left = np.mean(extracted_data)
    left_average = current_left
    print("Left Wall diff at sector:", LEFT_WALL_INDEX, current_left)
    left_wall = current_left < WALL_MAX_THRESHOLD


def right_wall_present():
    global right_wall, sectors, right_wall_prev, sectors, right_average
    right_wall_prev = right_wall
    num_data = len(lidar)
    start_index_float = (45 / 360.0) * num_data
    end_index_float = (135 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    extracted_data = lidar[start_index: end_index + 1]
    current_right = np.mean(extracted_data)
    right_average = current_right
    print("Right Wall diff at sector:", RIGHT_WALL_INDEX, current_right)
    right_wall = current_right < WALL_MAX_THRESHOLD


def front_wall_present():
    global front_wall, front_wall_prev, sectors, front_average
    front_wall_prev = front_wall
    num_data = len(lidar)
    start_index_float = (0 / 360.0) * num_data
    end_index_float = (40 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    slice1 = lidar[start_index: end_index + 1]

    start_index_float = (315 / 360.0) * num_data
    end_index_float = (355 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    slice2 = lidar[start_index: end_index + 1]

    extracted_data = np.concatenate((slice1, slice2))
    current_front = np.mean(extracted_data)
    front_average = current_front
    print("Front Wall diff at sector:", FRONT_WALL_INDEX, current_front)
    front_wall = current_front < WALL_MAX_THRESHOLD

# lidar distances are scanned in left to right, aka clockwise. if the index of the value is before
# our front marker, it is to the left. if it's more, it's to the right. where does it start?

def wall_direction(lidar_data, num_sectors=8):
    global sectors
    sectors = np.array_split(lidar_data, num_sectors)
    for i in range(num_sectors):
        print("sector", i, ":", "data size:", len(sectors[i]), "avg:", np.mean(sectors[i]))
    # split the data into 8 sectors, store em somewhere


while spot.step(spot.get_timestep()) != -1:
    print(AnsiCodes.CLEAR_SCREEN + "\nDATA:")
    lidar = np.array(spot.get_lidar_image())
    lidar = lidar[np.isfinite(lidar)]
    wall_direction(lidar)
    check_walls()
    # if not right_wall:  # No wall on the right
    #     spot.turnright(100)
    #     print("no right wall")
    # elif not front_wall:  # No wall ahead
    #     spot.forward(10)
    #     print("no front wall")
    # elif not left_wall:  # No wall on the left
    #     spot.turnleft(100)
    #     print("no left wall")
    # else:  # Wall on all sides (dead end or very tight turn)
    #     spot.turnleft(100)
    #     print("wall on all sides")
