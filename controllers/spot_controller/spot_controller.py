"""spot_controller controller."""
import time
from time import sleep

from controller import AnsiCodes
from controller import CameraRecognitionObject
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
    right_wall = current_right < 1.5


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

#50 0.01 5 works kinda
DESIRED_WALL_DISTANCE = 1
KP = 45
KI = 0.01
KD = 5
integral_error = 0.0
prev_error = 0.0
last_time = time.time()

def calc_pid(current_distance):
    global integral_error, prev_error, last_time
    error = DESIRED_WALL_DISTANCE - current_distance
    print("Error: ", error, current_distance)
    current_time = time.time()
    delta_time = current_time - last_time

    spring = KP * error

    integral_error += error * delta_time
    integral = KI * integral_error

    damper = (error - prev_error) / delta_time if delta_time > 0 else 0.0
    damper = KD * damper

    prev_error = error
    last_time = current_time

    return spring + integral + damper

while spot.step(spot.get_timestep()) != -1:
    print(AnsiCodes.CLEAR_SCREEN + "\nDATA:")
    lidar = np.array(spot.get_lidar_image())
    lidar = lidar[np.isfinite(lidar)]
    check_walls()

    right_distance = right_average
    pid_output = calc_pid(right_distance)

    forward_speed = 50
    turn_speed_multi = 3
    turn_step = pid_output * turn_speed_multi
    if turn_step > 0:
        spot.turnleft(abs(turn_step))
    elif turn_step < 0:
        spot.turnright(abs(turn_step))
    else:
        spot.move_forward(forward_speed)

    objects = spot.get_camera().getRecognitionObjects()
    if len(objects) > 0:
        colors = objects[0].getColors()
        if objects[0].getNumberOfColors() > 0:
            print("COLOR", colors[0], colors[1], colors[2])
            if colors[0] >= 0.95 and front_wall:
                break



    # if not right_wall:  # No wall on the right
    #     spot.turnright(50)
    #     print("no right wall")
    # elif not front_wall:  # No wall ahead
    #     spot.forward(10)
    #     print("no front wall")
    # elif not left_wall:  # No wall on the left
    #     spot.turnleft(50)
    #     print("no left wall")
    # else:  # Wall on all sides (dead end or very tight turn)
    #     spot.turnleft(50)
    #     print("wall on all sides")
    #
    # objects = spot.get_camera().getRecognitionObjects()
    # if len(objects) > 0:
    #     colors = objects[0].getColors()
    #     if objects[0].getNumberOfColors() > 0:
    #         print("COLOR", colors[0], colors[1], colors[2])
    #         if colors[0] == 1.0 and front_wall:
    #             break
