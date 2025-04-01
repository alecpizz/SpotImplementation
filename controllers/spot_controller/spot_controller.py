"""spot_controller controller."""
import time
from time import sleep

from PID import PID
from controller import AnsiCodes
from controller import CameraRecognitionObject
from spot_driver import SpotDriver
import numpy as np
from enum import Enum
from astarsolver import AstarSolver

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
    front_wall = current_front < WALL_MAX_THRESHOLD


# 50 0.01 5 works kinda
DESIRED_WALL_DISTANCE = 1
KP = 8
KI = 0.0005
KD = 1

last_time = time.time()
nodes = set()

right_PID = PID(KP, KI, KD)
left_PID = PID(KP, KI, KD)
def string_to_array(input, width):
    string_length = len(input)
    if string_length % width != 0:
        return None
    rows = string_length // width
    result = []
    for i in range(rows):
        start = i * width
        end = (i + 1) * width
        row = list(input[start:end])
        result.append(row)
    return result

mazeStr = spot.get_maze()
mazeWidth = spot.get_maze_width()
maze = string_to_array(mazeStr, mazeWidth)
astar = AstarSolver((1, 0), (7, 6), maze)
commands = astar.solve_maze()

cmd_length = len(commands)
for i in range(cmd_length):
    command = commands[i]
    print(command)
    if command == "left":
        spot.turn_left(4.8)
    elif command == "right":
        spot.turn_right(4.8)
    elif command == "forward":
        end_time = time.time() + 3.08
        while end_time > time.time():
            lidar = np.array(spot.get_lidar_image())
            lidar = lidar[np.isfinite(lidar)]
            check_walls()
            right_distance = right_average
            left_distance = left_average
            pid_output = right_PID.calc_pid(right_distance, DESIRED_WALL_DISTANCE)
            pid_output2 = left_PID.calc_pid(left_distance, DESIRED_WALL_DISTANCE)
            pid_multi = 0.005
            angular_z = pid_output * -pid_multi
            print(angular_z)
            if right_distance > WALL_MAX_THRESHOLD + 0.5 and left_distance < DESIRED_WALL_DISTANCE:
                print("flip")
                angular_z = (pid_output2) * -pid_multi
            linear_x = 0.5
            linear_y = 0.0
            spot.direction(linear_x, linear_y, angular_z)
            spot.step(spot.get_timestep())

    spot.stop_moving(0.2)


# while spot.step(spot.get_timestep()) != -1:
#     print(AnsiCodes.CLEAR_SCREEN + "\nDATA:")
#     lidar = np.array(spot.get_lidar_image())
#     lidar = lidar[np.isfinite(lidar)]
#     check_walls()
#
#     right_distance = right_average
#     pid_output = right_PID.calc_pid(right_distance, DESIRED_WALL_DISTANCE)
#
#     forward_speed = 50
#     print("PID", pid_output)
#     # TODO: check PID for Left, Forward
#     # for a forward, we should probably try to ensure 0 distance in front. or some other threshold
#     # that way if we see some distance, we know its turning time and can influence the other PIDs more.
#     turn_step = pid_output
#     if turn_step > 0:
#         spot.turnleft(abs(turn_step))
#     elif turn_step < 0:
#         spot.turnright(abs(turn_step))
#     elif front_wall:
#         spot.turnleft(forward_speed)
#     else:
#         spot.forward(forward_speed)
#
#     objects = spot.get_camera().getRecognitionObjects()
#     if len(objects) > 0:
#         for obj in objects:
#             colors = obj.getColors()
#             nodes.add(tuple([colors[0], colors[1], colors[2]]))
#         # colors = objects[0].getColors()
#         # if objects[0].getNumberOfColors() > 0:
#         #     print("COLOR", colors[0], colors[1], colors[2])
#         #     if colors[0] >= 0.95 and front_wall:
#         #         break
#
#     print("Colors: ", nodes)
