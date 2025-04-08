"""spot_controller controller."""
import time

from PID import PID
from spot_driver import SpotDriver
import numpy as np
from astarsolver import AstarSolver

spot = SpotDriver()

FRONT_DISTANCE = 0.27
# TODO: Figure out thresholds. will probably need to be setup based on the size of the maze.
WALL_MIN_THRESHOLD = 0.15
WALL_MAX_THRESHOLD = 1.05
LEFT_WALL_INDEX = 2
RIGHT_WALL_INDEX = 5
FRONT_WALL_INDEX = 0, 7
left_average = 0.0
right_average = 0.0
front_average = 0.0


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
    global  lidar, left_average
    num_data = len(lidar)
    start_index_float = (225 / 360.0) * num_data
    end_index_float = (270 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    extracted_data = lidar[start_index: end_index + 1]
    current_left = np.mean(extracted_data)
    left_average = current_left


def right_wall_present():
    global  right_average
    num_data = len(lidar)
    start_index_float = (45 / 360.0) * num_data
    end_index_float = (135 / 360.0) * num_data
    start_index = int(start_index_float)
    end_index = int(end_index_float)
    extracted_data = lidar[start_index: end_index + 1]
    current_right = np.mean(extracted_data)
    right_average = current_right


def front_wall_present():
    global  front_average
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


# 50 0.01 5 works kinda
DESIRED_WALL_DISTANCE = 1
KP = 8
KI = 0.0005
KD = 1

right_PID = PID(KP, KI, KD)
front_PID = PID(KP, KI, KD)
left_PID = PID(KP, KI, KD)
avg_PID = PID(KP, KI, KD)
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
        end_time = time.time() + 3.2
        while end_time > time.time():
            #grab lidar image and check surrounding walls
            lidar = np.array(spot.get_lidar_image())
            lidar = lidar[np.isfinite(lidar)]
            check_walls()
            right_distance = right_average
            left_distance = left_average
            front_distance = front_average
            #caluclate left-right pid, left pid, right pid, fwd pid
            pid_output_average = avg_PID.calc_pid(left_distance - right_distance, DESIRED_WALL_DISTANCE)
            pid_output_left = left_PID.calc_pid(left_distance, DESIRED_WALL_DISTANCE)
            pid_output_right = right_PID.calc_pid(right_distance, DESIRED_WALL_DISTANCE)
            pid_output_front = front_PID.calc_pid(front_distance, DESIRED_WALL_DISTANCE)
            #base movement
            pid_multi = 0.005
            linear_x = 0.5
            linear_y = 0.0
            angular_z = pid_output_average * -pid_multi
            #suddenly no right wall
            if right_distance > WALL_MAX_THRESHOLD + 0.5 and left_distance < DESIRED_WALL_DISTANCE:
                angular_z = pid_output_left * -pid_multi
            #suddenly no left wall
            elif left_distance > WALL_MAX_THRESHOLD + 0.5 and right_distance < DESIRED_WALL_DISTANCE:
                angular_z = pid_output_right * -pid_multi
            #no left wall or right wall, just try to go straight
            elif right_distance > WALL_MAX_THRESHOLD and left_distance > WALL_MAX_THRESHOLD:
                angular_z = 0.0
            #wall incoming, slow down
            if 1 > front_distance > 0.0:
                linear_x = pid_output_front * pid_multi

            spot.direction(linear_x, linear_y, angular_z)
            spot.step(spot.get_timestep())

    spot.stop_moving(0.5)
