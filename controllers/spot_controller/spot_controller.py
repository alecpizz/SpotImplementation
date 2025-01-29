"""spot_controller controller."""
from controller import AnsiCodes
from spot_driver import SpotDriver
import numpy as np
spot = SpotDriver()

FRONT_DISTANCE = 0.27

def is_front(value):
    return value > FRONT_DISTANCE
#lidar distances are scanned in left to right, aka clockwise. if the index of the value is before
# our front marker, it is to the left. if it's more, it's to the right. where does it start?

def wall_direction(lidar_data):
    front_index = 0 #first value is 0
    data_count = len(lidar_data)
    middle_pt = data_count // 2
    left_array, right_array = lidar_data[middle_pt + 1:], lidar_data[:middle_pt]
    left_avg = np.mean(left_array)
    right_avg = np.mean(right_array)
    print("left_avg: ", left_avg)
    print("right_avg: ", right_avg)


while spot.step(spot.get_timestep()) != -1:
    print(AnsiCodes.CLEAR_SCREEN + "\n")
    lidar = np.array(spot.get_lidar_image())
    lidar = lidar[np.isfinite(lidar)]
    max_lidar = np.max(lidar)
    min_lidar = np.min(lidar)
    print("max ", max_lidar)
    print("min ", min_lidar)
    wall_direction(lidar)
    spot.forward(25)

