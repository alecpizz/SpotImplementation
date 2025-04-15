import time


class PID:
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def calc_pid(self, current_distance, desired_distance):
        error = desired_distance - current_distance
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        spring = self.KP * error

        self.integral_error += error * delta_time
        integral = self.KI * self.integral_error

        damper = (error - self.prev_error) / delta_time if delta_time > 0 else 0.0
        damper = self.KD * damper

        self.prev_error = error
        self.last_time = current_time
        return spring + integral + damper

    def reset_pid(self):
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()