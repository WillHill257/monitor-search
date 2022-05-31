import numpy as np
import time


class PID():

    def __init__(self, Kp=1, Ki=0, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.prev_time = 0
        self.e_i = 0

    def u(self, pv, setpoint):

        curr_time = time.time()

        dt = curr_time - self.prev_time

        if self.prev_time == 0:
            dt = 0

        
        error = setpoint - pv

        

        if type(error).__module__ == np.__name__ and error.size > 1:
            error = np.linalg.norm(error)

        de = error - self.prev_error

        e_p = error
        curr_i = error * dt

        self.e_i += curr_i

        e_d = 0
        if dt > 0:
            e_d = de / dt

        self.prev_time = curr_time
        self.prev_error = np.copy(error)

        u = (self.Kp * e_p) + (self.Ki * self.e_i) + (self.Kd * e_d)

        

        # Anti integral windup
        if np.abs(u) > 1:
            self.e_i -= curr_i

        u = np.clip(u, -1, 1)

        return u
