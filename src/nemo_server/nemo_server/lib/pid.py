import time

class PID():
    def __init__(self, kp, ki = 0, kd = 0, min=None, max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min = min
        self.max = max

        self.reset()

    def update(self, error):
        now = time.time()
        self.dt = now - self.last_update
        self.last_update = now

        self.integral += (error * self.dt)

        deriv = (error - self.prev_error) / self.dt
        
        self.prev_error = error

        result = (self.kp * error) + (self.ki * self.integral) + (self.kd * deriv)

        if self.min is not None: result = max(self.min, result) # Constrain to minimum

        if self.max is not None: result = min(self.max, result) # Constrain to maximum
        
        return result
    
    def reset(self):
        self.dt = 0

        self.integral = 0
        self.prev_error = 0
        self.last_update = time.time()