from ..lib.pid import PID

class Depth:
    def __init__(self, pid_params, tolerance = 0.02):
        self.target = -0.3 # Target depth of 30cm by default

        self.tolerance = tolerance

        self.min_target = -1.0 # Don't try to go higher than 0.3 in the water

        self.max_target = -0.3 # Don't try to go lower than 1 metre in the water

        self.pid = PID(pid_params["kp"], pid_params["ki"], pid_params["kd"], pid_params["min"], pid_params["max"])

    def update(self, depth):
        error = self.target - depth

        return self.pid.update(error)
        
    def target_callback(self, msg):
        self.target = max(min(msg.data, self.max_target), self.min_target)

    def reset(self):
        self.pid.reset()