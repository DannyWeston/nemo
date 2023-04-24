from ..lib.pid import PID

class AutoDepth():
    def __init__(self, kp, ki, kd, min_depth, max_depth, tolerance):
        self.target = -0.3 # Target depth of 30cm by default

        self.tolerance = tolerance

        self.min_depth = min_depth # Don't try to go higher than 0.3 in the water

        self.max_depth = max_depth # Don't try to go lower than 1 metre in the water

        self.pid = PID(kp, ki, kd)

    def update(self, depth):
        return self.pid.update(self.target - depth)

    def reset(self):
        self.pid.reset()

    def set_target(self, target):
        self.target = min(self.max_depth, max(target, self.min_depth))