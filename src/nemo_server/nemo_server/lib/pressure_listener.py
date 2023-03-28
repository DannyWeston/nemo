from sensor_msgs.msg import FluidPressure

class PressureListener():

    def __init__(self, node, topic="/nemo/pressure", rate=10, logger=None):
        self.logger = logger
        self.rate = rate
        self.topic= topic

        self.air_pressure = 101325.0 # Air pressure in Pa
        self.water_pressure = 10122.0 # Water pressure/metre in Pa/m

        # Allow for queue that is 1 second's worth of data
        self.sub = node.create_subscription(FluidPressure, topic, self.callback, rate)

        self.value = None # History buffer with queue size of 10

    def callback(self, msg):
        # Take minimum reading as air pressure
        self.value = max(msg.fluid_pressure, self.air_pressure)