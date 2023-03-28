from .pressure_listener import PressureListener
from .odom_listener import OdomListener

class Localiser():
    def __init__(self, node, odom_topic = "/nemo/odom", pressure_topic = "/nemo/pressure", rate=10, logger=None):
        self.logger = logger
        self.rate = rate

        self.odom_listener = OdomListener(node, topic=odom_topic, rate=rate, logger=logger)

        self.pressure_listener = PressureListener(node, topic=pressure_topic, rate=rate, logger=logger)

    def get_position(self):
        # Convert pressure reading to depth in Metres
        reading = self.pressure_listener.value

        if reading is not None: # Pressure sensor present so use reading 
            depth = (reading - self.pressure_listener.air_pressure) / self.pressure_listener.water_pressure # Convert to metres
        else:
            # Can't do anything with non-existent value so fallback to odometry
            depth = -self.odom_listener.posY

        return (self.odom_listener.posX, depth, self.odom_listener.posZ)
    
    def get_orientation(self):
        return (self.odom_listener.rotX, self.odom_listener.rotY, self.odom_listener.rotZ)