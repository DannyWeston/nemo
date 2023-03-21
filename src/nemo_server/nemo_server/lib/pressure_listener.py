from .history_buffer import HistoryBuffer

class PressureListener():

    def __init__(self, logger=None):
        self.logger = logger

        # water pressure per meter of depth (assuming 1000 kg / m^3 @ 4 degrees Celsius)
        # With standard gravitational constant of 9.80665 m  /s^2
        # = 9.80665 kPa / m 
        # air pressure taken to be 101.325 kPa\
        self.air_pressure = 101325 
        self.water_pressure_per_metre = 9806.65

        self.pressure_data = HistoryBuffer(10) # History buffer with queue size of 10

        self.topic = '/nemo/pressure'

    def callback(self, msg):
        # Apply thresholding to water pressure reading
        reading = max(msg.fluid_pressure, self.air_pressure) - self.air_pressure

        self.pressure_data.insert(reading)

        water_pressure = self.pressure_data.mean_value()
        depth = water_pressure / self.water_pressure_per_metre

        #self.logger.info(f'{water_pressure:.2f}, {depth:.2f}') 