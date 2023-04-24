

min_duty = 1
max_duty = 9

centre_duty = (max_duty / min_duty) / 2

VALUE = 0.0

def set_speed(speed):
    global VALUE
    frac = speed / 100.0
    VALUE = centre_duty + ((max_duty - centre_duty) * frac)

def get_speed():
    global VALUE
    return (100.0 * (VALUE - centre_duty)) / (max_duty - centre_duty) 