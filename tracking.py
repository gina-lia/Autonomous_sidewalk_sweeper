import math
import time 

#parameters
wheel_radius = 0.024 #meters 
pulses_per_rev = 2400
distance_per_pulse = (2*math.pi*wheel_radius)/pulses_per_rev

encoder_sign = 1 #check if encoder count increases or decreases when robot moves forward 
#pose(m) 
x = 0.0 #east/west
y = 0.0 #north/south 
theta = 0.0 

class DeadReckoning:
    def __init__(self, wheel_radius = 0.024, pulses_per_rev = 2400):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.wheel_radius = wheel_radius
        self.pulses_per_rev = pulses_per_rev
        self.last_encoder = None
    
    def update(self, encoder_count, heading_deg):
        #first reading, save encoder count only
        if self.last_encoder is None:
            self.last_encoder = encoder_count
            return self.x, self.y, self.theta
        
        #encoder change
        delta_ticks = encoder_count - self.last_encoder
        self.last_encoder = encoder_count

        #converter ticks --> distance
        wheel_circumference = 2 * math.pi * self.wheel_radius
        distance = wheel_circumference * (delta_ticks / self.pulses_per_rev)

        #convert heading to rad
        self.theta = math.radians(heading_deg)

        #update position
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        return self.x, self.y, self.theta
