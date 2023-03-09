from scenic.simulators.gta.model import *
ego = Car 
spot = OrientedPoint on visible curb
badAngle = Uniform(1, -1) * Range(10, 20) deg
Car right of spot by 0.25,
    facing badAngle relative to roadDirection