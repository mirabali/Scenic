from scenic.simulators.gta.model import *
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town07.xodr')

model scenic.domains.driving.model

Car with behavior FollowLaneBehavior
ego = Car


behavior Drive():
    try:
        do FollowLaneBehavior()
    interrupt when self.distanceToNextObstacle() < 20:
        do PassingBehavior()
    interrupt when self.timeToCollision() < 5:
        do CollisionAvoidance()