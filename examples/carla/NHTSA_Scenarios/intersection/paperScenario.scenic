
# model scenic.domains.driving.model
#Could have used a different model like this one --> works with every simulator
"""
TITLE: First Implementation of Paper
AUTHOR: Mira Bali
DESCRIPTION: Ego vehicle goes straight at 4-way intersection while second vehicle (other/adversary vehicle) on the right side is making a right turn.
SOURCE: Inspired by the intersections in carla/NHTSA_Scenarios, especially Intersection 1, 4, 6
"""

#################################
# MAP AND MODEL                 #
#################################


#make this a param so changeable after when actually simulating through command line
param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'

model scenic.simulators.carla.model
#Why does this carla simulator model work with newtonian

#################################
# CONSTANTS                     #
#################################

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = Range(7, 10)

param SAFETY_DIST = Range(10, 20)
CRASH_DIST = 5
TERM_DIST = 70


#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)
	interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
		terminate

#################################
# SPATIAL RELATIONS             #
#################################


#filters to only four way intersections and randomly chooses a four way intersection (with equal probability for any choice)
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

#setting up the ego car to drive straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

#setting up adversary car to take a right
advInitLane = advInitLane = Uniform(*filter(lambda m:
		m.type is ManeuverType.STRAIGHT,
		Uniform(*filter(lambda m: 
			m.type is ManeuverType.STRAIGHT, 
			egoInitLane.maneuvers)
		).conflictingManeuvers)
	).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advInitLane.centerline

#centerline --> is at center of the one chosen lane?, have a direction on the centerline?

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior EgoBehavior(egoTrajectory)

adversary = Car at advSpawnPt,
	with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
