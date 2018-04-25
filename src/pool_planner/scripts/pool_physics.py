import math
import numpy as np


##########################################
# 
#
# Input:  
# 		cue    - (x,y,z) position of white ball
# 		target - (x,y,z) position of target ball
# 		pocket - (x,y,z) position of pocket
#
# Output:
#		hand_1 position
#		hand_2 position
#		velocity
##########################################

'''
Physical constants:
	ball diameter: 2.25 in
	ball mass: 6 oz
	ball mass moment of inertia: 2/5 mR2
	ball-ball coefficient of friction (mu): 0.03-0.08
	ball-ball coefficient of restitution (e): 0.92-0.98
	ball-cloth coefficient of rolling resistance (mu): 0.005 - 0.015
	ball-cloth coefficient of sliding friction (mu): 0.15-0.4 (typical value: 0.2)
	ball-cloth spin deceleration rate: 5-15 rad/sec2
	ball-rail coefficient of restitution (e): 0.6-0.9
	ball-table coefficient of restitution (e): 0.5
	cue-tip-ball coefficient of friction (mu): 0.6
	cue-tip-ball coefficient of restitution (e): 0.71-0.75 (leather tip), 0.81-0.87 (phenolic tip)

Shot speeds:
	touch: 1.5 mph = 2.2 fps
	slow: 3 mph = 4.4 fps
	medium-soft: 5 mph = 7.3 fps
	medium: 7 mph = 10 fps
	medium-fast: 8 mph = 12 fps
	fast: 12 mph = 18 fps 
	power: 15-20 mph = 22-29 fps
	powerful break: 25-30 mph = 36-44 fps

'''

CUE_LEN = 1.5 # cm
CUE_2_GRIPPER = 0.4 # dist from end of cue to gripper + a bit extra to allow for swing


def get_shot(CB, OB, pocket):
    global CUE_LEN, CUE_2_GRIPPER
    offset = CUE_LEN + CUE_2_GRIPPER

    # Get line from CB to pocket (assume balls are lined up straight)
    direction = norm(pocket.pos - CB.pos)

    # Get force
    '''Use baxter's Limb.set_joint_velocity()'''
    '''Maybe just calculate this with guess and check'''

    # To hit ball put hands on line 'direction'
    hand = CB.pos - direction*offset
    return hand # vel


def get_trajectory(CB, OB, pocket, step=0.1):
    '''Assume straight shot for now'''
    global CUE_LEN, CUE_2_GRIPPER

    direction = norm(pocket.pos - CB.pos)
    start = CB.pos - direction*(CUE_LEN+CUE_2_GRIPPER)
    end = CB.pos - direction*CUE_LEN
    waypoints = list()

    i = 0
    while i < 1:
        waypoints.append(parametric_point(start, end, i))
        i += step
    waypoints.append(parametric_point(start, end, 1))  # ensure destination point is added
    waypoints.append(parametric_point(start, end, 1))  # ensure destination point is added
    return waypoints


# Get the point on a line between two 2D points at a certain step
# when t=0, point=p1. when t=1, point=p2
def parametric_point(p1, p2, t):
    return p1 + (p2 - p1)*t


def norm(arr):
    return arr / math.sqrt(np.dot(arr, arr))  # normed

class Ball():
    def __init__(self, x, y):
        self.diameter = 2.25  # inches (convert to cm * 2.54)
        self.mass = 6  # oz
        self.pos = np.array([x, y])


class Pocket():
    def __init__(self, x, y):
        self.pos = np.array([x, y])
