import math

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
	ball-ball coefficient of friction (μ): 0.03-0.08
	ball-ball coefficient of restitution (e): 0.92-0.98
	ball-cloth coefficient of rolling resistance (μ): 0.005 - 0.015
	ball-cloth coefficient of sliding friction (μ): 0.15-0.4 (typical value: 0.2)
	ball-cloth spin deceleration rate: 5-15 rad/sec2
	ball-rail coefficient of restitution (e): 0.6-0.9
	ball-table coefficient of restitution (e): 0.5
	cue-tip-ball coefficient of friction (μ): 0.6
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

CUE_LEN = 150 # cm


def main():
	print('--Welcome to PoolBot--\n')

	# Initialize cue ball, other ball, and pocket
	CB = Ball(20, 20)
	OB = Ball(30, 30)
	pocket = Vector(0,0)
	hand_1, hand_2 = get_shot(CB, OB, pocket)

	return hand_1, hand_2 # , vel


def get_shot(CB, OB, pocket):
	# Get line from CB to pocket (assume balls are lined up straight)
	direction = (CB.pos - pocket).unit_vector()

	# Get force
	'''Use baxter's Limb.set_joint_velocity()'''
	'''Maybe just calculate this with guess and check''' 

	# To hit ball put hands on line 'direction'
	global CUE_LEN
	hand_1 = CB - direction*5
	hand_2 = CB - direction*CUE_LEN
	# print('hand_1:', hand_1)
	# print('hand_2:', hand_2)

	return hand_1, hand_2, # vel

def get_trajectory(ball1, ball2, pocket):
	'''Assume straight shot for now'''
	pass



class Ball():
	def __init__(self, x, y):
		self.diameter = 2.25  	# inches (convert to cm * 2.54)
		self.mass = 6 			# oz 
		self.x = x 
		self.y = y 
		self.pos = Vector(x,y)

	def __sub__(self, other):
		return ((self.x - other.x), ((self.y - other.y)))



class Vector():
	def __init__(self, x, y):
		self.mag = math.sqrt(x**2 + y**2)
		self.x = x 
		self.y = y
		self.pos = (x, y) 

	def __sub__(self, other):
		return Vector((self.x - other.x), ((self.y - other.y)))

	def __mul__(self, scalar):
		return Vector((scalar*self.x), ((scalar*self.y)))

	def __str__(self):
		return 'Dir: %s\nMag: %s' % (self.pos, self.mag)

	def unit_vector(self):
		return Vector(self.x / self.mag, self.y / self.mag)


if __name__ == '__main__':
	main()