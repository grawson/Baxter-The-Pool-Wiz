#!/usr/bin/env python

import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pool_physics
from std_msgs.msg import String
from pool_physics import Ball, Pocket, get_shot, get_trajectory, get_z_rot
from tf.transformations import euler_from_quaternion, quaternion_from_euler


BALL_Z = -0.17
# BALL_Z = 0

## WORLD 1 ##############
# CUE = [2.75, -0.5]
# TARGET = [3, -0.5]
# POCKET = [4, -0.5]
## WORLD 2 ##############
CUE = [ 2.366, -0.116]
TARGET = [3, -0.5]
POCKET = [ 3.166, 0.084]
########################

class PosePlanner(object):

    def __init__(self):
        print "============ Starting setup"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pose_planner', anonymous=True)
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.group.set_max_velocity_scaling_factor(1)

        ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        # print "============ Waiting for RVIZ..."
        rospy.sleep(2)
        # print "============ Starting tutorial "

        print "============ Genarating starting pose"
        self.pose_target = geometry_msgs.msg.Pose()

        # Pool physics
        self.cue_ball = Ball(CUE[0], CUE[1])
        self.target_ball = Ball(TARGET[0], TARGET[1])
        self.pocket = Pocket(POCKET[0], POCKET[1])


    def initial_pose(self):
        hand = get_shot(self.cue_ball, self.target_ball, self.pocket)
        print("[INFO] Moving to starting position: " + str(hand))

        # position
        self.pose_target.position.x = hand[0]
        self.pose_target.position.y = hand[1]
        self.pose_target.position.z = BALL_Z

        # orientation
        z = get_z_rot(self.cue_ball, self.target_ball, self.pocket)
        quaternion = quaternion_from_euler(0, math.pi/2.0, z)
        self.pose_target.orientation.x = quaternion[0]
        self.pose_target.orientation.y = quaternion[1]
        self.pose_target.orientation.z = quaternion[2]
        self.pose_target.orientation.w = quaternion[3]

        # Plan the pose
        self.group.set_pose_target(self.pose_target)
        plan1 = self.group.plan()

        # Move to pose
        self.group.go(wait=True)
        self.group.execute(plan1)

    def swing(self):
        waypoints = []

        # METHOD 1 #######
        # hand = get_shot(self.cue_ball, self.target_ball, self.pocket)
        # points = [hand, [self.cue_ball.pos[0], self.cue_ball.pos[1]]]
        #################

        # METHOD 2 #######
        # points = get_trajectory(self.cue_ball, self.target_ball, self.pocket, 0.1)
        #################

        # METHOD 3 #######
        points = get_trajectory(self.cue_ball, self.target_ball, self.pocket, 0.1)
        points = [points[1], points[5]]
        #################


        print "[INFO] Waypoints: " + str(points)


        # first orient gripper and move forward (+x)
        z = get_z_rot(self.cue_ball, self.target_ball, self.pocket)
        quaternion = quaternion_from_euler(0, math.pi/2.0, z)
        for point in points:
            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = point[0]
            wpose.position.y = point[1]
            wpose.position.z = BALL_Z
            wpose.orientation.x = quaternion[0]
            wpose.orientation.y = quaternion[1]
            wpose.orientation.z = quaternion[2]
            wpose.orientation.w = quaternion[3]
            waypoints.append(copy.deepcopy(wpose))


        ## We want the cartesian path to be interpolated at a resolution of 1 cm
        ## which is why we will specify 0.01 as the eef_step in cartesian
        ## translation.  We will specify the jump threshold as 0.0, effectively
        ## disabling it.
        (plan3, fraction) = self.group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.00)  # jump_threshold
        print "[INFO] fraction anchieved: " + str(fraction)

        print "============ Sinking the shot"
        rospy.sleep(1)
        self.group.execute(plan3)

    def shutdown(self):
        print "============ STOPPING"
        moveit_commander.roscpp_shutdown()


def main():
    pose_planner = PosePlanner()
    pose_planner.initial_pose()
    # pose_planner.swing()
    pose_planner.shutdown()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
