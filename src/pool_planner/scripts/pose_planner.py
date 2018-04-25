#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pool_physics
from std_msgs.msg import String
from pool_physics import Ball, Pocket, get_shot, get_trajectory
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def move_group_python_interface_tutorial():

    print "============ Starting setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    group = moveit_commander.MoveGroupCommander("right_arm")

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    # print "============ Waiting for RVIZ..."
    # rospy.sleep(10)
    # print "============ Starting tutorial "

    print "============ Genarating starting pose"
    pose_target = geometry_msgs.msg.Pose()

    # Pool physics
    CB = Ball(2.25, -0.5)
    OB = Ball(3, -0.5)
    pocket = Pocket(4, -0.5)
    hand = get_shot(CB, OB, pocket)

    # position
    pose_target.position.x = hand[0]
    pose_target.position.y = hand[1]
    pose_target.position.z = 0

    # orientation
    # TODO: calculate euler z orientation when not directly line dup
    quaternion = quaternion_from_euler(0, 1.5, 0)
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    # Plan the pose
    group.set_pose_target(pose_target)
    plan1 = group.plan()

    # Move to pose
    group.go(wait=True)
    group.execute(plan1)

    ## Cartesian Paths
    points = get_trajectory(CB, OB, pocket, 0.1)
    print "[INFO] Waypoints: " + str(points)
    waypoints = []

    # first orient gripper and move forward (+x)
    for point in points:
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = point[0]
        wpose.position.y = point[1]
        wpose.position.z = 0
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    (plan3, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold
    print "[INFO] fraction anchieved: " + str(fraction)

    print "============ Sinking the shot"
    # rospy.sleep(5)
    group.execute(plan3)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"


if __name__=='__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
