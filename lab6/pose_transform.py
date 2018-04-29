#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy as np
import math
from cozmo.util import degrees, Pose

def get_relative_pose(object_pose, reference_frame_pose):
    # ####
    # Solving basic trignometry, we get below.
	# 
	# Please note 
	#	x1' and y1' are position coordinates of cube relative to robot.
	#	beta' is rotation of cube relative to robot.
	#
    # x1' = (x1-x0)Cos(alpha) + (y1-y0)Sin(alpha)
    # y1' = (y1-y0)Cos(alpha) - (x1-x0)Sin(alpha)
    #
    # beta' = beta - alpha
	#
    # This can be expressed as transpose(rotation matrix along z) * (translation matix)
    #
    #                                         	Cos(alpha)     -Sin(alpha)      0
    #         Rotation Matrix           =      	Sin(alpha)     	Cos(alpha)      0
    #                                          	0              	0             	1
    #
    #                                        	Cos(alpha)     	Sin(alpha)      0
    #  transpose(Rotation Matrix)  		=      -Sin(alpha)     	Cos(alpha)      0
    #                                      		0               0               1
    #
    #                                          	x1-x0
    #  translation matrix               =      	y1-y0
    #                                           0
    #
    #
    #                                           Cos(alpha)     	Sin(alpha)     	0              	x1-x0
    #  combination                      =     - Sin(alpha)      Cos(alpha)      0       *    	y1-y0
    #                                           0               0               1              	0
    #
    ###
    #
    #   rotation component             =     object_pose.rotation - reference_frame_pose.rotation
    #
    #
    # ####

    alpha = reference_frame_pose.rotation.angle_z.radians
    rotation = np.array([[math.cos(alpha), -math.sin(alpha), 0],
                         [math.sin(alpha), math.cos(alpha), 0],
                         [0, 0, 1]])

    translation = np.array([object_pose.position.x - reference_frame_pose.position.x,
                            object_pose.position.y - reference_frame_pose.position.y,
                            0])

    relativeposition = rotation.T.dot(translation)

    beta = object_pose.rotation.angle_z.radians

    relativerotation = beta - alpha

	# since we are working with 2D and hence z position could just be substituted as is.
    return cozmo.util.pose_z_angle(relativeposition[0], relativeposition[1], object_pose.position.z, cozmo.util.radians(relativerotation))


def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
