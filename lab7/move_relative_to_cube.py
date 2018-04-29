#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys
import numpy as np

from odometry import cozmo_go_to_pose, my_go_to_pose1, my_go_to_pose2, my_go_to_pose3 
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

def move_relative_to_cube(robot: cozmo.robot.Robot):
    '''Looks for a cube while sitting still, when a cube is detected it 
    moves the robot to a given pose relative to the detected cube pose.'''

    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    cube = None

    while cube is None:
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=30)
            if cube:
                print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
        except asyncio.TimeoutError:
            print("Didn't find a cube")

    desired_pose_relative_to_cube = Pose(0, 100, 0, angle_z=degrees(90))
    
    # ####
    # TODO: Make the robot move to the given desired_pose_relative_to_cube.
    # Use the get_relative_pose function your implemented to determine the
    # desired robot pose relative to the robot's current pose and then use
    # one of the go_to_pose functions you implemented in Lab 6.
    # ####


    # We need to find desired pose relative to cube coordinates in World's frame.
    #   Multiply rotation matrix of cube with relative to cube pose.
    #   Add this to cube's pose to get world's frame.
    # Then find relative pose of desired pose to the robot's current pose.
    # Then use gotopose to move to the desired pose relative to cube.

    alpha = cube.pose.rotation.angle_z.radians
    rotation = np.array([[math.cos(alpha), -math.sin(alpha), 0],
                         [math.sin(alpha), math.cos(alpha), 0],
                         [0, 0, 1]])
    
    cubeposition = np.array([cube.pose.position.x, cube.pose.position.y, 0])
    
    relativetocubeposition = np.array([desired_pose_relative_to_cube.position.x, desired_pose_relative_to_cube.position.y, 0])
    
    desired_pose_world_frame_position = rotation.T.dot(cubeposition) + relativetocubeposition
    
    desired_pose_world_frame_angle_z = cube.pose.rotation.angle_z.degrees + desired_pose_relative_to_cube.rotation.angle_z.degrees
    
    desired_pose_world_frame = Pose(desired_pose_world_frame_position[0], desired_pose_world_frame_position[1], 0, angle_z=degrees(desired_pose_world_frame_angle_z))
    
    desired_pose_relative_to_robot = get_relative_pose(desired_pose_world_frame, robot.pose)
    
    my_go_to_pose3(robot, desired_pose_relative_to_robot.position.x, desired_pose_relative_to_robot.position.y, desired_pose_relative_to_robot.rotation.angle_z.degrees)
    

if __name__ == '__main__':

    cozmo.run_program(move_relative_to_cube)
