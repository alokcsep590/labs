#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

#import sys
#sys.path.insert(0, '../lab6')
#from pose_transform import get_relative_pose

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """
    robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius(robot):
    """Returns the radius of the Cozmo robot's front wheel in millimeters.
        Arguments:
        robot -- the Cozmo robot instance passed to the function        
    """
    
    # ####
    #
    # Drive cozmo straight and observe number of rotations of front wheel.
    # Using the distance driven and number of rotations calculate the radius.
    # Take avergage from multiple runs to get better estimate.
    # 
    # ####
    
    # Relationship between linear distance and rotations is
    #   NumRotations*2*pi*r = LinearDistance
    #   r = (LinearDistance)/(NumRotations*2*pi)

    cozmo_drive_straight(robot, 150, 30)
    NumObservedRotations = 1.8    
    r1 = 150/(1.8*2*math.pi)
    
    cozmo_drive_straight(robot, 150, 30)
    NumObservedRotations = 1.8
    r2 = 150/(1.7*2*math.pi)
    
    cozmo_drive_straight(robot, 150, 30)
    NumObservedRotations = 1.8
    r3 = 150/(1.8*2*math.pi)
                
    return (r1+r2+r3)/3

def get_distance_between_wheels(robot):
    """Returns the distance between the wheels of the Cozmo robot in millimeters.
        Arguments:
        robot -- the Cozmo robot instance passed to the function        
    """
    
    
    # ####
    # If robot is differentially driven using  lower speed on one wheel and higher speed on another wheel
    # it will circle (follow the arc of the circle) around some point.
    # From the trignometry we can know the destination coordinates will have below equation
    #   phi_left - rotations of left wheel in radians
    #   phi_right - rotations of right wheel in radians
    #      theta - angle_z of robot initially in radian
    #      r - radius of the wheel
    #      d - distance between wheels (we are interested in)
    #      x - x component of target position
    #      y - y component of target position
    #      thetatarget - angle_z of robot in target position
    #
    # Equation
    #
    #                               cos(theta)             -sin(theta)              0                                  
    #      rotation component    =  sin(theta)              cos(theta)              0      
    #                               0                       0                       1
    #
    #                               r*phi_left/2   +  r*phi_right/2  
    #      translation component  =         0
    #                               r*phi_right/d   -  r*phi_left/d
    #
    #
    #              x
    #              y                       =  rotation component*translation component
    #             thetatarget
    #
    #
    #  Experiment
    #           Use  drive_wheels with different left and right speed.
    #           Use initial and target pose.
    #           Use radius of the wheel found earlier.
    #           Key in these numbers in the equation above to solve for d i.e. distance between wheels.
    #
    # ####
    
    
    #####
    #
    #   Solving above equations by inverse (from Correll book, equation 3.64) 
    #
    #   phi_left = (2*xR - thetatarget*d)/(2*r)
    #
    #   Rearranging to get
    #       
    #   d = (2*xR - 2*r*phi_left)/thetatarget
    #
    #####
    
    initialpose = robot.pose
    
    robot.drive_wheels(60, 30, None, None, 3)
    
    targetpose = robot.pose
    
    r = 13 # get_front_wheel_radius(robot)
    
    # phi_left is obtained by observations as how many radians are turned by left wheel
    phi_left = 6
    
    thetatarget = initialpose.rotation.angle_z.radians - targetpose.rotation.angle_z.radians
    
    xR = (targetpose.position.x - initialpose.position.x)*math.cos(initialpose.rotation.angle_z.radians) + (targetpose.position.y - initialpose.position.y)*math.sin(initialpose.rotation.angle_z.radians)
    d = (2*xR - 2*r*phi_left)/thetatarget
    
    return d

def rotate_front_wheel(robot, angle_deg):
    """Rotates the front wheel of the robot by a desired angle.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle_deg -- Desired rotation of the wheel in degrees
    """
    
    # ####
    #
    # We know robot will move 2*pi*r in one revolution i.e. 360 degrees.
    #  angle_deg rotation means robot need to move (2*pi*r)*angle_deg/360
    #
    # ####
    
    r = 13 # get_front_wheel_radius(robot)
    
    dist = (2*math.pi*r*angle_deg)/360
    
    speed = 10
    
    robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def my_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    
    time = dist/speed
    robot.drive_wheels(speed, speed, None, None, time)

def my_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """
    # ####
    # Turn in place can be achieved using differential drive with same (magnitude) speed but opposite 
    # direction of right and left wheel.
    # 
    # For differential drive we have equation
    #   phi = angle of movement of robot in place
    #   r = radius of the wheel
    #   b = distance between wheels
    #   phi_left = angle of movement of left wheel
    #   phi_right = angle of movement of right wheel
    # 
    #   phi = (r*(phi_left - phi_right))/b
    # 
    # ####
    
    r = 13 # get_front_wheel_radius(robot)
    
    b = 49 # get_distance_between_wheels(robot)
    
    wheel_angular_speed = (b*angle)/(2*r)
    
    wheel_linear_speed = ((2*math.pi*r)/360)*wheel_angular_speed
    
    time = angle/speed
    
    robot.drive_wheels(wheel_linear_speed, -wheel_linear_speed, None, None, time)
    

def my_go_to_pose1(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # using the my_drive_straight and my_turn_in_place functions. This should
    # include a sequence of turning in place, moving straight, and then turning
    # again at the target to get to the desired rotation (Approach 1).
    # ####
    
    # ####
    # let's turn in place with certain speed. this may result in slight more or less desired rotation.
    # drive straight for distance between two points.
    # turn in place again at target to align with target.
    # ####
    
    angularspeed = 20
    
    my_turn_in_place(robot, angle_z, angularspeed)
    
    distance = math.sqrt(x*x + y*y)
    
    linearspeed = 10
    
    my_drive_straight(robot, distance, linearspeed)
    
    angle_z_correction = robot.pose.rotation.angle_z.degrees - angle_z
    
    my_turn_in_place(robot, angle_z_correction, angularspeed)
    
def my_go_to_pose2(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # using the robot.drive_wheels() function to jointly move and rotate the 
    # robot to reduce distance between current and desired pose (Approach 2).
    # ####
    
    
    ####
    #
    #   We need to find the differential speeds of left and right wheel and the time 
    #   duration so that robot will go around some point i.e. it will follow the arc by moving
    #   and rotating.
    #
    #
    #
    #                               cos(theta)              - sin(theta)                0                                  
    #      rotation component    =  sin(theta)                 cos(theta)               0      
    #                               0                            0                            1
    #
    #                                   r*phi_left/2   +  r*phi_right/2  
    #      translation component  =             0
    #                                   r*phi_right/d   -  r*phi_left/d
    #
    #
    #              x
    #              y                       =  rotation component*translation component
    #             thetatarget
    #
    ####
    
    #####
    #
    #   Solving above equations by inverse (from Correll book, equation 3.64) 
    #
    #   phi_left = (2*xR - thetatarget*d)/(2*r)
    #   
    #   phi_right = (2*xR + thetatarget*d)/(2*r)
    #   
    #
    #####
    
    d = 49 # get_distance_between_wheels(robot)
    
    r = 13 # get_front_wheel_radius(robot)
    
    thetatarget = (angle_z*2*math.pi)/360
     
    xR = ((x)*math.cos(thetatarget)) + ((y)*math.sin(thetatarget))

    
    phi_left = (2*xR - thetatarget*d)/(2*r)
    phi_right = (2*xR + thetatarget*d)/(2*r)
    
    t = 5
    
    angularspeed_left = phi_left/t
    angularspeed_right = phi_right/t
    
    linearspeed_left = angularspeed_left*r
    linearspeed_right = angularspeed_right*r
    
    print("xR = " + str(xR))
    print("phi_left = " + str(phi_left))
    print("phi_right = " + str(phi_right))
    print("angularspeed_left = " + str(angularspeed_left))
    print("angularspeed_right = " + str(angularspeed_right))
    
    robot.drive_wheels(linearspeed_left, linearspeed_right, None, None, t)
    
def my_go_to_pose3(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # as fast as possible. You can experiment with the built-in Cozmo function
    # (cozmo_go_to_pose() above) to understand its strategy and do the same.
    # ####
    
    # Since target is static that is robot wants to reach fixed destination, we
    # will choose to turn in place and move straight with maximum speeds.
    
    angularspeed = 30
    
    my_turn_in_place(robot, angle_z, angularspeed)
    
    distance = math.sqrt(x*x + y*y)
    
    linearspeed = 55
    
    my_drive_straight(robot, distance, linearspeed)
    
def run(robot: cozmo.robot.Robot):

    print("***** Front wheel radius: " + str(get_front_wheel_radius(robot)))
    print("***** Distance between wheels: " + str(get_distance_between_wheels(robot)))

    ## Example tests of the functions

    cozmo_drive_straight(robot, 62, 50)
    cozmo_turn_in_place(robot, 60, 30)
    cozmo_go_to_pose(robot, 100, 100, 45)

    rotate_front_wheel(robot, 90)
    my_drive_straight(robot, 62, 50)
    my_turn_in_place(robot, 90, 30)

    my_go_to_pose1(robot, 100, 100, 45)
    my_go_to_pose2(robot, 100, 100, 45)
    my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

    cozmo.run_program(run)



