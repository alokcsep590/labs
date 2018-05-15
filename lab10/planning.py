
#author1:
#author2:

from sympy import *  
from copy import deepcopy 

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import time

class Node:  
    def __init__(self,coord):
        self.point = coord
        self.parent = None
        self.H = 0.
        self.G = math.inf

    def __str__(self):
        return str(self.point[0]) + "," + str(self.point[1]) + "," + str(self.H) + "," + str(self.G)

def check_points(point1,point2):  
    if (abs(point1[0]-point2[0])<0.01)&(abs(point1[1]-point2[1])<0.01):
        # print point1,point2, 'True'
        return True
    else:
        # print point1,point2, 'False'
        return False        
        
def printPath(path):
    for node in path:
        print(str(node))

def ifSetContainsPoint(set, point):
    for item in set:  
        if (abs(item.point[0]-point[0])<0.01)&(abs(item.point[1]-point[1])<0.01):
            return item
    return None
    
def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
        
    currentNode = Node(grid.getStart())
    goalNode = Node(grid.getGoals()[0])
    
    openSet = []
    closedSet = []
    
    currentNode.H = heuristic(currentNode.point,goalNode.point)
    currentNode.G = 0.0
    
    openSet.append(deepcopy(currentNode))
    grid.addVisited(currentNode.point)
    
    while len(openSet) > 0:
        currentNode = min(openSet,key=lambda o:o.H + o.G)
        grid.addVisited(currentNode.point)
        
        # if it's the goal, we retrace the path from parents and return it
        if check_points(currentNode.point,goalNode.point):
            path = []
            while currentNode.parent:
                path.append(deepcopy(currentNode))
                currentNode = currentNode.parent
            # the start node does not have a parent
            path.append(currentNode)
            #print("path=")
            #printPath(path)
            
            foundNodePath = path[::-1]
            foundPath = []
            for nodeInPath in foundNodePath:
                foundPath.append(nodeInPath.point)
            grid.setPath(foundPath)
            return

        openSet.remove(currentNode)
        closedSet.append(deepcopy(currentNode))
        
        for neighbor in grid.getNeighbors(currentNode.point):
            neighborNode = Node(neighbor[0])
            
            if (ifSetContainsPoint(closedSet, neighborNode.point)!= None):
                continue

            nodeInOpenSet = ifSetContainsPoint(openSet, neighborNode.point)    
            if (nodeInOpenSet == None) :
                openSet.append(neighborNode)
            else:  
                neighborNode = nodeInOpenSet
            
            tentative_g = currentNode.G + neighbor[1]  
            if (tentative_g >= neighborNode.G):
                continue

            neighborNode.G = currentNode.G + neighbor[1]
            neighborNode.H = heuristic(neighborNode.point,goalNode.point)
            neighborNode.parent = currentNode
        
def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    #return math.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    return (min(abs(current[0]-goal[0]),abs(current[1]-goal[1]))*math.sqrt(2.0)) + max(abs(current[0]-goal[0]),abs(current[1]-goal[1]))-min(abs(current[0]-goal[0]),abs(current[1]-goal[1]))
    #return abs(current[0]-goal[0]) + abs(current[1]-goal[1])
    #return 0

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    while not stopevent.is_set():
        scale = 15.0
        cube1 = robot.world.get_light_cube(1)
        cube2 = robot.world.get_light_cube(2)
        cube3 = robot.world.get_light_cube(3)
        
        print("cube1="+str(cube1))
        print("cube2="+str(cube2))
        print("cube3="+str(cube3))
        print("cube1="+str(cube1.is_visible))
        print("cube2="+str(cube2.is_visible))
        print("cube3="+str(cube3.is_visible))
        
        if cube1.is_visible:
            cube1x = math.ceil(cube1.pose.position.x/scale)
            cube1y = math.ceil(cube1.pose.position.y/scale)
            grid.addObstacle((cube1x, cube1y))
            for neighbor in grid.getNeighbors((cube1x, cube1y)):
                grid.addObstacle(neighbor[0])
            grid.addGoal((cube1x+3, cube1y+3))
            
        if cube2.is_visible:
            cube2x = math.ceil(cube2.pose.position.x/scale)
            cube2y = math.ceil(cube2.pose.position.y/scale)
            grid.addObstacle((cube2x, cube2y))
            for neighbor in grid.getNeighbors((cube2x, cube2y)):
                grid.addObstacle(neighbor[0])
        
        if cube3.is_visible:
            cube3x = math.ceil(cube3.pose.position.x/scale)
            cube3y = math.ceil(cube3.pose.position.y/scale)
            grid.addObstacle((cube3x, cube3y))
            for neighbor in grid.getNeighbors((cube3x, cube3y)):
                grid.addObstacle(neighbor[0])
        
        astar(grid, heuristic)
        for step in grid.getPath():
            robot.go_to_pose(Pose(step[0]*scale,step[1]*scale,0,angle_z=degrees(0))).wait_for_completed()
        
        #time.sleep(3)
        #cube1.pose.rotation.angle_z.degrees
        #robot.go_to_pose(Pose((grid.getGoals()[0][0]-6)*scale,(grid.getGoals()[0][1]-6)*scale,0,angle_z=degrees(45))).wait_for_completed()
        
        #robot.turn_in_place(degrees(20), speed=degrees(5)).wait_for_completed()
        #robot.drive_straight(distance_mm(200), speed_mmps(25)).wait_for_completed()
        
        #astar(grid, heuristic)
        #robot.go_to_pose(Pose(grid.getGoals()[0][0]*scale,grid.getGoals()[0][1]*scale,0,angle_z=degrees(0))).wait_for_completed()
        
        #print(grid.getGoals())



######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

