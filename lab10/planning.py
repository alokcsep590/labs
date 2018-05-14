
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
import time

class Node:  
    def __init__(self,coord):
        self.point = coord
        self.parent = None
        self.H = 0.
        self.G = 1000.

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
        pass # Your code here


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

