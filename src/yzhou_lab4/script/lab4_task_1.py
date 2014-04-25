#!/usr/bin/env python

import rospy, tf, math, random, string, numpy, drive_module
from nav_msgs.msg import OccupancyGrid 
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from numpy import ndarray

#This function takes the start cell and goal cell, 
#then calculates the shortest path using A* algorithm
def aStar(startCell, goalCell):
    
    global frontierCells
    global expandedCells
    
    closelist = []
    openlist = []
    
    startCell.gScore = 0
    startCell.hScore = getHeuristic(startCell, goalCell)
    startCell.fScore = startCell.hScore + startCell.gScore
    
    openlist.append(startCell)
    frontierCells.append(startCell)
    
    while len(openlist) != 0:       #execute only if the expended cell still exists
        # select the expanded cell which has minimum F score in the list
        current = openlist[0]
        for cell in openlist:
            if current.fScore > cell.fScore:
                current = cell
        # return the best path to current cell
        if (current.x == goalCell.x) and (current.y == goalCell.y):
            return reconstruct_path(current) 
        
        openlist.remove(current)    #delete cell from the expanded list
        closelist.append(current)   #add cell to closed list
        expandedCells.append(current)
        expandedCells.append(current)
        #print expandedCells
        # calculate scores for the neighbor cells
        for neighbor in neighborCells(current):
            # ingore it if neighbor cell has been examined
            flag = 0;
            
            for cell in closelist:
                if (neighbor.x == cell.x) and (neighbor.y == cell.y):
                    flag = 1
            if flag == 1:
                continue
            # g score from start cell to neighbor
            tentative_g_score = current.gScore + dist_between(current, neighbor)
            
            flag2 = 0;
            for cell in openlist:
                if (neighbor.x == cell.x) and (neighbor.y == cell.y):
                    flag2 = 1
            if (flag2 == 0) or (tentative_g_score < neighbor.gScore):
                neighbor.parent = current
                neighbor.gScore = tentative_g_score
                neighbor.hScore = getHeuristic(neighbor, goalCell)
                neighbor.fScore = neighbor.gScore + neighbor.hScore
                if (flag2 == 0):
                    openlist.append(neighbor)
                    frontierCells.append(neighbor)
                    #print frontierCells
    return -1

#This function takes one cell and its neighbor cell and calculates the g score between the two cells
def dist_between(currentCell, neighborCell):
    
    x_diff = neighborCell.x - currentCell.x
    y_diff = neighborCell.y - currentCell.y
    
    if (x_diff == 1) and (y_diff == 1):
        return math.sqrt(2)
    if (x_diff == -1) and (y_diff == 1):
        return math.sqrt(2)
    if (x_diff == -1) and (y_diff == -1):
        return math.sqrt(2)
    if (x_diff == 1) and (y_diff == -1):
        return math.sqrt(2)
    
    if (x_diff == 0) and (y_diff == 1):
        return 1
    if (x_diff == -1) and (y_diff == 0):
        return 1
    if (x_diff == 0) and (y_diff == -1):
        return 1
    if (x_diff == 1) and (y_diff == 0):
        return 1

#This function takes the current cell and construct a path from start to current cell
def reconstruct_path(current):
    global path
    
    path.append(current)
    
    if current.parent == None:
        #print path
        return path
    else:
        return reconstruct_path(current.parent)
    
class Cell:
    # initialize the cell with its x, y position, parent cell and g cost value
    def __init__(self, xPosition, yPosition, parent, gScore, hScore, fScore):
        self.x = xPosition
        self.y = yPosition
        self.parent = parent
        self.gScore = gScore
        self.hScore = hScore
        self.fScore = fScore
        
    # determine if two cells are equal to each other
    def _equal(self, anotherCell):
        if self.x == anotherCell.x and self.y == anotherCell.yPosition:
            return True
        else: 
            return False
            
# heuristic_cost_estimate         
def getHeuristic(currentCell, goalMapCell): #Straight line distance between the two cells

    currentCell.hScore = round(math.sqrt( ((goalMapCell.y - currentCell.y)**2) + ((goalMapCell.x - currentCell.x)**2) ),4)

    #print currentCell.hScore
    return currentCell.hScore
    
# return the updated Cell information used in return the list of neighbors
# BUG: direction does not do anything
def handleCell(x, y, parent, direction):
    
    global goalMapCell
    global mapCell

    currentCell = Cell(x, y, parent, 0, 0, 0)
    
    #determine whether the currentCell is obstacle or not
    if currentCell is not None and mapCell[x][y] == 100:

        currentCell.fScore = 1000               
    
    #print currentCell.fScore
    
    return currentCell
    
# This function takes the currentCell and return a list of neighbors of currentCell
def neighborCells(currentCell):

    global occupancyValue
    global map
    global mapCell
    global mapHeight
    global mapWidth
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    
    neighbors = list()

    x = currentCell.x
    y = currentCell.y
    
    eastCell = handleCell(x + 1, y, currentCell, 'east')
    if eastCell.fScore != 1000: neighbors.append(eastCell)
    
    northEastCell = handleCell(x + 1, y + 1, currentCell, 'northEast')
    if northEastCell.fScore  != 1000: neighbors.append(northEastCell)
    
    northCell = handleCell(x, y + 1, currentCell, 'north')
    if northCell.fScore  != 1000: neighbors.append(northCell)
    
    northWestCell = handleCell(x - 1, y + 1, currentCell, 'northWest')
    if northWestCell.fScore  != 1000: neighbors.append(northWestCell)
    
    westCell = handleCell(x - 1, y, currentCell, 'west')
    if westCell.fScore  != 1000: neighbors.append(westCell)
    
    southWestCell = handleCell(x - 1, y - 1, currentCell, 'southWest')
    if southWestCell.fScore  != 1000: neighbors.append(southWestCell)
    
    southCell = handleCell(x, y - 1, currentCell, 'south')
    if southCell.fScore  != 1000: neighbors.append(southCell)
    
    southEastCell = handleCell(x + 1, y - 1, currentCell, 'southEast')
    if southEastCell.fScore  != 1000: neighbors.append(southEastCell)    
    
    return neighbors


    
# Start position Callback Function 
def readStartCellCallback(msg):

    global startMapCell
    global mapPose
    global mapResolution
    
    # convert into cell representation 
    x = msg.pose.pose.position.x #+ mapResolution / 2
    y = msg.pose.pose.position.y #+ mapResolution / 2
    
    #print x
    #print y
    
    x = int(round((x - mapPose.position.x) / mapResolution))
    y = int(round((y - mapPose.position.y) / mapResolution))
    
    print "Got Start"
    startMapCell = Cell(x, y, None, 0, 0, 0)
    #print startMapCell.x
    #print startMapCell.y
    if goalMapCell and startMapCell != None:
        aStar(startMapCell, goalMapCell)
        displayFrontier()
        displayExpanded()
        displayPath()
        displayWayPoints()
    else:
        print 'Assign Goal and Start'
        
    
def readGoalCellCallback(msg):
    
    global goalMapCell
    global mapPose
    global mapResolution
    
    # convert into cell representation 
    x = msg.pose.position.x - mapResolution / 2
    y = msg.pose.position.y - mapResolution / 2
    
    x = int(round((x - mapPose.position.x) / mapResolution))
    y = int(round((y - mapPose.position.y) / mapResolution))
    
    print "Receive Goal"
    goalMapCell = Cell(x, y, None, 0, 0, 0)
    #print goalMapCell.x
    #print goalMapCell.y
    if goalMapCell and startMapCell != None:
        aStar(startMapCell, goalMapCell)
        displayFrontier()
        displayExpanded()
        displayPath()
        displayWayPoints()
    else:
        print 'Assign Goal and Start'
        
# OccupancyGrid Callback Function - readMapCallback
def readMapCallback(msg):

    global header
    global occupancyValue
    global map
    global mapHeight
    global mapWidth
    global mapPose
    global mapResolution
    
    map = msg

    occupancyValue = map.data
    mapHeight = map.info.height
    mapWidth = map.info.width
    mapResolution = map.info.resolution
    mapPose = map.info.origin

# convert the map into a 37*37 matrix
def convertMap(occupancyValue):

    global mapCell

    occupancyValueIndex = 0
    mapCell = ndarray((mapWidth, mapHeight))
    
    for y in range(mapHeight):
        for x in range(mapWidth):
            mapCell[x][y] = int(occupancyValue[occupancyValueIndex])
            occupancyValueIndex = occupancyValueIndex + 1


def cellsToPoints(cells):
    array = []
    for i in cells:
        x = (i.x * map.info.resolution) + map.info.resolution/2 + map.info.origin.position.x 
        y = (i.y * map.info.resolution) + map.info.resolution/2 + map.info.origin.position.y
        array.append(Point(x,y,0))
    return array


def displayFrontier():
    
    global map
    global mapCell
    global mapHeight
    global mapWidth
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    global pathCells
    global expandedCells
    global wayPoints
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution 
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH frontier cells
    cells.cells = cellsToPoints(frontierCells)
    frontierPub.publish(cells)
    
    

    
def displayExpanded():
    
    global map
    global mapCell
    global mapHeight
    global mapWidth
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    global pathCells
    global expandedCells
    global wayPoints
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution 
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH explored cells
    cells.cells = cellsToPoints(expandedCells)
    expandPub.publish(cells)


def displayPath():

    global map
    global mapCell
    global mapHeight
    global mapWidth
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    global pathCells
    global expandedCells
    global wayPoints
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH path cells
    cells.cells = cellsToPoints(path)
    pathPub.publish(cells)
    
    
# This function is to follow the path and find out all the waypoints
def checkWayPoints():
    global path
    global wayPoints
    
    x_diff = path[1].x - path[0].x
    y_diff = path[1].y - path[0].y
    
    for i in range(1, len(path)-1):
        
       x_diff_temp = path[i+1].x - path[i].x
       y_diff_temp = path[i+1].y - path[i].y
       
       if (x_diff_temp != x_diff) or (y_diff_temp != y_diff):
           wayPoints.append(path[i])
           
       x_diff = x_diff_temp
       y_diff = y_diff_temp
        
    print wayPoints
            

def displayWayPoints():
    
    global map
    global mapCell
    global mapHeight
    global mapWidth
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    global pathCells
    global expandedCells
    global wayPoints
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #call to check waypoints
    checkWayPoints()
    
    #PUBLISH waypoint cells
    cells.cells = cellsToPoints(wayPoints)
    wayPub.publish(cells)


# ------------------------------- Functions below are for Lab 4 -------------------------------------

# This helper function returns the angle between the two cells
def calAngle(thisCell, nextCell):   ###
    return math.atan2((nextCell.y - thisCell.y), (nextCell.x - thisCell.x))
    
# This helper function is to calculate the distance that robot is going to drive
def calDistance(thisCell, nextCell):    ###
    return math.sqrt((nextCell.y*0.2 - thisCell.y*0.2)**2 + (nextCell.x*0.2 - thisCell.x*0.2)**2)


# TODO
# This function will navigate robot along the astar path until it reaches the goal
def navigateToGoal():   ###
    global path
    global wayPoints
    
    lastAngle = 0  #the starting angle of robot
    
    for index in range(len(wayPoints)-1):
        thisAngle = calAngle(wayPoints[index], wayPoints[index+1])
        print "Absolute Angle" + str(thisAngle)
        print " "
        print "last angle" + str(lastAngle)
        print " "
        print "xiangdui Angle" + str(thisAngle - lastAngle)
        print " "
        distance = calDistance(wayPoints[index], wayPoints[index+1])
        print "Distance" + str(distance)
        print " "
        
        # first rotate the robot to an angle and then drive it to a distance
        drive_module.rotate(thisAngle - lastAngle)
        drive_module.driveStraight(0.5, distance)
        
        lastAngle = thisAngle
    

# TODO
# This function will expand the obstacles the occupancy grid so that the robot does not attempt to
# drive through such gaps
#def expandObstacle():

# TODO
# This function will change the occupancy grid size to better suit the TurtleBot's capabilities



# ---------------------------------------------------------------------------------------------------
# This is the program's main function
if __name__ == '__main__':  ###

    drive_module.run()
   

    global pi
    global header
    global map
    global mapCell
    global mapPose
    global startMapCell
    global goalMapCell
    global frontierCells
    global expandedCells
    global neighbors
    global pathCells
    global expandedCells
    global wayPoints
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global frontierPub
    global expandedObstacles
    global newMapPub

    pi = math.pi
    r = rospy.Rate(10)
    path = list()
    frontierCells = list()
    expandedCells = list()
    wayPoints = list()
    expandedObstacles = list()
    
    # subscribe the occupancy grid message under map topic
    subMap = rospy.Subscriber('map', OccupancyGrid, readMapCallback, queue_size=1)
    subStartCell = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStartCellCallback, queue_size=1)
    subGoalCell = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoalCellCallback, queue_size = 10)
    
    # publish updated map information
    expandPub = rospy.Publisher('expandedCells',GridCells, queue_size = None)
    frontierPub = rospy.Publisher('frontierCells',GridCells, queue_size = None)
    pathPub = rospy.Publisher('pathCells',GridCells, queue_size = None)
    wayPub = rospy.Publisher('wayPoints',GridCells, queue_size = None)
    expandedObstaclesPub = rospy.Publisher('expandedObstacles',GridCells, queue_size = None)
    newMapPub = rospy.Publisher('optimizedMap', OccupancyGrid, latch=True)

    #subCost = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readCostMap, queue_size=1)
    #costPub = rospy.Publisher('/costOptimize',OccupancyGrid, latch = True)
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 4"
    
    #convertMap(occupancyValue)
    #print dist_between(startMapCell, cell1)
    #readMapCallback(OccupancyGrid())
    #startMapCell = Cell(3, 3, None, 0, 0, 0)
    #goalMapCell = Cell(15, 8, None, 0, 0, 0)
    
    #aStar(startMapCell, goalMapCell)
    #displayExpandedandFrontierCells()
    #displayPath()
    #displayExpandedObstacles()
    #neighborCells(goalMapCell)
    #getHeuristic(startMapCell, goalMapCell)
    #print neighbors[6].hScore
    #print neighbors[6].gScore
    #print neighbors[6].fScore
    
    #rospy.spin()
    print "Lab 4 complete!"
