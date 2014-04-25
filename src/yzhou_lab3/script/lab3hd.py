#!/usr/bin/env python

import rospy, tf, math, random
from nav_msgs.msg import OccupancyGrid 
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from numpy import sign, ndarray

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
    def _equal(self, anotherCell)
        if self.x = anotherCell.x and self.y = anotherCell.yPosition:
            return True
        else: 
            return False

'''# calculate the distance from current cell to another cell
def cellFromPose(pose):    
     return int((pose.position.x - map.info.origin.position.x)/map.info.resolution), int((pose.position.y - map.info.origin.position.y)/map.info.resolution)
     
# calculate heuristic values from current cell
def getHeuristic(node):
    goalCell = cellFromPose(goalPose)
    return math.sqrt((goalCell[0] - node[0]) ** 2 + (goalCell[1] - node[1]) ** 2)
    
# read the start cell from massage
def readStartCell(msg):
    global initialPose
    initialPose = msg.pose.pose
    initialCell = cellFromPose(initialPose)
    lightWayCell(initialCell[0], initialCell[1])   

# read the goal cell from massage
def readGoalPose(msg):
    global goalPose 
    goalPose = msg.pose
    expandedCells = GridCells()
    goalCell = cellFromPose(goalPose)
    lightWayCell(goalCell[0], goalCell[1])
    path = runAStarSearch()
    #print(path)
    print computeWayPoints(path)
    showOnlyPathCells()'''
    
# convert the map into a 37*37 matrix
def convertMap(occupancyValue):

    global mapPoint
    occupancyValueIndex = 0
    mapPoint = ndarray((mapWidth, mapHeight))
    
    for y in range(mapHeight):
            for x in range(mapWidth):
                mapPoint[x][mapHeight - y - 1] = int(occupancyValue[occupancyValueIndex])
                occupancyValueIndex = occupancyValueIndex + 1
    #expandPub.publish(mapPoint)
    
def display():
    global occupancyValue
    global map
    global mapHeight
    global mapWidth
    global mapPoint
    
    convertMap(occupancyValue)
    print 'Cell value is'
    print mapPoint[0][36]
    
# OccupancyGrid Callback Function - ReadMap
def readMap(msg):

    global header
    global occupancyValue
    global map
    global mapHeight
    global mapWidth
    map = msg

    occupancyValue = map.data
    mapHeight = map.info.height
    mapWidth = map.info.width

# This is the program's main function
if __name__ == '__main__':

    rospy.init_node('lab3')

    global pi
    global header
    global occupancyValue
    global map
    global goalPose 
    global initialPose
    global mapPoint
    global mapHeight
    global mapWidth
    
    pi = math.pi
    r = rospy.Rate(10)

    # subscribe the occupancy grid message under map topic
    occupationGrid = rospy.Subscriber('map', OccupancyGrid, readMap, queue_size=1)
    expandPub = rospy.Publisher('expandedCells',GridCells, queue_size = None)
    publisher = rospy.Publisher('visualization_marker_array', MarkerArray)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 3"
    #overlayRandomTiles()
    #printMap()
    #convertMap(occupancyValue)
    display()
    print "Lab 3 complete!"
