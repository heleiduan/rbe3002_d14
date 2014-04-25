#!/usr/bin/env python

import rospy, tf, math, random, string, numpy, copy#, drive_module
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

    return currentCell.hScore
    
# return the updated Cell information used in return the list of neighbors
# BUG: direction does not do anything
def handleCell(x, y, parent, direction):
    
    global goalMapCell
    global mapCell

    currentCell = Cell(x, y, parent, 0, 0, 0)
    
    #determine whether the currentCell is obstacle or not
    if currentCell is not None and (mapCell[x][y] == 100):
        currentCell.fScore = 1000
        ###############################################print mapCell[5][2]
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
    if northEastCell.fScore != 1000: neighbors.append(northEastCell)
    
    northCell = handleCell(x, y + 1, currentCell, 'north')
    if northCell.fScore != 1000: neighbors.append(northCell)
    
    northWestCell = handleCell(x - 1, y + 1, currentCell, 'northWest')
    if northWestCell.fScore != 1000: neighbors.append(northWestCell)
    
    westCell = handleCell(x - 1, y, currentCell, 'west')
    if westCell.fScore != 1000: neighbors.append(westCell)
    
    southWestCell = handleCell(x - 1, y - 1, currentCell, 'southWest')
    if southWestCell.fScore != 1000: neighbors.append(southWestCell)
    
    southCell = handleCell(x, y - 1, currentCell, 'south')
    if southCell.fScore != 1000: neighbors.append(southCell)
    
    southEastCell = handleCell(x + 1, y - 1, currentCell, 'southEast')
    if southEastCell.fScore != 1000: neighbors.append(southEastCell)    
    
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
    print x
    print y
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
    print x
    print y
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
    
    convertMap(occupancyValue)
    expandObstaclesAndOptimizeGrid(0.2, 4)
    
# convert the map into a 37*37 matrix
def convertMap(occupancyValue):
    
    print "START CONVERT MAP"
    global mapCell
    global mapHeight
    global mapWidth
    global map

    occupancyValueIndex = 0
    mapCell = ndarray((mapWidth, mapHeight))

    for y in range(mapHeight):
        for x in range(mapWidth):
            mapCell[x][y] = int(occupancyValue[occupancyValueIndex])
            occupancyValueIndex = occupancyValueIndex + 1
    
    print "FINISH CONVERT MAP"
    
# expand the obstacles for ideal c-space for robot
def expandObstaclesAndOptimizeGrid(robotSize, optimizeRate):

    global map
    global newMapPub
    global mapCell
    expandedObstacle = list()
    
    print "START EXPAND OBSTACLES"
    
    expandObstaclesbyRange = int(math.ceil(robotSize / map.info.resolution))

    newMapData = list(map.data)
    
    #Save obstacles into a Cell list
    for x in range(0, map.info.width ):
        for y in range(0, map.info.height ):
            if(mapCell[x][y] == 100):
                for i in range(-expandObstaclesbyRange, expandObstaclesbyRange + 1):
                    for j in range(-expandObstaclesbyRange, expandObstaclesbyRange + 1):
                        if x < map.info.width-1 and y < map.info.height-1:
                            expandedObstacle.append(Cell(x + i, y + j, None, 0, 0, 0))
                            
    # check obstacles cells and write 100 into each cell
    for x in range(0, map.info.width):
        for y in range(0, map.info.height):
            for obstacle in expandedObstacle:
                if obstacle.x == x and obstacle.y == y:
                    mapCell[x][y] = 100
                
    print "FINISH EXPAND OBSTACLES" 
    
    optimizedMap = OccupancyGrid()
    optimizedMap.header = map.header
    optimizedMap.info = map.info
    
    '''for j in range(0,map.info.height):
            for i in range(0,map.info.width):
                optimizedMap.data.append(mapCell[(i,j)])'''
    
    print "START OPTIMIZE MAP"
#     newMapCelList = []
    #mapCell[0].append(100);
    
    
    #newRow = [ 100,100,100 ... 100]
    #mapCell.append(newRow);
    # generate correct height and width from optimization rate
    if (map.info.height % optimizeRate != 0): newHeight = map.info.height + optimizeRate - (map.info.height % optimizeRate)
    if (map.info.width % optimizeRate != 0): newWidth = map.info.width + optimizeRate - (map.info.width % optimizeRate)
    
    # add additional elements to the array 
    for i in range(0, map.info.width):
        for j in range(map.info.height, newHeight+1):
            print "====1"
            print str(i)+", "+str(j)
            mapCell[i][j] = 100
            
    for j in range(map.info.height, newHeight+1):
        for i in range(0, newWidth):
            print "====2"
            print str(i)+", "+str(j)
            mapCell[i][j] = 100
            
            
    for j in range(0, map.info.height, optimizeRate):
        for i in range(0, map.info.width, optimizeRate):
            optimizedMap.data.append(checkLargeGrid(mapCell,optimizeRate,i,j))
                
                
    optimizedMap.info.width = optimizeRate / optimizedMap.info.width
    optimizedMap.info.height = optimizeRate / optimizedMap.info.height
    optimizedMap.info.resolution = optimizeRate * optimizedMap.info.resolution


    newMapPub.publish(optimizedMap)
    

    print optimizedMap.data
#     print newMapCelList[0][0]
#     print newMapCelList[1][1]
#     print newMapCelList[2][2]
#     print newMapCelList[3][3]
    #rospy.sleep(rospy.Duration(4,0))
    print "FINISH UPDATING MAP"
    
def checkLargeGrid(cell,by,i,j):
    global mapCell
    #print "erere"
    #print str(mapCell[0])
    cells = []
    for k in range(0,by):
        for l in range(0,by): 
            print "i+k:" + str(i+k) + " j+l:" + str(j+l)
            print "i:" + str(i) + " k:" + str(k)
            
            try:
                 cells.append(mapCell[i+k][j+l])
            except IndexError as ex:
                continue
            
    val = max(cells)
    return val

'''def checkLargeGrid(possibleStates,by,i,j,compare):
    for k in range(0,by):
        for l in range(0,by): 
            if not (possibleStates[(i+k,j+l)] >= compare):
                 return False
    return True'''
    
                 
def displayFrontier():
    
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global map
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution 
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH frontier cells
    array = []
    for i in frontierCells:
        x = (i.x - map.info.width/2 + 3) * mapResolution 
        y = (i.y - map.info.height/2 + 3) * mapResolution
        array.append(Point(x,y,0))
    cells.cells = array
    frontierPub.publish(cells)
    
def displayExpanded():
    
    global path
    global expandPub
    global markerPub
    global pathPub
    global wayPub
    global map
    global frontierPub
    
    cells = GridCells()
    cells.cell_height = map.info.resolution 
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH explored cells
    array = []
    for i in expandedCells:
        x = (i.x - map.info.width/2 + 3) * mapResolution 
        y = (i.y - map.info.height/2 + 3) * mapResolution

        array.append(Point(x,y,0))
    cells.cells = array
    expandPub.publish(cells)
    
def displayExpandedObstacles():
    
    global newMapData
    
    cells = GridCells()
    cells.cell_height = map.info.resolution 
    cells.cell_width = map.info.resolution
    cells.header.frame_id = "map"
    
    #PUBLISH explored cells
    array = []
    for i in newMapData:
        x = (i.x - map.info.width/2 + 3) * mapResolution 
        y = (i.y - map.info.height/2 + 3) * mapResolution

        array.append(Point(x,y,0))
    cells.cells = array
    expandedObstaclesPub.publish(cells)
    
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
    array = []
    for i in path:
        x = (i.x - map.info.width/2 + 3) * mapResolution 
        y = (i.y - map.info.height/2 + 3) * mapResolution

        array.append(Point(x,y,0))
    cells.cells = array
    pathPub.publish(cells)
    
# This function is to follow the path and find out all the waypoints
def checkWayPoints():
    global path
    global wayPoints
    
    x_diff = path[1].x - path[0].x
    y_diff = path[1].y - path[0].y
    wayPoints.append(path[0])
    
    for i in range(1, len(path)-1):
        
       x_diff_temp = path[i+1].x - path[i].x
       y_diff_temp = path[i+1].y - path[i].y
       
       if (x_diff_temp != x_diff) or (y_diff_temp != y_diff):
           wayPoints.append(path[i])
           
       x_diff = x_diff_temp
       y_diff = y_diff_tem         

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
    array = []
    for i in wayPoints:
        x = (i.x - map.info.width/2 + 3) * mapResolution 
        y = (i.y - map.info.height/2 + 3) * mapResolution

        array.append(Point(x,y,0))
    cells.cells = array
    wayPub.publish(cells)

# ---------------------------------------------------------------------------------------------------
# This is the program's main function

if __name__ == '__main__':

    rospy.init_node('lab3')

    global pi
    global header
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
