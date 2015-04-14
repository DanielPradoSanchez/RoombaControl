#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg #check if this is possible
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2, LaserScan #, PointField
import sensor_msgs.point_cloud2 as pc2

import numpy as np
import re


# look at nav-msgs for occupancy grid like structure
class occupancyGrid():
    def __init__(self):
        # n rows, m columns
        # 500 width
        # 300 height
        self.n = 300 # set this manually, 300
        self.m = 500 # set this manually, 500
        self.grid = np.zeros((self.n,self.m))
        self.occupancyGrid = np.zeros((self.n,self.m))
        self.mapAsString = ""
        self.nonZeroCells = set()
        self.permanentlyOccupiedCellCoordinatesWidthLength = [[1,1,2,2],[4,0,2,5]] # Needs to be set manually
        self.permanentlyOccupiedCells = self.generatePermanentlyOccupiedCells(self.permanentlyOccupiedCellCoordinatesWidthLength)
        for cell in self.permanentlyOccupiedCells:
            x = cell[0]
            y = cell[1]
            self.occupancyGrid[y][x] = 100


    def generatePermanentlyOccupiedCells(self,permanentlyOccupied):
        setOfTuples = set()
        for cell in permanentlyOccupied:
            x = cell[0]
            y = cell[1]
            width = cell[2]
            length = cell[3]
            for dx in range(width):
                for dy in range(length):
                    newTuple = (x+dx,y+dy)
                    setOfTuples.add(newTuple)
        return setOfTuples
    

    def parseMap(self,theMap):
        splitMap = filter(None,re.split("/+",theMap)) #split by / and also remove empty spaces created by split    
        mapping = {}
        for objectAndData in splitMap:
            objectAndDataAsList = filter(None,objectAndData.split(" ")) #split by " " and remove empt spaces on list
            objectType = objectAndDataAsList[0]
            dataAsString = objectAndDataAsList[1:]
            data = []
            for i in dataAsString:
                data.append(int(float(i)))
            if objectType in mapping.keys():
                mapping[objectType].append(data)
            else:
               mapping[objectType] = [data]
        return mapping

    def convertMappingToOccupancyGrid(self,mapping):
        n = self.n
        m = self.m
        parsedMap = self.parseMap(mapping)
        newGrid = set()
        for typeOfObject in parsedMap:
            for currentObject in parsedMap[typeOfObject]:
                if typeOfObject is "Table":
                    x = currentObject[0]
                    width = currentObject[2]
                    length = width
                    y = currentObject[1] + currentObject[3] - width
                    
                else:
                    x = currentObject[0]
                    y = currentObject[1]
                    width = currentObject[2]
                    length = currentObject[3]
                for i in range(width):
                    for j in range(length):
                        if x + i >= m or y + j >= n:
                            pass
                        else:
                            newGrid.add((x + i,y + j))
                            self.nonZeroCells.add((x + i,y + j))
        return newGrid

    def generateBlankOccupancyGrid(self):
        # n rows, m columns
        newGrid = np.zeros((self.n,self.m))
        return newGrid


    def updateOccupancyGrid(self, mapping):
        # n rows, m columns
        increase = 25
        decrease = 40
        currentlyOccupied = self.convertMappingToOccupancyGrid(mapping)
        for previousNonZeroCell in self.nonZeroCells:
            x = previousNonZeroCell[0]
            y = previousNonZeroCell[1]
            if previousNonZeroCell in currentlyOccupied:
                self.occupancyGrid[y][x] += increase
                currentlyOccupied.remove(previousNonZeroCell)
                if self.occupancyGrid[y][x] > 100:
                    self.occupancyGrid[y][x] = 100
            elif previousNonZeroCell not in self.permanentlyOccupiedCells:
                self.occupancyGrid[y][x] += -decrease
                if self.occupancyGrid[y][x] < 0:
                    self.occupancyGrid[y][x] = 0
            
        for currentlyOccupiedCell in currentlyOccupied:
            x = currentlyOccupiedCell[0]
            y = currentlyOccupiedCell[1]
            self.occupancyGrid[y][x] += increase
            if self.occupancyGrid[y][x] > 100:
                self.occupancyGrid[y][x] = 100

    #def publish(self):
    #    pub = rospy.Publisher('gridPub', OccupancyGrid, queue_size=10)
    #    rate = rospy.Rate(10) #appropriate rate?
    #    while not rospy.is_shutdown():
    #       pub.publish(self.occupancyGrid)
    #       rate.sleep()

    def callback(self, data):
        self.mapAsString = data.data
        self.updateOccupancyGrid(self.mapAsString)

        #ros message header
        occGrid = OccupancyGrid(header=rospy.Header())
        occGrid.header.stamp = rospy.Time.now()
        occGrid.header.frame_id = "map"

        #ros metadata
        occGrid.info.resolution  = 1.0
        occGrid.info.height = self.n
        occGrid.info.width = self.m
        occGrid.info.origin = Pose(Point(-5.0, -5.0, 0.), Quaternion(0.0,0.0,0.0,1.0))

        #2d to 1d iterator
        it = self.occupancyGrid.flat
        occGrid.data = list(it)

        #publish to ros
        self.pub.publish(occGrid)
        #print self.occupancyGrid


        ##now send a point cloud msg
        header=rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_laser_link"

        # fields = [pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1), 
        #           pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1), 
        #           pc2.PointField("z" , 8, pc2.PointField.FLOAT32, 1), 
        #           ]
        points = []
        for y in range(self.n):
            for x in range(self.m):
                points.append([x,y,0])

        cloud = pc2.create_cloud_xyz32(header, points)
        self.cloud.publish(cloud)
        #cloud = pc2.create_cloud(header, fields, data)

    def laserCallback(self, data):
        l = LaserScan()
        # l.header = header
        # l.angle_min= -2.35837626457
        # l.angle_max= 2.35837626457
        # l.angle_increment= 0.00436736317351
        # l.time_increment= 0.0
        # l.scan_time = 0.0
        # l.range_min = 0.0
        # l.range_max =2.0

        l.header = data.header
        l.angle_min= data.angle_min
        l.angle_max= data.angle_max
        l.angle_increment= data.angle_increment
        l.time_increment= data.time_increment
        l.scan_time = data.scan_time
        l.range_min = data.range_min
        l.range_max = data.range_max

        it = self.occupancyGrid.flat
        l.ranges = data.ranges
        l.intensities = list(it)
        self.baseLaserPub.publish(l)

    #def listen(self):
    #    rospy.Subscriber("machineVision", String, callback)
    #    # spin() simply keeps python from exiting until this node is stopped
    #    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('myName')
        grid = occupancyGrid()
        grid.pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        grid.cloud = rospy.Publisher('cloud_in', PointCloud2, queue_size=10)
        grid.baseLaserPub = rospy.Publisher('base_scan2', LaserScan, queue_size=10)
        rospy.Subscriber("machineVision", String, grid.callback)
        rospy.Subscriber("scan", LaserScan, grid.laserCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
       pass                
