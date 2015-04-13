import rospy
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg #check if this is possible

import numpy as np
import re


# look at nav-msgs for occupancy grid like structure
class occupancyGrid():
    def __init__(self):
        # n rows, m columns
        # 1920 width
        # 1080 height
        self.n = 1920 # set this manually
        self.m = 1080 # set this manually
        self.grid = np.zeros((self.n,self.m))
        self.occupancyGrid = np.zeros((self.n,self.m))
        self.mapAsString = ""
        self.nonZeroCells = set()

    def parseMap(self,theMap):
        splitMap = filter(None,re.split("/+",theMap)) #split by / and also remove empty spaces created by split    
        mapping = {}
        for objectAndData in splitMap:
            objectAndDataAsList = filter(None,objectAndData.split(" ")) #split by " " and remove empt spaces on list
            objectType = objectAndDataAsList[0]
            dataAsString = objectAndDataAsList[1:]
            data = []
            for i in dataAsString:
                data.append(int(i))
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
            else:
                self.occupancyGrid[y][x] += -decrease
                if self.occupancyGrid[y][x] < 0:
                    self.occupancyGrid[y][x] = 0
            
        for currentlyOccupiedCell in currentlyOccupied:
            x = currentlyOccupiedCell[0]
            y = currentlyOccupiedCell[1]
            self.occupancyGrid[y][x] += 25
            if self.occupancyGrid[y][x] > 100:
                self.occupancyGrid[y][x] = 100

    #def publish(self):
    #    pub = rospy.Publisher('gridPub', OccupancyGrid, queue_size=10)
    #    rate = rospy.Rate(10) #appropriate rate?
    #    while not rospy.is_shutdown():
    #       pub.publish(self.occupancyGrid)
    #       rate.sleep()

    def callback(data):
        self.mapAsString = data.data
        updateOccupancyGrid(self.mapAsString)
        #pub.publish(self.occupancyGrid)
        print self.occupancyGrid


    #def listen(self):
    #    rospy.Subscriber("machineVision", String, callback)
    #    # spin() simply keeps python from exiting until this node is stopped
    #    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('myName')
        grid = occupancyGrid()
        rospy.Subscriber("machineVision", String, grid.callback)
        pub = rospy.Publisher('gridPub', OccupancyGrid, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
       pass                
