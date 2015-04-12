import rospy
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg #check if this is possible

import numpy as np
import re


# look at nav-msgs for occupancy grid like structure
class occupancyGrid():
    def __init__(self):
        # n rows, m columns
        self.n = 5 # set this manually
        self.m = 10 # set this manually
        self.grid = np.zeros((self.n,self.m))
        self.occupancyGrid = np.zeros((self.n,self.m))
        self.mapAsString = ""

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
        newGrid = self.generateBlankOccupancyGrid()
        for typeOfObject in parsedMap:
            for currentObject in parsedMap[typeOfObject]:
                x = currentObject[0]
                y = currentObject[1]
                width = currentObject[2]
                length = currentObject[3]
                newGrid[y][x] = 1
                for i in range(width + 1):
                    for j in range(length + 1):
                        newGrid[y + i][x + j] = 1
        return newGrid

    def generateBlankOccupancyGrid(self):
        # n rows, m columns
        newGrid = np.zeros((self.n,self.m))
        return newGrid


    def updateOccupancyGrid(self, mapping):
        # n rows, m columns
        increase = 0.25
        decrease = 0.4
        currentlyOccupied = self.convertMappingToOccupancyGrid(mapping)
        for j in range(self.m):
            for i in range(self.n):
                currentCell = currentlyOccupied[i][j]
                if currentCell == 1:
                    #print i
                    #print j
                    #print "done"
                    self.occupancyGrid[i][j] += 25
                else:
                    self.occupancyGrid[i][j] += -40
                if self.occupancyGrid[i][j] > 100:
                    self.occupancyGrid[i][j] = 100
                if self.occupancyGrid[i][j] < 0:
                    self.occupancyGrid[i][j] = 0

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