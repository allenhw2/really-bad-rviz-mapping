#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Allen Wang
"""

from matplotlib import lines
import numpy as np
import math
import rospy
from nav_msgs.msg import *
from rospy.rostime import Duration
from sensor_msgs.msg import *
from geometry_msgs.msg import * 
from visualization_msgs.msg import Marker
from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score



class listenerNode():
    def __init__(self):
        self.loop_hertz = 10
        self.laser = None
        self.odometry = None
        self.ranges = None
        self.angles = None
        self.points = []
        self.lines = []

    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)
        name = "/terrasentia"
        rospy.Subscriber(name + "/scan", LaserScan, self.laserscan_callback)
        rospy.Subscriber(name + "/ekf", Odometry, self.odometry_callback)

        self.pub_lines = rospy.Publisher("/ans/lines", Marker, queue_size=1)


        while not rospy.is_shutdown():
            self.points.clear()
            if not self.laser is None:
                self.publish()
                self.plot()
            self.rate.sleep()
        
        self.plot()
    
    def plot(self):
        lineCollect = LineCollection(self.lines, colors=['r', 'g', 'b'])
        fig = plt.figure()
        ax1 = fig.add_subplot(1, 2, 1)
        ax1.add_collection(lineCollect)
        ax1.autoscale()
        ax1.set_title('Map')
        plt.show()

    def publish(self):
        # ans = self.fit_lines()
        self.fit_lines()
        # self.pub_lines.publish(ans)

    def convertToCart(self):
        self.ranges  = self.laser.ranges
        angles = np.linspace(self.laser.angle_min, self.laser.angle_max, len(self.ranges))
        
        for i in range(len(self.ranges)):
            if self.ranges[i] > 63:
                continue
            x = self.ranges[i] * math.cos(angles[i])
            y = self.ranges[i] * math.sin(angles[i])
            z = 0
            point = (x,y,z)
            self.points.append(point)


    def distance(self, x1,y1,x2,y2,x3,y3):
        x = np.array([x1,x2]).reshape((-1,1))
        y = np.array([y1,y2])

        # points = [(x1,y1),(x2,y2)]
        # x_coords, y_coords = zip(*points)
        # A = np.vstack([x_coords,np.ones(len(x_coords))]).T
        # m, c = np.linalg.lstsq(A, y_coords)[0]

        # point = np.array([x3,y3])

        # d = abs(y3 - m * x3 + c)
        # d = abs((coef[0]*point[0])-point[1]+coef[1])/math.sqrt((coef[0]*coef[0])+1)

        regression_model = LinearRegression()
        regression_model.fit(x,y)

        y_predicted = regression_model.predict(np.array([x3]).reshape(1,-1))

        deviation = abs(y_predicted[0] - y3)
        return deviation

    def mannhattanDistance(self,x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    def calculateLines(self, fitCoef = 0.1, gapCoef = 10):
        self.convertToCart()

        lines = []

        p1, p2, p3 = 0,1,2
        while p1 < len(self.points) and p2 < len(self.points) and p3 < len(self.points):
            x1,y1, z = self.points[p1]
            x2,y2, z = self.points[p2]
            x3,y3, z = self.points[p3]

            # if  self.mannhattanDistance(x2,y2, x3, y3) > 30:
            #     line = []
            #     point1 = (x1, y1)
            #     line.append(point1)

            #     point2 = (x2, y2)
            #     line.append(point2)
                    
            #     lines.append(line)
            #     index = p3
            #     p1 = p3
            #     p2 = p3 + 1
            #     p3 = p3 + 2
            #     continue
            
            d = abs(self.distance(x1,y1,x2,y2,x3,y3))

            if d > fitCoef:
                line = []
                point1 = (x1, y1)
                line.append(point1)
                point2 = (x2, y2)
                line.append(point2)
                    
                lines.append(line)
                index = p3
                p1 = p3
                p2 = p3 + 1
                p3 = p3 + 2

                continue

            p2 = p3
            p3 = p3 + 1
            

            
        # for p1 in range(1, len(self.points) - 3):
        #     for p2 in range(p1 + 1, len(self.points) - 1):
        #         p3 = p2 + 1

        #         x1,y1, z = self.points[p1]
        #         x2,y2, z = self.points[p2]
        #         x3,y3, z = self.points[p3]

        #         if self.ranges[p1] > 60 or self.ranges[p2] > 60:
        #             p1 = p3
                
        #         d = abs(self.distance(x1,y1,x2,y2,x3,y3))

        #         if d > fitCoef:
        #             line = []
        #             point1 = (x1, y1)
        #             line.append(point1)

        #             point2 = (x2, y2)
        #             line.append(point2)
                    
        #             lines.append(line)
        #             p1 = p2
        #             break
        self.lines = lines
        return lines


    def fit_lines(self):
        # marker = Marker()
        # marker.type = Marker.LINE_LIST
        # marker.action = Marker.ADD
        # marker.header.frame_id = "laser"
        # marker.ns = "line"
        # marker.scale.x = 0.03
        # marker.color.r = 1.0
        # marker.color.a = 1.0
        # marker.lifetime = Duration(secs=0.0)
        # marker.frame_locked = False
        # marker.points = self.calculateLines(0.1)
        self.calculateLines(0.01)
        # return marker
    
    def laserscan_callback(self, msg):
        self.laser = msg
    def odometry_callback(self, msg):
        self.odometry = msg
    
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('listener', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()