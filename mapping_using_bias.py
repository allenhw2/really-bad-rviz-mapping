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

        # there was an attempt to use markers in rviz, but it got complicated real fast
        #self.pub_lines = rospy.Publisher("/ans/lines", Marker, queue_size=1)

        while not rospy.is_shutdown():
            self.points.clear()
            if not self.laser is None:
                self.publish()
            self.rate.sleep()
        
        self.plot()

    def publish(self):
        # ans = self.fit_lines()
        self.calculateLines(0.005)
        # self.pub_lines.publish(ans)

    def convertToCart(self):
        self.ranges  = self.laser.ranges
        angles = np.linspace(self.laser.angle_min, self.laser.angle_max, len(self.ranges))

        baseX = self.odometry.pose.pose.position.x
        baseY = self.odometry.pose.pose.position.y

        quatX = self.odometry.pose.pose.orientation.x
        quatY = self.odometry.pose.pose.orientation.y
        quatZ = self.odometry.pose.pose.orientation.z
        quatW = self.odometry.pose.pose.orientation.w
        
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(quatX, quatY, quatZ, quatW)

        for i in range(len(self.ranges)):
            if self.ranges[i] > 63:
                continue
            
            x = baseX + (self.ranges[i] * math.cos(yaw_z + angles[i]))
            y = baseY + (self.ranges[i] * math.sin(yaw_z + angles[i]))
            z = 0
            point = (x,y,z)
            self.points.append(point)


    def distance(self, x1,y1,x2,y2,x3,y3):
        x = np.array([x1,x2]).reshape((-1,1))
        y = np.array([y1,y2])

        regression_model = LinearRegression()
        regression_model.fit(x,y)

        y_predicted = regression_model.predict(np.array([x3]).reshape(1,-1))

        deviation = abs(y_predicted[0] - y3)
        return deviation

    def mannhattanDistance(self,x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    def calculateLines(self, fitCoef = 0.1, gapCoef = 10):
        self.convertToCart()

        p1, p2, p3 = 0,1,2
        while p1 < len(self.points) and p2 < len(self.points) and p3 < len(self.points):
            x1,y1, z = self.points[p1]
            x2,y2, z = self.points[p2]
            x3,y3, z = self.points[p3]
            
            d = abs(self.distance(x1,y1,x2,y2,x3,y3))

            if d > fitCoef:
                line = []
                point1 = (x1, y1)
                line.append(point1)
                point2 = (x2, y2)
                line.append(point2)
                    
                self.lines.append(line)
                index = p2
                p1 = p2
                p2 = p3 
                p3 = p3 + 1

                continue

            p2 = p3
            p3 = p3 + 1
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def laserscan_callback(self, msg):
        self.laser = msg
    def odometry_callback(self, msg):
        self.odometry = msg
    
    def plot(self):
        lineCollect = LineCollection(self.lines, colors=['b'])
        fig = plt.figure()
        ax1 = fig.add_subplot(1, 2, 1)
        ax1.add_collection(lineCollect)
        ax1.autoscale()
        ax1.set_title('Map')
        plt.show()
    
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('listener', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()