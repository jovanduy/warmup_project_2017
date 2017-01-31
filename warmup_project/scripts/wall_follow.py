#!/usr/bin/env python

"""  Neato drives parallel to nearest wall. """

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
import rospy
import math
import sys

class WallNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.closest_corner = None
        self.parallel = False
        self.ranges = []
        self.threshold = .07

    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    
    def process_scan(self, m):
        front_left = []
        front_right = []
        back_left = []
        back_right = []
        for i in range (0,9):
            front_left.append(m.ranges[41+i])
            back_left.append(m.ranges[131+i])
            back_right.append(m.ranges[221+i])
            front_right.append(m.ranges[311+i])
         
        self.ranges = [front_left, back_left, back_right, front_right] 


    def is_parallel(self):
        if self.closest_corner == 0 or self.closest_corner == 1:
            # 10 degree range
            for i in range (0,9):
                if abs(self.ranges[0][i]-self.ranges[1][8-i]) < self.threshold and (self.ranges[0][i] != sys.maxint or self.ranges[1][8-i] != sys.maxint):
                    self.parallel = True
                    return
                    # find any equal ranges 90 degrees apart to determine if parallel
                else:
                    self.parallel = False
        


        elif self.closest_corner == 2 or self.closest_corner == 3:
            for i in range(0,9):
                
                if abs(self.ranges[2][i]-self.ranges[3][8-i]) < self.threshold and (self.ranges[2][i] != sys.maxint or self.ranges[3][8-i] != sys.maxint):
                    self.parallel = True
                    return
                else:
                    self.parallel = False
            
        return   

    def run(self):
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if self.ranges != []:

                for i, array in enumerate(self.ranges):
                    for j in range(len(array)):
                        if self.ranges[i][j] == 0.0:
                            self.ranges[i][j] = sys.maxint

                print self.ranges
                min_ranges = []
                maxint_counter = 0
                for i in range(0,4):
                    minimum = min(self.ranges[i])
                    min_ranges.append(minimum)
                    if minimum == sys.maxint:
                        maxint_counter += 1
                if maxint_counter >= 3:
                    pass
                print 'min_ranges'
                print min_ranges
                self.closest_corner = min_ranges.index(min(min_ranges))
                print self.closest_corner
                self.is_parallel()
                if self.parallel:
                    # go forward
                    twist= Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))


                elif self.closest_corner == 0 or self.closest_corner == 2:
                    twist = Twist(linear=Vector3(0.05,0.0,0.0), angular=Vector3(0.0,0.0,-0.2))
                    # 0 is the front right corner
                    # am I parallel to the right?

                    #turn right
                elif self.closest_corner == 1 or self.closest_corner == 3:

                    # 0 is the back right corner
                    # am I parallel to the right?
                    #turn left
                    twist = Twist(linear=Vector3(0.05,0.0,0.0), angular=Vector3(0.0,0.0,0.2))

   

                self.publisher.publish(twist)
            self.r.sleep()
        
            
wall_node = WallNode()
wall_node.run()
