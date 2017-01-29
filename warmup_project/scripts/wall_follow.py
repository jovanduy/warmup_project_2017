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

    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    
    def process_scan(self, m):
         self.ranges = [m.ranges[45], m.ranges[135], m.ranges[225], m.ranges[315]] #Front right, back right, back left, front left


    def is_parallel(self):
        if self.closest_corner == 0 or self.closest_corner == 1:

            if abs(self.ranges[0]-self.ranges[1]) < .25:
                self.parallel = True
            else:
                self.parallel = False
        elif self.closest_corner == 2 or self.closest_corner == 3:
            if abs(self.ranges[2]-self.ranges[3]) < .25:
                self.parallel = True
            else:
                self.parallel = False
        
            

    def run(self):
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if self.ranges != []:

                for i in range(len(self.ranges)):
                    if self.ranges[i] == 0.0:
                        self.ranges[i] = sys.maxint

                print self.ranges
                self.closest_corner = self.ranges.index(min(self.ranges))
                self.is_parallel()
                if self.parallel:
                    # go forward
                    twist= Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))


                elif self.closest_corner == 0 or self.closest_corner == 2:
                    twist = Twist(linear=Vector3(0.0,0.0,0.0), angular=Vector3(0.0,0.0,-0.4))
                    # 0 is the front right corner
                    # am I parallel to the right?

                    #turn right
                elif self.closest_corner == 1 or self.closest_corner == 3:

                    # 0 is the back right corner
                    # am I parallel to the right?
                    #turn left
                    twist = Twist(linear=Vector3(0.0,0.0,0.0), angular=Vector3(0.0,0.0,0.4))

   

                self.publisher.publish(twist)
            self.r.sleep()
        
            
wall_node = WallNode()
wall_node.run()
