#!/usr/bin/env python

"""  Neato drives parallel to nearest wall. """

from geometry_msgs.msg import Vector3, Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, ColorRGBA
import rospy
import math
import sys

class WallNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/wall_topic', Marker, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.closest_corner = None
        self.parallel = False
        self.ranges = []
        self.threshold = .07
        self.my_marker = None
        self.twist= None
        
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

    def go_forward(self):
        self.twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        self.publisher.publish(self.twist)

    def turn_left(self):
        if self.parallel:
            self.time = rospy.Time.now()
            return self.go_forward
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        self.publisher.publish(self.twist)
        return self.turn_left

    def turn_right(self):
        if self.is_parallel():
            self.time = rospy.Time.now()
            return self.go_forward
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -0.5))
        self.publisher.publish(self.twist)
        return self.turn_right

        
    def wall_viz(self, a, b, angle_a, angle_b):
        my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        root2 = math.sqrt(2)/2
        point1 = Point(a*math.cos(math.radians(angle_a)), a*math.sin(math.radians(angle_a)), 0)
        point2 = Point(b*math.cos(math.radians(angle_b)), b*math.sin(math.radians(angle_b)), 0)
        self.my_marker = Marker(header=my_header, type=4, points=[point1, point2], color=ColorRGBA(0.0, 0.0, 1.0, 1.0), scale=Vector3(0.2, 0.2, 0.2))


    def is_parallel(self):
        self.compute_closest_corner()
        if self.closest_corner == 0 or self.closest_corner == 1:
            # 10 degree range
            self.wall_viz(self.ranges[0][4], self.ranges[1][4], 45, 135)
            for i in range (0,9):
                if abs(self.ranges[0][i]-self.ranges[1][8-i]) < self.threshold and (self.ranges[0][i] != sys.maxint or self.ranges[1][8-i] != sys.maxint):
                    self.parallel = True
                    return
                    # find any equal ranges 90 degrees apart to determine if parallel
            self.parallel = False
                    
        elif self.closest_corner == 2 or self.closest_corner == 3:
            self.wall_viz(self.ranges[2][4], self.ranges[3][4], 225, 315)
            for i in range(0,9):                
                if abs(self.ranges[2][i]-self.ranges[3][8-i]) < self.threshold and (self.ranges[2][i] != sys.maxint or self.ranges[3][8-i] != sys.maxint):
                    self.parallel = True
                    return
            self.parallel = False

    def clean_data(self):
        if self.ranges != []:
            for i, array in enumerate(self.ranges):
                for j in range(len(array)):
                    if self.ranges[i][j] == 0.0:
                        self.ranges[i][j] = sys.maxint
            self.is_parallel()

    def compute_closest_corner(self):
        min_ranges = []
        maxint_counter = 0
        for i in range(0,4):
            minimum = min(self.ranges[i])
            min_ranges.append(minimum)
            if minimum == sys.maxint:
                maxint_counter += 1
            if maxint_counter < 3:
                self.closest_corner = min_ranges.index(min(min_ranges))

    def choose_action(self):
        if self.parallel:
            # go forward
            self.twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        elif self.closest_corner == 0 or self.closest_corner ==2:
            # turn right
            self.twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, -0.2))
        elif self.closest_corner == 1 or self.closest_corner == 3:
            # turn left
            self.twist = Twist(linear=Vector3(0.05,0.0,0.0), angular=Vector3(0.0,0.0,0.2))
        
    def run(self):
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            self.clean_data()
            self.choose_action()           
            if self.twist:
                self.publisher.publish(self.twist)
	    if self.my_marker:
            	self.marker_publisher.publish(self.my_marker)
            self.r.sleep()
        
            
wall_node = WallNode()
wall_node.run()
