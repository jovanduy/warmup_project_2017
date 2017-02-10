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
    """
    Class for a wall follower node
    """
    def __init__(self):
        """
        Initialize a WallNode object and subscribe to the /scan topic.
        Create publishers to publish /cmd_vel Twists and Markers
        representing the detected wall being followed.
        """
        rospy.init_node('wall_follow')
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/wall_topic', Marker, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.closest_corner = None
        self.parallel = False
        self.ranges = []
        self.threshold = .07 # max distance difference (m) for being equidistant
        self.my_marker = None
        self.twist= None
        
    def stop(self):
        """
        Publish a /cmd_vel Twist corresponding to not moving
        """
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    
    def process_scan(self, m):
        """
        Callback function whenever a /scan topic is received.
        Save the scan readings from the four corners of the Neato
        to a list of lists as self.ranges

        self.ranges will have readings from:
        [front left, back left, back right, front right]
        Where every corner is represented as a list of the 9 readings
        surrounding each corner (ex: front left reads from
        41 degrees through  49 degrees)
        """
        
        front_left = []
        front_right = []
        back_left = []
        back_right = []
        # use 9 different degrees per corner
        # because readings can contain bad data
        for i in range (0,9):
            front_left.append(m.ranges[41+i])
            back_left.append(m.ranges[131+i])
            back_right.append(m.ranges[221+i])
            front_right.append(m.ranges[311+i])
        self.ranges = [front_left, back_left, back_right, front_right] 

    def wall_viz(self, a, b, angle_a, angle_b):
        """
        Save a marker of type LINE_STRIP (4) to self.my_marker.

        a:       polar radius coordinate of first endpoint of marker
        b:       polar radius coordinate of second endpoint of marker
        angle_a: polar angle coordinate of first endpoint of marker
        angle_b: polar angle coordinate of second endpoint of marker
        """

        my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")

        # inputs are in polar relative to base_link
        # convert to Cartesian to publish marker
        root2 = math.sqrt(2)/2
        point1 = Point(a*math.cos(math.radians(angle_a)), a*math.sin(math.radians(angle_a)), 0)
        point2 = Point(b*math.cos(math.radians(angle_b)), b*math.sin(math.radians(angle_b)), 0)
        self.my_marker = Marker(header=my_header, type=4, points=[point1, point2], color=ColorRGBA(0.0, 0.0, 1.0, 1.0), scale=Vector3(0.2, 0.2, 0.2))


    def is_parallel(self):
        """
        Detects if Neato is parallel to the detected wall.
        Sets self.parallel to True or False
        """

        self.compute_closest_corner()
        if self.closest_corner == 0 or self.closest_corner == 1:
            # wall is on the left, compare
            # two left corners' laser readings

            # create marker representing the wall
            self.wall_viz(self.ranges[0][4], self.ranges[1][4], 45, 135)

            # always compare the two angles equidistant
            # from middle of the Neato's side
            for i in range (0,9):
                if abs(self.ranges[0][i]-self.ranges[1][8-i]) < self.threshold and (self.ranges[0][i] != sys.maxint or self.ranges[1][8-i] != sys.maxint):
                    self.parallel = True
                    return
                    # find any equal ranges 90 degrees apart to determine if parallel
            self.parallel = False
                    
        elif self.closest_corner == 2 or self.closest_corner == 3:
            # wall is on the right, compare
            # two right corners' laser readings
            self.wall_viz(self.ranges[2][4], self.ranges[3][4], 225, 315)
            for i in range(0,9):                
                if abs(self.ranges[2][i]-self.ranges[3][8-i]) < self.threshold and (self.ranges[2][i] != sys.maxint or self.ranges[3][8-i] != sys.maxint):
                    self.parallel = True
                    return
            self.parallel = False

    def clean_data(self):
        """
        Set all "bad" data (0.0 readings) to maxint
        and determine if parallel to a wall
        """
        if self.ranges != []:
            for i, array in enumerate(self.ranges):
                for j in range(len(array)):
                    if self.ranges[i][j] == 0.0:
                        # need to set bad 0s to maxint
                        # because closest corner is calculated
                        # by lowest reading
                        self.ranges[i][j] = sys.maxint
            self.is_parallel()

    def compute_closest_corner(self):
        """
        Set self.closest_corner to int representing a corner
        0 = front left, 1 = back left, 2 = back right, 3 = front right
        """
        min_ranges = []
        maxint_counter = 0
        for i in range(0,4):
            minimum = min(self.ranges[i])
            min_ranges.append(minimum)
            if minimum == sys.maxint:
                maxint_counter += 1
        if maxint_counter < 3:
            # do not update if there is too much bad data
            self.closest_corner = min_ranges.index(min(min_ranges))

    def choose_action(self):
        """
        Set self.twist to appropriate action
        to move Neato along the wall
        """
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
        """
        Main run method
        """
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            self.clean_data()
            self.choose_action()           
            if self.twist:
                self.publisher.publish(self.twist)
	    if self.my_marker:
            	self.marker_publisher.publish(self.my_marker)
            self.r.sleep()
        
if __name__ == "__main__":
    wall_node = WallNode()
    wall_node.run()
