#!/usr/bin/env python

"""  Neato follows person around. """

from geometry_msgs.msg import Vector3, Twist, Point, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header, ColorRGBA
from person_follow import PersonNode
from drive_square import SquareNode
import rospy
import math
import sys

class FSM_Node(object):
    def __init__(self):
        self.r = rospy.Rate(5)
        self.person_follower = PersonNode()
        self.drive_square = SquareNode()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.twist = None
        
    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))

    def process_scan(self, m):
        self.person_follower.process_scan(m)
        self.person_follower.find_person()
    
    def run(self):
        rospy.on_shutdown(self.stop)
        curr_state = self.drive_square.go_forward

        while not rospy.is_shutdown():
            if 1.25 > self.person_follower.person_polar[1] > 0:
                # there is a person
                curr_state = self.person_follower.calculate_velocity
                self.twist = self.person_follower.twist
            elif curr_state == self.person_follower.calculate_velocity:
                # just lost person, begin square
                curr_state = self.drive_square.go_forward
                self.twist = self.drive_square.twist
            else:
                # continue square
                self.twist = self.drive_square.twist
            if curr_state:
                print curr_state
                curr_state = curr_state()
            self.person_follower.visualize()
            if self.person_follower.my_marker != 0:
                
                self.person_follower.marker_publisher.publish(self.person_follower.my_marker)
                print self.person_follower.person_polar

            if self.twist:
                self.publisher.publish(self.twist)
            
            self.r.sleep()
        
if __name__ == '__main__':            
    rospy.init_node('FSM')
    fsm = FSM_Node()
    fsm.run()
